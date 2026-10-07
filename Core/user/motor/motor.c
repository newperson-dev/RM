/**
  ******************************************************************************
  * @file    motor.c
  * @brief   4 路电机控制实现：PWM + 方向 + 编码器 + 速度/位置双环 PID
  *
  * 线程约定（很重要）：
  *   - **所有电机硬件操作都只发生在 MotorTask 里**（PWM/方向/编码器计数器清零）；
  *     其他任务（CommTask）只写"请求标志 + 32 位整型参数"，由 MotorTask 在下个
  *     周期执行，所以不存在两个任务同时写 CCR/方向线的竞态。
  *   - Motor_SetXxx() 只允许任务上下文调用（内部用了 taskENTER_CRITICAL），
  *     绝对不要放进中断里。
 *   - 闭环目标斜坡（MOTOR_TARGET_SLEW_CPS_PER_S）只在 MotorTask 内部生效：
 *     共享的 target_cps 仍是“指令值”，斜坡后的 ramped_cps 是任务私有状态。
  *   - 成组的参数（3 个增益）用一小段临界区保证"三个一起生效"。
  ******************************************************************************
  */
#include "motor.h"
#include "../common/controller.h"
#include "../common/control_time.h"
#include "tim.h"

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

/* ========================== 硬件映射 ========================== */

typedef struct
{
  TIM_HandleTypeDef *pwm_tim;
  uint32_t           pwm_ch;
  GPIO_TypeDef      *in1_port;
  uint16_t           in1_pin;
  GPIO_TypeDef      *in2_port;
  uint16_t           in2_pin;
  TIM_HandleTypeDef *enc_tim;
} MotorHw_t;

static const MotorHw_t s_hw[MOTOR_NUM] =
{
  { &htim1, TIM_CHANNEL_1, Motor1GPIOA_GPIO_Port, Motor1GPIOA_Pin, Motor1GPIOB_GPIO_Port, Motor1GPIOB_Pin, &htim2 },
  { &htim1, TIM_CHANNEL_2, Motor2GPIOA_GPIO_Port, Motor2GPIOA_Pin, Motor2GPIOB_GPIO_Port, Motor2GPIOB_Pin, &htim3 },
  { &htim1, TIM_CHANNEL_3, Motor3GPIOA_GPIO_Port, Motor3GPIOA_Pin, Motor3GPIOB_GPIO_Port, Motor3GPIOB_Pin, &htim4 },
  { &htim1, TIM_CHANNEL_4, Motor4GPIOA_GPIO_Port, Motor4GPIOA_Pin, Motor4GPIOB_GPIO_Port, Motor4GPIOB_Pin, &htim5 },
};

/** 某个电机"正方向"与预期相反时把对应位置 1：PWM 输出与编码器测速**同时**翻转，
 *  相当于把该电机的正方向整体镜像，开环/闭环下都稳定（闭环正反馈的前提是只翻一侧）。
 *  注意：这只定义"该电机的正方向"；底盘前后左右的方向标定在 chassis.c 的 s_dir[]。 */
static const uint8_t s_invert[MOTOR_NUM] = { 0U, 0U, 0U, 0U };

/* ========================== 内部状态 ========================== */

typedef struct
{
  /* ---------- 跨任务共享：参数（只写 32 位整型） ---------- */
  volatile int32_t  target_cps;            /* 速度环给定：计数/秒 */
  volatile int32_t  kp_milli;              /* 速度环增益 */
  volatile int32_t  ki_milli;
  volatile int32_t  kd_milli;
  volatile uint32_t closed_loop;           /* 速度环使能 */

  volatile int32_t  pos_target_milli_deg;  /* 位置环给定：度 ×1000 */
  volatile int32_t  p_kp_milli;            /* 位置环增益 */
  volatile int32_t  p_ki_milli;
  volatile int32_t  p_kd_milli;
  volatile uint32_t pos_loop_en;           /* 位置环使能 */

  volatile uint32_t last_cmd_tick;

  /* ---------- 跨任务共享：一次性请求（由 MotorTask 消费） ---------- */
  volatile int32_t  open_pwm_req;          /* 开环 PWM 请求值 */
  volatile uint32_t open_pwm_pending;      /* 1 = 有一个待执行的开环请求 */
  volatile uint32_t enc_reset_req;         /* 1 = 请求编码器清零（位置环定零点用） */
  volatile uint32_t pid_reset_req;         /* 1 = 请求重置两个 PID 的积分 */

  /* ---------- 仅 MotorTask 访问 ---------- */
  PID_t    speed_pid;
  PID_t    pos_pid;
  int32_t  applied_kp_milli, applied_ki_milli, applied_kd_milli;
  int32_t  applied_p_kp_milli, applied_p_ki_milli, applied_p_kd_milli;
  uint32_t pos_loop_active;

  int16_t  enc_prev;
  int32_t  enc_total;
  int16_t  enc_delta;
  int32_t  ramped_cps;   /* 斜坡后的闭环目标（仅 MotorTask 使用），见 Motor_UpdateAll 第 8 步 */
  float    speed_cps;
  float    speed_rpm;
  float    angle_deg;
  int32_t  out_pwm;
} Motor_t;

static Motor_t  s_m[MOTOR_NUM];
static uint32_t s_pwm_max = MOTOR_PWM_MAX_DEFAULT;

/* ========================== 内部函数 ========================== */

/** 真正写硬件：方向 + 占空比。顺序是"先清零 -> 再换向 -> 最后给占空比"。
 *  只在 MotorTask（或调度器启动前的 Motor_Init）里调用。 */
static void motor_apply(MotorId_t id, int32_t pwm)
{
  const MotorHw_t *hw = &s_hw[id];
  Motor_t         *m  = &s_m[id];
  uint32_t         duty;

  if (pwm > (int32_t)s_pwm_max)  { pwm = (int32_t)s_pwm_max; }
  if (pwm < -(int32_t)s_pwm_max) { pwm = -(int32_t)s_pwm_max; }
  if (s_invert[id] != 0U)        { pwm = -pwm; }

  duty = (uint32_t)((pwm < 0) ? -pwm : pwm);

  /* 1. 先关占空比，避免换向瞬间"新方向 + 旧占空比"的电流冲击 */
  __HAL_TIM_SET_COMPARE(hw->pwm_tim, hw->pwm_ch, 0U);

  /* 2. 再换向 */
  if (pwm > 0)
  {
    HAL_GPIO_WritePin(hw->in1_port, hw->in1_pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(hw->in2_port, hw->in2_pin, GPIO_PIN_RESET);
  }
  else if (pwm < 0)
  {
    HAL_GPIO_WritePin(hw->in1_port, hw->in1_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(hw->in2_port, hw->in2_pin, GPIO_PIN_SET);
  }
  else
  {
    /* 两个输入都拉低 = 滑行(coast)。想要刹车(brake)就把两个都置 SET */
    HAL_GPIO_WritePin(hw->in1_port, hw->in1_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(hw->in2_port, hw->in2_pin, GPIO_PIN_RESET);
  }

  /* 3. 最后给占空比 */
  __HAL_TIM_SET_COMPARE(hw->pwm_tim, hw->pwm_ch, duty);
  m->out_pwm = (s_invert[id] != 0U) ? -pwm : pwm;
}

#if (MOTOR_TARGET_SLEW_CPS_PER_S > 0)
/** 斜坡推进一步：把 cur 朝 target 推进最多 step（正数，单位 cps）。
 *  差值用 int64 计算——`SPD` 命令没有范围校验（手册 7.1 的已知限制），目标可能接近 INT32_MAX，
 *  直接 int32 累加会溢出回绕，反而把电机推向反方向。只在 MotorTask 里调用。
 *  （斜坡宏设为 0 时本函数不参与编译，避免"静态函数未使用"警告。） */
static int32_t motor_slew_step(int32_t cur, int32_t target, int32_t step)
{
  int64_t diff = (int64_t)target - (int64_t)cur;

  if (diff > (int64_t)step)
  {
    return cur + step;      /* 只在差值大于一步时前进，故不会越过 target、也不会溢出 */
  }
  if (diff < -(int64_t)step)
  {
    return cur - step;
  }
  return target;            /* 剩余差值不足一步：直接对齐 */
}
#endif

/* ========================== 初始化 ========================== */

void Motor_Init(void)
{
  uint32_t i;

  /* PWM 满量程直接取 TIM1 的 ARR，MX 里改频率不用改代码 */
  if (htim1.Init.Period != 0U)
  {
    s_pwm_max = htim1.Init.Period;
  }

  /* 速度环：输出就是 PWM，所以 MaxOut / IntegralLimit 必须等于 PWM 满量程。
     注意 MaxOut 一旦 <= 0，PID_Calc 会直接返回 0（库的保护），
     所以这个值只能等到这里读到 ARR 之后再确定。 */
  PID_Config_t speed_cfg =
  {
    .Kp = 0.0f,
    .Ki = 0.0f,
    .Kd = 0.0f,
    .MaxOut = (float)s_pwm_max,
    .DeadBand = 0.0f,
    .Improve = (PID_Improve_e)(PID_Integral_Limit |
                               PID_Trapezoid_Integral |
                               PID_Derivative_On_Measurement |
                               PID_ChangingIntegrationRate |
                               PID_DerivativeFilter),
    .IntegralLimit = (float)s_pwm_max,
    .CoefA = (float)MOTOR_SPEED_VARINT_COEF_A,
    .CoefB = (float)MOTOR_SPEED_VARINT_COEF_B,
    .Output_LPF_RC = 0.0f,
    .Derivative_LPF_RC = MOTOR_DERIVATIVE_LPF_RC_S
  };

  /* 位置环：输出是"速度给定"，所以 MaxOut 用速度上限；测量/给定单位是度 */
  PID_Config_t pos_cfg =
  {
    .Kp = 0.0f,
    .Ki = 0.0f,
    .Kd = 0.0f,
    .MaxOut = (float)MOTOR_MAX_SPEED_CPS,
    .DeadBand = 0.0f,
    .Improve = (PID_Improve_e)(PID_Integral_Limit |
                               PID_Trapezoid_Integral |
                               PID_Derivative_On_Measurement),
    .IntegralLimit = (float)MOTOR_MAX_SPEED_CPS,
    .CoefA = 0.0f,
    .CoefB = 0.0f,
    .Output_LPF_RC = 0.0f,
    .Derivative_LPF_RC = 0.0f
  };

  for (i = 0U; i < MOTOR_NUM; i++)
  {
    Motor_t *m = &s_m[i];

    m->target_cps           = 0;
    m->kp_milli             = MOTOR_DEFAULT_KP_MILLI;
    m->ki_milli             = MOTOR_DEFAULT_KI_MILLI;
    m->kd_milli             = MOTOR_DEFAULT_KD_MILLI;
    m->closed_loop          = 0U;
    m->pos_target_milli_deg = 0;
    m->p_kp_milli           = 0;
    m->p_ki_milli           = 0;
    m->p_kd_milli           = 0;
    m->pos_loop_en          = 0U;
    m->last_cmd_tick        = HAL_GetTick();

    m->open_pwm_req         = 0;
    m->open_pwm_pending     = 0U;
    m->enc_reset_req        = 0U;
    m->pid_reset_req        = 0U;

    /* 故意设成不可能相等的值，保证第一次就下发增益 */
    m->applied_kp_milli     = 0x7FFFFFFF;
    m->applied_ki_milli     = 0x7FFFFFFF;
    m->applied_kd_milli     = 0x7FFFFFFF;
    m->applied_p_kp_milli   = 0x7FFFFFFF;
    m->applied_p_ki_milli   = 0x7FFFFFFF;
    m->applied_p_kd_milli   = 0x7FFFFFFF;
    m->pos_loop_active      = 0U;

    m->enc_prev             = 0;
    m->enc_total            = 0;
    m->enc_delta            = 0;
    m->ramped_cps           = 0;
    m->speed_cps            = 0.0f;
    m->speed_rpm            = 0.0f;
    m->angle_deg            = 0.0f;
    m->out_pwm              = 0;

    PID_Init(&m->speed_pid, &speed_cfg);
    PID_Init(&m->pos_pid,   &pos_cfg);

    /* 启动 PWM 输出（MX 只做了 Init，没有 Start） */
    (void)HAL_TIM_PWM_Start(s_hw[i].pwm_tim, s_hw[i].pwm_ch);

    /* 启动编码器接口 */
    (void)HAL_TIM_Encoder_Start(s_hw[i].enc_tim, TIM_CHANNEL_ALL);
    __HAL_TIM_SET_COUNTER(s_hw[i].enc_tim, 0U);

    motor_apply((MotorId_t)i, 0);
  }
}

/* ========================== 指令接口（只写请求，不碰硬件） ========================== */

void Motor_SetPwm(MotorId_t id, int32_t pwm)
{
  if ((uint32_t)id >= MOTOR_NUM)
  {
    return;
  }

  /* 先写值、再写标志：这样 MotorTask 要么看到"没有请求"，要么看到完整的新值 */
  s_m[id].open_pwm_req      = pwm;
  s_m[id].closed_loop       = 0U;    /* 开环优先：两个环都退出 */
  s_m[id].pos_loop_en       = 0U;
  s_m[id].target_cps        = 0;
  s_m[id].pid_reset_req     = 1U;    /* 下次闭环从零积分开始，避免突然冲一下 */
  s_m[id].last_cmd_tick     = HAL_GetTick();
  s_m[id].open_pwm_pending  = 1U;
}

void Motor_SetTargetSpeed(MotorId_t id, int32_t counts_per_sec)
{
  if ((uint32_t)id >= MOTOR_NUM)
  {
    return;
  }

  s_m[id].target_cps    = counts_per_sec;
  s_m[id].closed_loop   = 1U;
  s_m[id].last_cmd_tick = HAL_GetTick();
}

uint8_t Motor_SetTargetSpeedRpm(MotorId_t id, float rpm)
{
  if ((uint32_t)id >= MOTOR_NUM)
  {
    return 0U;
  }
  if ((MOTOR_ENCODER_CALIBRATED == 0U) || (MOTOR_COUNTS_PER_REV <= 0.0f))
  {
    return 0U;      /* 未标定：物理单位不可信，直接拒绝 */
  }

  Motor_SetTargetSpeed(id, (int32_t)(rpm * MOTOR_COUNTS_PER_REV / 60.0f));
  return 1U;
}

void Motor_SetPidMilli(MotorId_t id, int32_t kp_milli, int32_t ki_milli, int32_t kd_milli)
{
  if ((uint32_t)id >= MOTOR_NUM)
  {
    return;
  }

  taskENTER_CRITICAL();               /* 三个值必须一起生效 */
  s_m[id].kp_milli = kp_milli;
  s_m[id].ki_milli = (ki_milli < 0) ? 0 : ki_milli;   /* 积分增益不允许为负 */
  s_m[id].kd_milli = kd_milli;
  taskEXIT_CRITICAL();
}

void Motor_SetPid(MotorId_t id, float kp, float ki, float kd)
{
  Motor_SetPidMilli(id,
                    (int32_t)(kp * 1000.0f),
                    (int32_t)(ki * 1000.0f),
                    (int32_t)(kd * 1000.0f));
}

void Motor_EnableClosedLoop(MotorId_t id, uint32_t enable)
{
  if ((uint32_t)id >= MOTOR_NUM)
  {
    return;
  }

  s_m[id].closed_loop = (enable != 0U) ? 1U : 0U;
  if (enable == 0U)
  {
    s_m[id].target_cps = 0;
    s_m[id].pid_reset_req = 1U;
  }
}

uint8_t Motor_SetTargetAngle(MotorId_t id, float deg)
{
  if ((uint32_t)id >= MOTOR_NUM)
  {
    return 0U;
  }
  if (MOTOR_ENCODER_CALIBRATED == 0U)
  {
    return 0U;      /* 未标定：位置环拒绝工作，防止按错误的比例乱转 */
  }

  s_m[id].pos_target_milli_deg = (int32_t)(deg * 1000.0f);
  s_m[id].pos_loop_en          = 1U;
  s_m[id].closed_loop          = 1U;   /* 位置环由速度环执行，自动带上 */
  s_m[id].last_cmd_tick        = HAL_GetTick();
  return 1U;
}

void Motor_EnablePositionLoop(MotorId_t id, uint32_t enable)
{
  if ((uint32_t)id >= MOTOR_NUM)
  {
    return;
  }
  if ((enable != 0U) && (MOTOR_ENCODER_CALIBRATED == 0U))
  {
    return;      /* 未标定不允许打开 */
  }

  s_m[id].pos_loop_en = (enable != 0U) ? 1U : 0U;
  if (enable != 0U)
  {
    s_m[id].closed_loop = 1U;
  }
  else
  {
    s_m[id].pid_reset_req = 1U;
  }
}

void Motor_SetPosPidMilli(MotorId_t id, int32_t kp_milli, int32_t ki_milli, int32_t kd_milli)
{
  if ((uint32_t)id >= MOTOR_NUM)
  {
    return;
  }

  taskENTER_CRITICAL();
  s_m[id].p_kp_milli = kp_milli;
  s_m[id].p_ki_milli = (ki_milli < 0) ? 0 : ki_milli;
  s_m[id].p_kd_milli = kd_milli;
  taskEXIT_CRITICAL();
}

void Motor_SetPosPid(MotorId_t id, float kp, float ki, float kd)
{
  Motor_SetPosPidMilli(id,
                       (int32_t)(kp * 1000.0f),
                       (int32_t)(ki * 1000.0f),
                       (int32_t)(kd * 1000.0f));
}

void Motor_ResetCount(MotorId_t id)
{
  if ((uint32_t)id >= MOTOR_NUM)
  {
    return;
  }

  /* 只置请求：真正的清零由 MotorTask 完成，避免与它读编码器打架 */
  s_m[id].enc_reset_req = 1U;
}

void Motor_ResetAllCounts(void)
{
  uint32_t i;

  for (i = 0U; i < MOTOR_NUM; i++)
  {
    s_m[i].enc_reset_req = 1U;
  }
}

void Motor_StopAll(void)
{
  uint32_t i;

  for (i = 0U; i < MOTOR_NUM; i++)
  {
    s_m[i].target_cps       = 0;
    s_m[i].closed_loop      = 0U;
    s_m[i].pos_loop_en      = 0U;
    s_m[i].pid_reset_req    = 1U;
    s_m[i].open_pwm_req     = 0;
    s_m[i].open_pwm_pending = 1U;
  }
}

/* ========================== 周期控制 ========================== */

void Motor_UpdateAll(float dt)
{
  uint32_t i;

  if ((dt <= 0.0f) || (dt > 0.2f))
  {
    dt = (float)MOTOR_PERIOD_MS / 1000.0f;   /* dt 异常时退回标称周期 */
  }

  for (i = 0U; i < MOTOR_NUM; i++)
  {
    Motor_t *m = &s_m[i];
    int32_t  kp_m, ki_m, kd_m, pkp_m, pki_m, pkd_m;
    int32_t  speed_target_cps, pos_target_milli;
    int32_t  open_pwm;
    uint32_t closed_loop, pos_loop_en, open_pending, enc_reset, pid_reset;
    int16_t  now, delta;
    float    raw_speed, alpha;

    /* ---- 1. 取指令快照。临界区保证：增益三件套一致 + 请求"读到即消费"不丢失 ---- */
    taskENTER_CRITICAL();
    kp_m  = m->kp_milli;
    ki_m  = m->ki_milli;
    kd_m  = m->kd_milli;
    pkp_m = m->p_kp_milli;
    pki_m = m->p_ki_milli;
    pkd_m = m->p_kd_milli;
    speed_target_cps = m->target_cps;
    pos_target_milli = m->pos_target_milli_deg;
    closed_loop      = m->closed_loop;
    pos_loop_en      = m->pos_loop_en;

    open_pending = m->open_pwm_pending;
    open_pwm     = m->open_pwm_req;
    m->open_pwm_pending = 0U;

    enc_reset = m->enc_reset_req;
    m->enc_reset_req = 0U;

    pid_reset = m->pid_reset_req;
    m->pid_reset_req = 0U;
    taskEXIT_CRITICAL();

    /* ---- 2. 处理"重置积分"请求 ---- */
    if (pid_reset != 0U)
    {
      PID_Reset(&m->speed_pid);
      PID_Reset(&m->pos_pid);
      m->ramped_cps = 0;         /* 重新使能闭环时目标从 0 开始斜坡，避免跳变 */
    }

    /* ---- 3. 增益变化时才下发到 PID ---- */
    if ((kp_m != m->applied_kp_milli) || (ki_m != m->applied_ki_milli) || (kd_m != m->applied_kd_milli))
    {
      PID_SetGains(&m->speed_pid,
                   (float)kp_m * 0.001f, (float)ki_m * 0.001f, (float)kd_m * 0.001f);
      m->applied_kp_milli = kp_m;
      m->applied_ki_milli = ki_m;
      m->applied_kd_milli = kd_m;
    }
    if ((pkp_m != m->applied_p_kp_milli) || (pki_m != m->applied_p_ki_milli) || (pkd_m != m->applied_p_kd_milli))
    {
      PID_SetGains(&m->pos_pid,
                   (float)pkp_m * 0.001f, (float)pki_m * 0.001f, (float)pkd_m * 0.001f);
      m->applied_p_kp_milli = pkp_m;
      m->applied_p_ki_milli = pki_m;
      m->applied_p_kd_milli = pkd_m;
    }

    /* ---- 4. 处理"编码器清零"请求（位置环定零点） ---- */
    if (enc_reset != 0U)
    {
      __HAL_TIM_SET_COUNTER(s_hw[i].enc_tim, 0U);
      m->enc_prev  = 0;
      m->enc_total = 0;
      m->enc_delta = 0;
      m->angle_deg = 0.0f;
    }

    /* ---- 5. 编码器：有符号差值，一次解决正负号和 16 位回绕 ----
       s_invert 置 1 时测速与 motor_apply 的 PWM 翻转保持同侧，
       闭环里"指令 -> 输出 -> 测量"的符号才是一致的（否则正反馈飞转）。 */
    now          = (int16_t)__HAL_TIM_GET_COUNTER(s_hw[i].enc_tim);
    delta        = (int16_t)(now - m->enc_prev);
    m->enc_prev  = now;
    if (s_invert[i] != 0U)
    {
      delta = (int16_t)(-delta);
    }
    m->enc_delta = delta;
    m->enc_total += (int32_t)delta;

    /* ---- 6. 测速 + RC 低通（alpha = dt/(RC+dt)，换采样周期不用重调） ---- */
    raw_speed = (float)delta / dt;
    if (MOTOR_SPEED_FILTER_RC_S <= 0.0f)
    {
      alpha = 1.0f;
    }
    else
    {
      alpha = dt / (MOTOR_SPEED_FILTER_RC_S + dt);
    }
    m->speed_cps += alpha * (raw_speed - m->speed_cps);

    /* ---- 7. 物理量换算 ---- */
    if (MOTOR_COUNTS_PER_REV > 0.0f)
    {
      m->speed_rpm = m->speed_cps * 60.0f / MOTOR_COUNTS_PER_REV;
      m->angle_deg = (float)m->enc_total * 360.0f / MOTOR_COUNTS_PER_REV;
    }

    /* ---- 8. 输出：开环请求优先，其次位置环，其次速度环 ---- */
    if (open_pending != 0U)
    {
      /* 开环指令：直接写 PWM，本周期不跑环 */
      if (m->pos_loop_active != 0U)
      {
        PID_Reset(&m->pos_pid);
      }
      m->pos_loop_active = 0U;
      motor_apply((MotorId_t)i, open_pwm);
    }
    else if ((pos_loop_en != 0U) && (MOTOR_ENCODER_CALIBRATED != 0U))
    {
      float speed_ref;

      if (m->pos_loop_active == 0U)
      {
        PID_Reset(&m->pos_pid);      /* 刚打开位置环：积分从零开始 */
      }

      speed_ref = PID_Calc(&m->pos_pid, m->angle_deg, (float)pos_target_milli * 0.001f, dt);

      /* 位置环输出就是速度给定，再限一次幅（PID 内部已有 MaxOut，这是双保险） */
      if (speed_ref > (float)MOTOR_MAX_SPEED_CPS)   { speed_ref = (float)MOTOR_MAX_SPEED_CPS; }
      if (speed_ref < -(float)MOTOR_MAX_SPEED_CPS)  { speed_ref = -(float)MOTOR_MAX_SPEED_CPS; }

      motor_apply((MotorId_t)i, (int32_t)PID_Calc(&m->speed_pid, m->speed_cps, speed_ref, dt));
      m->pos_loop_active = 1U;
    }
    else if (closed_loop != 0U)
    {
      if (m->pos_loop_active != 0U)
      {
        PID_Reset(&m->pos_pid);      /* 退出位置环，清掉它的积分 */
      }

#if (MOTOR_TARGET_SLEW_CPS_PER_S > 0)
      /* ---- 目标斜坡：ramped_cps 按固定速率逼近指令目标，轮速不跳变 ----
         含过零换向：目标从正变负时同样被斜率限制，不会一帧反转。 */
      {
        int32_t step = (int32_t)(((float)MOTOR_TARGET_SLEW_CPS_PER_S * dt) + 0.5f);

        if (step < 1)
        {
          step = 1;      /* dt 异常时至少走 1 cps，保证一定能到目标 */
        }
        m->ramped_cps = motor_slew_step(m->ramped_cps, speed_target_cps, step);
      }
#else
      m->ramped_cps = speed_target_cps;   /* 斜坡关闭：目标立即生效（旧行为） */
#endif

      motor_apply((MotorId_t)i, (int32_t)PID_Calc(&m->speed_pid, m->speed_cps, (float)m->ramped_cps, dt));
      m->pos_loop_active = 0U;
    }
    else
    {
      m->pos_loop_active = 0U;       /* 既没有开环请求也没有闭环：保持上一次输出 */
    }

#if (MOTOR_CMD_TIMEOUT_MS > 0U)
    /* ---- 9. 指令超时保护 ---- */
    if ((closed_loop != 0U) && ((HAL_GetTick() - m->last_cmd_tick) > MOTOR_CMD_TIMEOUT_MS))
    {
      m->closed_loop = 0U;
      m->pos_loop_en = 0U;
      m->target_cps  = 0;
      m->ramped_cps  = 0;
      PID_Reset(&m->speed_pid);
      PID_Reset(&m->pos_pid);
      motor_apply((MotorId_t)i, 0);
    }
#endif
  }
}

/* ========================== 只读查询 ========================== */

int32_t Motor_GetSpeedMilli(MotorId_t id)
{
  int32_t milli = 0;

  if ((uint32_t)id >= MOTOR_NUM)
  {
    return 0;
  }

  taskENTER_CRITICAL();                 /* float 跨任务读，临界区防撕裂 */
  milli = (int32_t)(s_m[id].speed_cps * 1000.0f);
  taskEXIT_CRITICAL();

  return milli;
}

int32_t Motor_GetSpeedRpmMilli(MotorId_t id)
{
  int32_t milli = 0;

  if ((uint32_t)id >= MOTOR_NUM)
  {
    return 0;
  }
  if (MOTOR_ENCODER_CALIBRATED == 0U)
  {
    return 0;                           /* 未标定：不给物理量，免得被误导 */
  }

  taskENTER_CRITICAL();
  milli = (int32_t)(s_m[id].speed_rpm * 1000.0f);
  taskEXIT_CRITICAL();

  return milli;
}

int32_t Motor_GetAngleMilliDeg(MotorId_t id)
{
  int32_t milli = 0;

  if ((uint32_t)id >= MOTOR_NUM)
  {
    return 0;
  }
  if (MOTOR_ENCODER_CALIBRATED == 0U)
  {
    return 0;
  }

  taskENTER_CRITICAL();
  milli = (int32_t)(s_m[id].angle_deg * 1000.0f);
  taskEXIT_CRITICAL();

  return milli;
}

int16_t Motor_GetDelta(MotorId_t id)
{
  return ((uint32_t)id < MOTOR_NUM) ? s_m[id].enc_delta : 0;
}

int32_t Motor_GetTotal(MotorId_t id)
{
  return ((uint32_t)id < MOTOR_NUM) ? s_m[id].enc_total : 0;
}

int32_t Motor_GetPwm(MotorId_t id)
{
  return ((uint32_t)id < MOTOR_NUM) ? s_m[id].out_pwm : 0;
}

uint32_t Motor_GetMode(MotorId_t id)
{
  uint32_t mode;

  if ((uint32_t)id >= MOTOR_NUM)
  {
    return 0U;
  }

  taskENTER_CRITICAL();               /* 两个标志要读成一致的组合 */
  if ((s_m[id].pos_loop_en != 0U) && (MOTOR_ENCODER_CALIBRATED != 0U))
  {
    mode = 2U;
  }
  else if (s_m[id].closed_loop != 0U)
  {
    mode = 1U;
  }
  else
  {
    mode = 0U;
  }
  taskEXIT_CRITICAL();

  return mode;
}

uint8_t Motor_IsCalibrated(void)
{
  return (MOTOR_ENCODER_CALIBRATED != 0U) ? 1U : 0U;
}

float Motor_GetCountsPerRev(void)
{
  return MOTOR_COUNTS_PER_REV;
}

/* ========================== FreeRTOS 任务体 ========================== */

/* 强符号定义：会覆盖 freertos.c 里的 __weak StartMotorTask() */
void StartMotorTask(void const *argument)
{
  uint32_t   last_count = 0U;
  TickType_t wake;

  (void)argument;

  /* 注意：Motor_Init() 在 main.c 的 USER CODE BEGIN 2 里调用（调度器启动前） */

  ControlTime_Mark(&last_count);
  wake = xTaskGetTickCount();

  for (;;)
  {
    float dt = ControlTime_DeltaSeconds(&last_count);

    Motor_UpdateAll(dt);

    vTaskDelayUntil(&wake, pdMS_TO_TICKS(MOTOR_PERIOD_MS));
  }
}
