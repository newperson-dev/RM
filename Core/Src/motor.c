/**
  ******************************************************************************
  * @file    motor.c
  * @brief   4 路电机：PWM + 方向 + 编码器测速 + PI(D) 闭环（用户代码）
  *
  * 设计要点：
  *   1) 跨任务共享的数据全部是 32 位整型（目标速度、增益、PWM），
  *      Cortex-M3 上对齐的 32 位读写是单条指令，天然原子，不需要加锁；
  *   2) 编码器用 int16_t 差值读取，正反转都正确（ARR=65535 时跨 0 也正确）；
  *   3) 反向前先关 PWM，再做方向切换，避免 H 桥直通/大电流冲击；
  *   4) 默认增益为 0 -> 闭环输出恒为 0，未调参前不会乱转。
  ******************************************************************************
  */
#include "motor.h"
#include "tim.h"
#include "control_time.h"

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

/** 某个电机转向装反了就把对应位改成 1 */
static const uint8_t s_invert[MOTOR_NUM] = { 0U, 0U, 0U, 0U };

/* ========================== 内部状态 ========================== */

typedef struct
{
  /* 跨任务共享（只写 32 位整型） */
  volatile int32_t  target_cps;      /* 目标速度：编码器计数/秒 */
  volatile int32_t  kp_milli;
  volatile int32_t  ki_milli;
  volatile int32_t  kd_milli;
  volatile uint32_t closed_loop;
  volatile uint32_t last_cmd_tick;

  /* 仅 MotorTask 使用 */
  int16_t  enc_prev;
  int32_t  enc_total;
  int16_t  enc_delta;
  float    speed;                    /* 滤波后速度，计数/秒 */
  float    integ;                    /* 积分项（已折算成 PWM 量纲前） */
  float    prev_meas;
  int32_t  out_pwm;
} Motor_t;

static Motor_t s_m[MOTOR_NUM];
static uint32_t s_pwm_max = MOTOR_PWM_MAX_DEFAULT;

/* ========================== 内部函数 ========================== */

/** 真正写硬件：方向 + 占空比。所有调用者都在任务上下文。 */
static void motor_apply(MotorId_t id, int32_t pwm)
{
  const MotorHw_t *hw = &s_hw[id];
  Motor_t         *m  = &s_m[id];
  uint32_t         duty;

  if (pwm > (int32_t)s_pwm_max)  { pwm = (int32_t)s_pwm_max; }
  if (pwm < -(int32_t)s_pwm_max) { pwm = -(int32_t)s_pwm_max; }
  if (s_invert[id] != 0U)        { pwm = -pwm; }

  duty = (uint32_t)((pwm < 0) ? -pwm : pwm);

  /* 先关占空比再换向 */
  __HAL_TIM_SET_COMPARE(hw->pwm_tim, hw->pwm_ch, 0U);

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

  __HAL_TIM_SET_COMPARE(hw->pwm_tim, hw->pwm_ch, duty);
  m->out_pwm = (s_invert[id] != 0U) ? -pwm : pwm;
}

/* ========================== 对外接口 ========================== */

void Motor_Init(void)
{
  uint32_t i;

  /* PWM 满量程直接取 TIM1 的 ARR，MX 里改频率不用改代码 */
  if (htim1.Init.Period != 0U)
  {
    s_pwm_max = htim1.Init.Period;
  }

  for (i = 0U; i < MOTOR_NUM; i++)
  {
    Motor_t *m = &s_m[i];

    m->target_cps   = 0;
    m->kp_milli     = MOTOR_DEFAULT_KP_MILLI;
    m->ki_milli     = MOTOR_DEFAULT_KI_MILLI;
    m->kd_milli     = MOTOR_DEFAULT_KD_MILLI;
    m->closed_loop  = 0U;
    m->last_cmd_tick = HAL_GetTick();
    m->enc_prev     = 0;
    m->enc_total    = 0;
    m->enc_delta    = 0;
    m->speed        = 0.0f;
    m->integ        = 0.0f;
    m->prev_meas    = 0.0f;
    m->out_pwm      = 0;

    /* 启动 PWM 输出（MX 只做了 Init，没有 Start） */
    (void)HAL_TIM_PWM_Start(s_hw[i].pwm_tim, s_hw[i].pwm_ch);

    /* 启动编码器接口 */
    (void)HAL_TIM_Encoder_Start(s_hw[i].enc_tim, TIM_CHANNEL_ALL);
    __HAL_TIM_SET_COUNTER(s_hw[i].enc_tim, 0U);

    motor_apply((MotorId_t)i, 0);
  }
}

void Motor_SetPwm(MotorId_t id, int32_t pwm)
{
  if ((uint32_t)id >= MOTOR_NUM)
  {
    return;
  }

  s_m[id].closed_loop   = 0U;          /* 开环指令优先，退出闭环 */
  s_m[id].target_cps    = 0;
  s_m[id].integ         = 0.0f;
  s_m[id].last_cmd_tick = HAL_GetTick();

  motor_apply(id, pwm);
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

void Motor_SetPidMilli(MotorId_t id, int32_t kp_milli, int32_t ki_milli, int32_t kd_milli)
{
  if ((uint32_t)id >= MOTOR_NUM)
  {
    return;
  }

  s_m[id].kp_milli = kp_milli;
  s_m[id].ki_milli = (ki_milli < 0) ? 0 : ki_milli;   /* 积分项不允许负增益 */
  s_m[id].kd_milli = kd_milli;
  s_m[id].integ    = 0.0f;
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
    s_m[id].integ      = 0.0f;
  }
}

void Motor_StopAll(void)
{
  uint32_t i;

  for (i = 0U; i < MOTOR_NUM; i++)
  {
    s_m[i].target_cps  = 0;
    s_m[i].closed_loop = 0U;
    s_m[i].integ       = 0.0f;
    motor_apply((MotorId_t)i, 0);
  }
}

void Motor_UpdateAll(float dt)
{
  uint32_t i;

  if (dt <= 0.0f || dt > 0.2f)
  {
    dt = (float)MOTOR_PERIOD_MS / 1000.0f;   /* dt 异常时退回到标称周期 */
  }

  for (i = 0U; i < MOTOR_NUM; i++)
  {
    Motor_t *m  = &s_m[i];
    int16_t  now;
    int16_t  delta;
    float    raw_speed;

    /* ---- 1. 读编码器（有符号差值，正反转都正确） ---- */
    now        = (int16_t)__HAL_TIM_GET_COUNTER(s_hw[i].enc_tim);
    delta      = (int16_t)(now - m->enc_prev);
    m->enc_prev = now;
    m->enc_delta = delta;
    m->enc_total += (int32_t)delta;

    /* ---- 2. 测速 + 一阶低通（MOTOR_SPEED_LPF_ALPHA = 1.0f 表示不平滑） ---- */
    raw_speed = (float)delta / dt;
    m->speed += MOTOR_SPEED_LPF_ALPHA * (raw_speed - m->speed);

    /* ---- 3. PI(D) 闭环 ---- */
    if (m->closed_loop != 0U)
    {
      float kp = (float)m->kp_milli * 0.001f;
      float ki = (float)m->ki_milli * 0.001f;
      float kd = (float)m->kd_milli * 0.001f;
      float target = (float)m->target_cps;
      float err    = target - m->speed;
      float out;
      float i_limit;
      int32_t pwm;

      /* 积分限幅：限制积分项单独贡献不超过一半量程 */
      if (ki > 0.000001f)
      {
        i_limit = ((float)s_pwm_max * 0.5f) / ki;
        m->integ += err * dt;
        if (m->integ > i_limit)       { m->integ = i_limit; }
        else if (m->integ < -i_limit) { m->integ = -i_limit; }
      }
      else
      {
        m->integ = 0.0f;
      }

      out = (kp * err) + (ki * m->integ) - (kd * (m->speed - m->prev_meas) / dt);
      m->prev_meas = m->speed;

      pwm = (int32_t)out;
      motor_apply((MotorId_t)i, pwm);
    }

#if (MOTOR_CMD_TIMEOUT_MS > 0U)
    /* ---- 4. 指令超时保护 ---- */
    if ((m->closed_loop != 0U) &&
        ((HAL_GetTick() - m->last_cmd_tick) > MOTOR_CMD_TIMEOUT_MS))
    {
      m->closed_loop = 0U;
      m->target_cps  = 0;
      m->integ       = 0.0f;
      motor_apply((MotorId_t)i, 0);
    }
#endif
  }
}

int32_t Motor_GetSpeedMilli(MotorId_t id)
{
  int32_t milli = 0;

  if ((uint32_t)id >= MOTOR_NUM)
  {
    return 0;
  }

  taskENTER_CRITICAL();
  milli = (int32_t)(s_m[id].speed * 1000.0f);
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
