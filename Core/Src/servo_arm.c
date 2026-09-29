/**
  ******************************************************************************
  * @file    servo_arm.c
  * @brief   4 路舵机控制实现（用户代码）
  *
  * 为什么用整型共享状态：
  *   CommTask 会设定角度，ArmTask 会读角度并做斜坡。若共享 float，
  *   在两个任务之间可能出现"半个 float"的撕裂读。这里统一存 us 整型，
  *   Cortex-M3 上对齐的 32 位读写是单指令，天然安全，不需要加锁。
  *
  * 硬件操作全部集中在 ArmTask：
  *   CommTask 只改 target_us / snap_us / enabled / slew 这些 32 位整型字段，
  *   PWM 的 Start/Stop 与 CCR 写入都由 Servo_Update() 完成，避免跨任务操作外设。
  ******************************************************************************
  */
#include "servo_arm.h"
#include "control_time.h"

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

#if (SERVO_HW_ENABLED != 0)
#include "tim.h"
/* CubeMX 配置了 TIM8 之后 tim.c 里会有这个定义。这里再 extern 一次是为了
   不依赖 tim.h 是否已经生成 htim8 的声明（重复 extern 声明是合法的）。 */
extern TIM_HandleTypeDef htim8;
#endif

/* 舵机编号（串口命令里的 <1-4>）-> TIM8 通道。
   F103 的 TIM8 没有重映射，通道和引脚是固定对应的：
     TIM8_CH1 = PC6, CH2 = PC7, CH3 = PC8, CH4 = PC9
   对照 main.h 里 CubeMX 生成的标签：
     Arm1PWM = PC9 = TIM8_CH4
     Arm2PWM = PC8 = TIM8_CH3
     Arm3PWM = PC7 = TIM8_CH2
     Arm4PWM = PC6 = TIM8_CH1
   所以下面按"串口号 = Arm 编号"来排，这样 S 1 控制的就是你接在 Arm1PWM 上的舵机。
   如果你更希望 "S 1" 对应 PC6，把这里改成 { CH1, CH2, CH3, CH4 } 即可。 */
static const uint32_t s_ch[SERVO_NUM] =
{
  TIM_CHANNEL_4,   /* 舵机 1 -> Arm1PWM (PC9) */
  TIM_CHANNEL_3,   /* 舵机 2 -> Arm2PWM (PC8) */
  TIM_CHANNEL_2,   /* 舵机 3 -> Arm3PWM (PC7) */
  TIM_CHANNEL_1    /* 舵机 4 -> Arm4PWM (PC6) */
};

typedef struct
{
  /* 跨任务共享（只写 32 位整型，天然原子） */
  volatile uint32_t target_us;      /* 斜坡目标脉宽 */
  volatile int32_t  snap_us;        /* >=0 表示下一周期直接跳到该脉宽 */
  volatile uint32_t slew_us_per_s;  /* 斜坡速度 */
  volatile uint32_t enabled;        /* 输出使能 */

  /* 仅 ArmTask 使用 */
  uint32_t cur_us;
} Servo_t;

static Servo_t s_sv[SERVO_NUM];
static uint8_t s_hw_on[SERVO_NUM];   /* 仅 ArmTask 使用：PWM 通道是否已启动 */

/* ========================== 内部函数 ========================== */

static uint32_t servo_clamp_us(uint32_t us)
{
  if (us < SERVO_MIN_US) { us = SERVO_MIN_US; }
  if (us > SERVO_MAX_US) { us = SERVO_MAX_US; }
  return us;
}

static uint32_t servo_deg_to_us(float deg)
{
  float us;

  if (deg < 0.0f)            { deg = 0.0f; }
  if (deg > SERVO_ANGLE_MAX) { deg = SERVO_ANGLE_MAX; }

  us = (float)SERVO_MIN_US +
       ((float)(SERVO_MAX_US - SERVO_MIN_US) * deg / SERVO_ANGLE_MAX);

  return servo_clamp_us((uint32_t)(us + 0.5f));
}

static int32_t servo_us_to_deg_milli(uint32_t us)
{
  uint32_t span  = (SERVO_MAX_US - SERVO_MIN_US);
  uint32_t ratio;   /* 0..1000 */

  if (us < SERVO_MIN_US) { us = SERVO_MIN_US; }
  if (us > SERVO_MAX_US) { us = SERVO_MAX_US; }

  ratio = ((us - SERVO_MIN_US) * 1000U) / span;

  return (int32_t)(((uint32_t)((float)ratio * SERVO_ANGLE_MAX / 1000.0f) + 0.5f));
}

static void servo_write(uint8_t idx, uint32_t us)
{
#if (SERVO_HW_ENABLED != 0)
  __HAL_TIM_SET_COMPARE(&htim8, s_ch[idx], us);
#else
  (void)idx;
  (void)us;
#endif
}

static void servo_hw_set(uint8_t idx, uint8_t on)
{
#if (SERVO_HW_ENABLED != 0)
  if ((on != 0U) && (s_hw_on[idx] == 0U))
  {
    (void)HAL_TIM_PWM_Start(&htim8, s_ch[idx]);
    s_hw_on[idx] = 1U;
  }
  else if ((on == 0U) && (s_hw_on[idx] != 0U))
  {
    (void)HAL_TIM_PWM_Stop(&htim8, s_ch[idx]);
    s_hw_on[idx] = 0U;
  }
#else
  (void)idx;
  (void)on;
#endif
}

/* ========================== 对外接口 ========================== */

void Servo_Init(void)
{
  uint32_t i;

  for (i = 0U; i < SERVO_NUM; i++)
  {
    s_sv[i].cur_us        = SERVO_MID_US;
    s_sv[i].target_us     = SERVO_MID_US;
    s_sv[i].snap_us       = -1;
    s_sv[i].slew_us_per_s = SERVO_DEFAULT_SLEW;
    s_sv[i].enabled       = 1U;
    s_hw_on[i]            = 0U;

    /* MX 只做了 HAL_TIM_PWM_Init，必须自己 Start 才有波形 */
    servo_hw_set((uint8_t)i, 1U);
    servo_write((uint8_t)i, SERVO_MID_US);
  }
}

void Servo_SetSlew(uint8_t idx, uint32_t us_per_sec)
{
  if (idx < SERVO_NUM)
  {
    s_sv[idx].slew_us_per_s = us_per_sec;
  }
}

void Servo_SetPulseUs(uint8_t idx, uint32_t us)
{
  if (idx >= SERVO_NUM)
  {
    return;
  }

  us = servo_clamp_us(us);
  s_sv[idx].target_us = us;
  s_sv[idx].snap_us   = (int32_t)us;   /* 下一周期直接到位 */
  s_sv[idx].enabled   = 1U;
}

void Servo_SetAngle(uint8_t idx, float deg)
{
  Servo_SetPulseUs(idx, servo_deg_to_us(deg));
}

void Servo_SetTargetPulseUs(uint8_t idx, uint32_t us)
{
  if (idx >= SERVO_NUM)
  {
    return;
  }

  s_sv[idx].target_us = servo_clamp_us(us);
  s_sv[idx].snap_us   = -1;            /* 走斜坡，取消未生效的跳变 */
  s_sv[idx].enabled   = 1U;
}

void Servo_SetTargetAngle(uint8_t idx, float deg)
{
  Servo_SetTargetPulseUs(idx, servo_deg_to_us(deg));
}

void Servo_Update(float dt)
{
  uint32_t i;

  if (dt <= 0.0f || dt > 1.0f)
  {
    dt = (float)SERVO_PERIOD_MS / 1000.0f;
  }

  for (i = 0U; i < SERVO_NUM; i++)
  {
    Servo_t *s = &s_sv[i];
    int32_t  snap;
    uint32_t target;
    uint32_t step;

    /* --- 输出使能（Relax/Enable 的实际动作在这里发生） --- */
    servo_hw_set((uint8_t)i, (s->enabled != 0U) ? 1U : 0U);
    if (s->enabled == 0U)
    {
      continue;
    }

    snap   = s->snap_us;
    target = s->target_us;

    if (snap >= 0)
    {
      /* 立即到位 */
      s->snap_us = -1;
      s->cur_us  = servo_clamp_us((uint32_t)snap);
    }
    else if (s->slew_us_per_s == 0U)
    {
      s->cur_us = target;
    }
    else
    {
      step = (uint32_t)((float)s->slew_us_per_s * dt);
      if (step == 0U)
      {
        step = 1U;
      }

      if (s->cur_us < target)
      {
        s->cur_us += step;
        if (s->cur_us > target) { s->cur_us = target; }
      }
      else if (s->cur_us > target)
      {
        if (s->cur_us > (step + target)) { s->cur_us -= step; }
        else                             { s->cur_us = target; }
      }
      else
      {
        /* 已到位 */
      }
    }

    servo_write((uint8_t)i, s->cur_us);
  }
}

void Servo_Relax(void)
{
  uint32_t i;

  for (i = 0U; i < SERVO_NUM; i++)
  {
    s_sv[i].enabled = 0U;    /* 实际停止由 ArmTask 在 20ms 内完成 */
  }
}

void Servo_Enable(void)
{
  uint32_t i;

  for (i = 0U; i < SERVO_NUM; i++)
  {
    s_sv[i].enabled = 1U;
  }
}

int32_t Servo_GetAngleMilli(uint8_t idx)
{
  return (idx < SERVO_NUM) ? servo_us_to_deg_milli(s_sv[idx].cur_us) : 0;
}

uint32_t Servo_GetPulseUs(uint8_t idx)
{
  return (idx < SERVO_NUM) ? s_sv[idx].cur_us : 0U;
}

int32_t Servo_GetTargetAngleMilli(uint8_t idx)
{
  return (idx < SERVO_NUM) ? servo_us_to_deg_milli(s_sv[idx].target_us) : 0;
}

/* ========================== FreeRTOS 任务体 ========================== */

/* 强符号定义：会覆盖 freertos.c 里的 __weak StartArmTask() */
void StartArmTask(void const *argument)
{
  uint32_t   last_count = 0U;
  TickType_t wake;

  (void)argument;

  /* 注意：Servo_Init() 在 main.c 的 USER CODE BEGIN 2 里调用（调度器启动前） */

  ControlTime_Mark(&last_count);
  wake = xTaskGetTickCount();

  for (;;)
  {
    float dt = ControlTime_DeltaSeconds(&last_count);

    Servo_Update(dt);

    /* 机械臂逆解 / 动作序列以后加在这里，注意不要阻塞 20ms 的周期 */

    vTaskDelayUntil(&wake, pdMS_TO_TICKS(SERVO_PERIOD_MS));
  }
}
