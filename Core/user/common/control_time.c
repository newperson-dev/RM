/**
  ******************************************************************************
  * @file    control_time.c
  * @brief   DWT 周期计数器时间戳实现（用户代码，不由 CubeMX 管理）
  ******************************************************************************
  */
#include "control_time.h"
#include "main.h"

static uint32_t dwt_cpu_hz = 0U;

void ControlTime_Init(uint32_t cpu_hz)
{
  if (cpu_hz == 0U)
  {
    cpu_hz = SystemCoreClock;
  }
  if (cpu_hz == 0U)
  {
    return;
  }

  /* 1. 使能跟踪与调试模块 */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  /* 2. 清零并在读取前先写一次，避免脏值 */
  DWT->CYCCNT = 0U;
  /* 3. 启动周期计数器 */
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  dwt_cpu_hz = cpu_hz;
}

uint32_t ControlTime_Now(void)
{
  return DWT->CYCCNT;
}

void ControlTime_Mark(uint32_t *last_count)
{
  if (last_count != NULL)
  {
    *last_count = DWT->CYCCNT;
  }
}

uint32_t ControlTime_DeltaCycles(uint32_t *last_count)
{
  uint32_t now_count;
  uint32_t delta_count;

  if ((last_count == NULL) || (dwt_cpu_hz == 0U))
  {
    return 0U;
  }

  now_count   = DWT->CYCCNT;
  delta_count = now_count - *last_count;   /* 无符号减法：跨一次回绕仍然正确 */
  *last_count = now_count;

  return delta_count;
}

float ControlTime_DeltaSeconds(uint32_t *last_count)
{
  return (float)ControlTime_DeltaCycles(last_count) / (float)dwt_cpu_hz;
}
