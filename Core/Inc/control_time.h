/**
  ******************************************************************************
  * @file    control_time.h
  * @brief   基于 DWT 周期计数器的高精度时间戳（Cortex-M3 @72MHz -> 13.9ns）
  * @note    用户代码，不由 CubeMX 管理。
  *
  * 使用约定（必须遵守，否则第一次读数会是垃圾值）：
  *   1) 启动时调用一次 ControlTime_Init(SystemCoreClock / HAL_RCC_GetHCLKFreq())；
  *   2) 每个需要测 dt 的循环，进入循环前调用一次 ControlTime_Mark(&last)；
  *   3) 循环内用 ControlTime_DeltaSeconds(&last) 取"距上次调用"的秒数。
  *
  * 注意：
  *   - CYCCNT 是 32 位自由计数器，72MHz 下 59.65s 回绕一次。
  *     无符号减法只对"跨一次回绕"正确，因此调用间隔必须远小于 59.65s；
  *   - 调试器 halt 时 CYCCNT 停止计数（正常现象，不是 bug）；
  *   - M3 没有 FPU，float 运算是软件模拟，别把 DeltaSeconds 放进高频内层循环，
  *     需要高频时用 ControlTime_DeltaCycles()。
  ******************************************************************************
  */
#ifndef __CONTROL_TIME_H
#define __CONTROL_TIME_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/** @brief 初始化 DWT 周期计数器。cpu_hz 传 0 时自动使用 SystemCoreClock。 */
void ControlTime_Init(uint32_t cpu_hz);

/** @brief 读取当前周期计数值（原始值，用于打时间戳）。 */
uint32_t ControlTime_Now(void);

/** @brief 记录一次起点，必须在循环开始前调用，否则首次 delta 无意义。 */
void ControlTime_Mark(uint32_t *last_count);

/** @brief 返回距上次调用经过的 CPU 周期数（自动处理一次回绕）。 */
uint32_t ControlTime_DeltaCycles(uint32_t *last_count);

/** @brief 返回距上次调用经过的秒数（float，软浮点）。 */
float ControlTime_DeltaSeconds(uint32_t *last_count);

#ifdef __cplusplus
}
#endif

#endif /* __CONTROL_TIME_H */
