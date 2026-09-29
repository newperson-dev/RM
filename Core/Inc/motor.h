/**
  ******************************************************************************
  * @file    motor.h
  * @brief   4 路直流电机驱动 + 编码器测速 + 增量式 PI(D) 闭环
  * @note    用户代码，不由 CubeMX 管理。
  *
  * 硬件映射（引脚/定时器由 CubeMX 生成，本文件只引用 main.h 里的宏）：
  *   PWM   : TIM1 CH1..CH4  -> Motor1..Motor4
  *   方向  : MotorXGPIOA / MotorXGPIOB（H 桥 IN1/IN2）
  *   编码器: TIM2/TIM3/TIM4/TIM5（编码器模式 TI12，ARR=65535）
  *
  * 线程约定：
  *   - Motor_UpdateAll() 只允许 MotorTask 调用（1kHz）；
  *   - Motor_SetPwm() / Motor_SetTargetSpeed() / Motor_SetPid*() 只允许任务上下文调用，
  *     不要放进中断里（内部没有用临界区，靠"只共享 32 位整型"来保证原子性）；
  *   - 跨任务读取速度请用 Motor_GetSpeedMilli()（内部有临界区保护）。
  ******************************************************************************
  */
#ifndef __MOTOR_H
#define __MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "main.h"

#define MOTOR_NUM               4U

/* ============================ 可调参数 ============================ */

/** PWM 满量程默认值：必须与 TIM1 的 ARR 一致。
 *  实际运行时 Motor_Init() 会直接读 htim1.Init.Period，所以你在 MX 里改 ARR
 *  不需要动这里（这里只是 Motor_Init 之前的兜底值）。 */
#define MOTOR_PWM_MAX_DEFAULT   3599

/** 控制周期（ms），必须与 MotorTask 的 vTaskDelayUntil 一致 */
#define MOTOR_PERIOD_MS         1U

/** 速度低通系数：越小越平滑、相位滞后越大；1.0f = 不平滑 */
#define MOTOR_SPEED_LPF_ALPHA   0.35f

/** 安全保护：>0 时，超过该毫秒数没收到新指令就自动停机；0 = 关闭 */
#define MOTOR_CMD_TIMEOUT_MS    0U

/** 闭环默认增益（千分之一单位，避免在任务间共享 float） */
#define MOTOR_DEFAULT_KP_MILLI  0
#define MOTOR_DEFAULT_KI_MILLI  0
#define MOTOR_DEFAULT_KD_MILLI  0

/* ================================================================ */

typedef enum
{
  MOTOR_1 = 0,
  MOTOR_2 = 1,
  MOTOR_3 = 2,
  MOTOR_4 = 3
} MotorId_t;

/** @brief 启动 TIM1 四路 PWM 与 TIM2..5 编码器，清零方向与占空比。 */
void Motor_Init(void);

/** @brief 开环设定：pwm 范围 -满量程..+满量程（正=正转）。
 *  调用后该电机退出闭环（避免闭环把开环指令覆盖掉）。 */
void Motor_SetPwm(MotorId_t id, int32_t pwm);

/** @brief 闭环设定目标速度，单位 = 编码器计数/秒，正负代表方向。
 *  会打开该电机的闭环开关；增益为 0 时输出恒为 0（安全默认）。 */
void Motor_SetTargetSpeed(MotorId_t id, int32_t counts_per_sec);

/** @brief 设置闭环增益（float 版本，内部转成千分之一整型存储）。 */
void Motor_SetPid(MotorId_t id, float kp, float ki, float kd);

/** @brief 设置闭环增益（千分之一整型版本，例如 1200 表示 1.2）。 */
void Motor_SetPidMilli(MotorId_t id, int32_t kp_milli, int32_t ki_milli, int32_t kd_milli);

/** @brief 闭环开关：0 = 开环（保留当前 PWM），1 = 闭环。 */
void Motor_EnableClosedLoop(MotorId_t id, uint32_t enable);

/** @brief 全部电机：清目标、退闭环、PWM 归零（滑行）。 */
void Motor_StopAll(void);

/** @brief 控制周期调用（MotorTask，1kHz）：读编码器 -> 测速 -> PI(D) -> 写 PWM。
 *  @param dt 距离上次调用的实际秒数，来自 ControlTime_DeltaSeconds()。 */
void Motor_UpdateAll(float dt);

/* ------------------------------ 只读查询 ------------------------------ */

/** @brief 速度（计数/秒），带临界区保护，可跨任务调用。 */
int32_t Motor_GetSpeedMilli(MotorId_t id);

/** @brief 本周期编码器增量（带符号）。 */
int16_t Motor_GetDelta(MotorId_t id);

/** @brief 累计计数（带符号，32 位，直接读，天然原子）。 */
int32_t Motor_GetTotal(MotorId_t id);

/** @brief 当前实际 PWM 输出（带符号）。 */
int32_t Motor_GetPwm(MotorId_t id);

/** @brief FreeRTOS 任务体：定义强符号以覆盖 freertos.c 里的 __weak 空实现。 */
void StartMotorTask(void const *argument);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_H */
