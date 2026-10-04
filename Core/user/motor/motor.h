/**
  ******************************************************************************
  * @file    motor.h
  * @brief   4 路直流电机：PWM + 方向 + 编码器测速 + 速度环/位置环（串级）
  * @note    目录按任务职责划分：Core/user/motor/
  *
  * 本次吸收的 4 项（对照旧工程 ex/user/motor/）：
  *   1) 速度滤波改成 RC 时间常数形式：alpha = dt / (RC + dt)，
  *      好处是以后改控制周期不用重新调滤波系数（旧工程 encode.c 的做法）；
  *   2) 速度环改用通用 PID 库（common/controller.h），
  *      自动获得抗积分饱和、梯形积分、变积分速率、微分低通等能力；
  *   3) 增加编码器物理量换算：计数 <-> 圈 <-> 角度/RPM（旧工程 encode.h 的做法）；
  *   4) 增加位置环，形成 位置 -> 速度 串级双闭环（旧工程 motor.c 的结构）。
  *
  * 单位约定：
  *   - 速度环的测量/给定单位 = **编码器计数/秒**（cps），PWM 输出单位 = TIM1 的 ARR 计数；
  *   - 位置环的测量/给定单位 = **度**，它的输出单位 = 计数/秒（也就是速度环的给定）。
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

/* ==================== 机械参数（已按 520 电机标定） ====================
 * 计数/圈 = 编码器线数 × 四倍频 × 减速比
 *   520 电机：13 线编码器，减速比 1:30，编码器模式 TI1&T2（四倍频）
 *   -> 13 × 4 × 30 = 1560 计数/圈（输出轴转一圈）
 *
 * 由这个数可以推出速度量程：
 *   空载 330 rpm -> 330/60 × 1560 = 8580 计数/秒   （这就是速度上限，PWM 满量程对应它）
 *   额定 320 rpm -> 320/60 × 1560 = 8320 计数/秒
 */
#define MOTOR_ENCODER_PPR         13.0f    /* 编码器线数 */
#define MOTOR_ENCODER_QUADRATURE  4.0f     /* 四倍频（Encoder Mode = TI1 and TI2 即四倍频） */
#define MOTOR_GEAR_RATIO          30.0f    /* 减速比 1:30 */
#define MOTOR_COUNTS_PER_REV      (MOTOR_ENCODER_PPR * MOTOR_ENCODER_QUADRATURE * MOTOR_GEAR_RATIO)
/*                                                        = 1560.0f 计数/圈 */

/** 机械参数是否已标定：
 *    0 = 未标定。此时为了安全，**拒绝**位置环(POS/POSEN)和 RPM 指令(SPDR)，
 *        遥测也只打印计数/秒，不打印 RPM/角度；
 *    1 = 已标定，物理单位可用。
 *  本工程已按 520 电机（13 线 / 1:30 / 1560 计数每圈）标定，置 1。 */
#define MOTOR_ENCODER_CALIBRATED  1

/* ==================== 控制参数 ==================== */

/** PWM 满量程兜底值：运行时 Motor_Init() 会直接读 htim1.Init.Period（ARR），
 *  所以在 MX 里改 PWM 频率不需要改这里。 */
#define MOTOR_PWM_MAX_DEFAULT     3599

/** 控制周期（ms），必须与 MotorTask 的 vTaskDelayUntil 一致 */
#define MOTOR_PERIOD_MS           1U

/** 速度低通时间常数（秒）。
 *  alpha = dt/(RC+dt)：1kHz 下 RC=2ms 约等于上一版固定系数 0.33，
 *  以后把周期改成 2kHz/500Hz，滤波强度自动保持不变。0 = 不滤波。 */
#define MOTOR_SPEED_FILTER_RC_S   0.002f

/** 变积分速率（只在误差很大时抑制积分，专治"启动/急停时的超调"）。
 *  误差单位 = 编码器计数/秒（满量程 8580，见上面的速度量程推算）：
 *    |err| <= CoefB(=800)          -> 正常积分
 *    800 < |err| <= 2500           -> 线性衰减积分
 *    |err| >  2500                 -> 本次完全不积分
 *  设成 0/0 就等于关闭这个功能（退回普通积分）。 */
#define MOTOR_SPEED_VARINT_COEF_A 1700
#define MOTOR_SPEED_VARINT_COEF_B 800

/** 微分项一阶低通时间常数（秒），0 = 不滤波。
 *  测速噪声大的时候，微分项是噪声放大器，这里给 2ms 的轻滤波。 */
#define MOTOR_DERIVATIVE_LPF_RC_S 0.002f

/** 位置环输出限幅（= 送给速度环的最大速度，单位 计数/秒）。
 *  空载 330rpm ≈ 8580 计数/秒，这里取略低一点的 8500 作为速度上限。 */
#define MOTOR_MAX_SPEED_CPS       8500

/** 安全保护：>0 时，超过该毫秒数没收到新指令就自动停机；0 = 关闭 */
#define MOTOR_CMD_TIMEOUT_MS      0U

/** 默认增益（千分之一单位）。全 0 = 不调参绝不动，安全优先 */
#define MOTOR_DEFAULT_KP_MILLI    0
#define MOTOR_DEFAULT_KI_MILLI    0
#define MOTOR_DEFAULT_KD_MILLI    0

/* ================================================================ */

typedef enum
{
  MOTOR_1 = 0,
  MOTOR_2 = 1,
  MOTOR_3 = 2,
  MOTOR_4 = 3
} MotorId_t;

/** @brief 启动 TIM1 四路 PWM 与 TIM2..5 编码器，初始化两个 PID，清零输出。 */
void Motor_Init(void);

/** @brief 全部电机：清目标、退双环、PWM 归零（滑行）。 */
void Motor_StopAll(void);

/** @brief 把某个电机的累计计数清零（位置环使用前必须做，用来确定零点）。 */
void Motor_ResetCount(MotorId_t id);
void Motor_ResetAllCounts(void);

/* ------------------------------ 开环 ------------------------------ */

/** @brief 开环设定：pwm 范围 -满量程..+满量程（正=正转）。会关闭该电机的两个环。 */
void Motor_SetPwm(MotorId_t id, int32_t pwm);

/* ------------------------------ 速度环 ------------------------------ */

/** @brief 目标速度（编码器计数/秒），自动打开速度环。 */
void Motor_SetTargetSpeed(MotorId_t id, int32_t counts_per_sec);

/** @brief 目标速度（RPM）。需要已标定，否则返回 0 且不生效。 */
uint8_t Motor_SetTargetSpeedRpm(MotorId_t id, float rpm);

/** @brief 速度环增益（float 版本，内部转成千分之一整型存储）。 */
void Motor_SetPid(MotorId_t id, float kp, float ki, float kd);
void Motor_SetPidMilli(MotorId_t id, int32_t kp_milli, int32_t ki_milli, int32_t kd_milli);

/** @brief 速度环开关：0 = 停用速度环，1 = 启用。 */
void Motor_EnableClosedLoop(MotorId_t id, uint32_t enable);

/* ------------------------------ 位置环（外环） ------------------------------ */

/** @brief 设定目标角度（度），并自动打开位置环。
 *  @retval 1 = 已接受，0 = 被拒绝（机械参数未标定 或 id 非法）。 */
uint8_t Motor_SetTargetAngle(MotorId_t id, float deg);

/** @brief 位置环开关：打开后速度环的给定由位置环产生。
 *  @note  打开时内部会重置位置环积分，避免上一次的积分突然顶出去。 */
void Motor_EnablePositionLoop(MotorId_t id, uint32_t enable);

/** @brief 位置环增益（千分之一单位）。位置环输出单位 = 计数/秒。 */
void Motor_SetPosPidMilli(MotorId_t id, int32_t kp_milli, int32_t ki_milli, int32_t kd_milli);
void Motor_SetPosPid(MotorId_t id, float kp, float ki, float kd);

/* ------------------------------ 周期任务 ------------------------------ */

/** @brief 控制周期调用（MotorTask，1kHz）：读编码器 -> 测速 -> 双环 -> 写 PWM。
 *  @param dt 距离上次调用的实际秒数（来自 ControlTime_DeltaSeconds）。 */
void Motor_UpdateAll(float dt);

/* ------------------------------ 只读查询 ------------------------------ */

/** @brief 速度，编码器计数/秒 ×1000（带临界区保护，可跨任务调用）。 */
int32_t Motor_GetSpeedMilli(MotorId_t id);

/** @brief 速度 RPM ×1000；未标定时返回 0。 */
int32_t Motor_GetSpeedRpmMilli(MotorId_t id);

/** @brief 角度 ×1000（度，累计多圈）；未标定时返回 0。 */
int32_t Motor_GetAngleMilliDeg(MotorId_t id);

/** @brief 本周期编码器增量（带符号）。 */
int16_t Motor_GetDelta(MotorId_t id);

/** @brief 累计计数（带符号，32 位，直接读，天然原子）。 */
int32_t Motor_GetTotal(MotorId_t id);

/** @brief 当前实际 PWM 输出（带符号）。 */
int32_t Motor_GetPwm(MotorId_t id);

/** @brief 当前模式：0 = 开环，1 = 速度环，2 = 位置环。 */
uint32_t Motor_GetMode(MotorId_t id);

/** @brief 机械参数是否已标定（1/0）。 */
uint8_t Motor_IsCalibrated(void);

/** @brief 编码器一圈的计数（标定值），用于遥测显示。 */
float Motor_GetCountsPerRev(void);

/** @brief FreeRTOS 任务体：定义强符号以覆盖 freertos.c 里的 __weak 空实现。 */
void StartMotorTask(void const *argument);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_H */
