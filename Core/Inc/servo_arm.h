/**
  ******************************************************************************
  * @file    servo_arm.h
  * @brief   4 路舵机（TIM8 CH1..CH4 @50Hz）+ 斜坡限速控制
  * @note    用户代码，不由 CubeMX 管理。
  *
  * 硬件：TIM8 CH1/CH2/CH3/CH4 = PC6 / PC7 / PC8 / PC9
  *       PSC = 71（1MHz，1 tick = 1us），ARR = 19999（20ms = 50Hz）
  *       脉宽 500us ~ 2500us 对应 0 ~ 180 度
  *
  * 线程约定：
  *   - Servo_Update() 只允许 ArmTask 调用（20ms 周期）；
  *   - Servo_SetXxx() 允许 CommTask 调用：内部只写 32 位整型（原子），
  *     "立即生效"由 snap 标志实现，最迟在下一个 20ms 周期被应用；
  *   - 舵机是机械件，禁止一帧跳变，常规控制请用 Servo_SetTargetAngle()。
  ******************************************************************************
  */
#ifndef __SERVO_ARM_H
#define __SERVO_ARM_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "main.h"

#define SERVO_NUM          4U

/* ============================ 可调参数 ============================ */

/** 舵机硬件开关：
 *  - 必须在 MX 里配置 TIM8（PWM Generation CH1..CH4 -> PC6..PC9, PSC=71, ARR=19999）后
 *    才能置 1；置 1 而没配 TIM8 时链接会报 "undefined reference to `htim8'"；
 *  - 没配之前先保持 0，工程可以正常编译、运行，只是舵机没有任何输出。 */
#define SERVO_HW_ENABLED        1

/** 脉宽范围（us）：不同舵机略有差异，首次调试请从 1000~2000 开始试 */
#define SERVO_MIN_US            500U
#define SERVO_MAX_US            2500U
#define SERVO_MID_US            1500U

/** 角度量程（度）：与 SERVO_MIN_US / SERVO_MAX_US 线性对应 */
#define SERVO_ANGLE_MAX         180.0f

/** 默认斜坡速度（us/秒）：越小越慢。1200 约等于 0.17s 走完全量程 */
#define SERVO_DEFAULT_SLEW      1200U

/** 控制周期（ms）：50Hz */
#define SERVO_PERIOD_MS         20U

/* ================================================================ */

/** @brief 启动 TIM8 四路 PWM，全部回中位。 */
void Servo_Init(void);

/** @brief 立即生效（不做斜坡）设定脉宽，自动限幅到 [MIN_US, MAX_US]。 */
void Servo_SetPulseUs(uint8_t idx, uint32_t us);

/** @brief 立即生效（不做斜坡）设定角度（度）。 */
void Servo_SetAngle(uint8_t idx, float deg);

/** @brief 斜坡逼近目标角度（度），推荐用这个。 */
void Servo_SetTargetAngle(uint8_t idx, float deg);

/** @brief 斜坡逼近目标脉宽（us）。 */
void Servo_SetTargetPulseUs(uint8_t idx, uint32_t us);

/** @brief 设置该路舵机的斜坡速度（us/秒），0 = 立即到位。 */
void Servo_SetSlew(uint8_t idx, uint32_t us_per_sec);

/** @brief 20ms 周期调用（ArmTask）：做斜坡并写 CCR。 */
void Servo_Update(float dt);

/** @brief 关闭输出（卸力/掉电），再次 Set 会自动恢复输出。 */
void Servo_Relax(void);

/** @brief 打开输出（默认位置保持在中位）。 */
void Servo_Enable(void);

/* ------------------------------ 只读查询 ------------------------------ */

/** @brief 当前角度 x1000（整数），跨任务安全。 */
int32_t Servo_GetAngleMilli(uint8_t idx);

/** @brief 当前脉宽（us）。 */
uint32_t Servo_GetPulseUs(uint8_t idx);

/** @brief 目标角度 x1000（整数）。 */
int32_t Servo_GetTargetAngleMilli(uint8_t idx);

/** @brief FreeRTOS 任务体：定义强符号以覆盖 freertos.c 里的 __weak 空实现。 */
void StartArmTask(void const *argument);

#ifdef __cplusplus
}
#endif

#endif /* __SERVO_ARM_H */
