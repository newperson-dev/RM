/**
  ******************************************************************************
  * @file    chassis.h
  * @brief   4 轮麦克纳姆轮底盘运动学：vx / vy / wz -> 四轮速度目标
  * @note    目录按任务职责划分：Core/user/chassis/
  *
  * 依赖方向：comm -> chassis -> motor -> common（单向，不反向）。
  *
  * 本模块是纯运动学映射，**没有周期任务、没有控制状态**：
  *   - Chassis_SetVelocity() 可在任意任务上下文调用（内部只调 Motor_* 公开 API，
  *     只写 32 位整型共享变量，符合 motor.c 的线程约定）；
  *   - "轮速不跳变"由 motor.c 的 MOTOR_TARGET_SLEW_CPS_PER_S 目标斜坡保证，
  *     本模块不做斜坡，因此也不需要自己的周期任务。
  *
  * 坐标系约定（俯视，右手系，见 参数.md）：
  *   vx = 机体前方为正，单位 m/s
  *   vy = 机体左方为正，单位 m/s
  *   wz = 逆时针（俯视）旋转为正，单位 rad/s
  *
  * 轮位编号（默认预设，接线不同改 chassis.c 的 s_corner_motor[]，标定见 参数.md）：
  *        1 左前(FL)    2 右前(FR)
  *        3 左后(RL)    4 右后(RR)
  ******************************************************************************
  */
#ifndef __CHASSIS_H
#define __CHASSIS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ==================== 可调参数（每项的调整时机详见 参数.md） ==================== */

/** 麦轮直径（米）。直接决定 vx/vy 的换算精度，必须按实车轮径修改。当前预设 65mm。 */
#define CHASSIS_WHEEL_DIAM_M      0.065f

/** 底盘几何（米）：半轴距(x 向) / 半轮距(y 向)，按"对面两轮接地中心的间距/2"取值。
 *  预设对应约 70cm x 70cm 底盘。 */
#define CHASSIS_HALF_LENGTH_M     0.35f
#define CHASSIS_HALF_WIDTH_M      0.35f

/** 底盘速度限幅：输入超限会被静默裁剪；四轮合成后任一轮超速时全体等比缩小。
 *  预设依据：单轮 8500 cps x 65mm 轮约 1.11 m/s，给旋转分量留混合裕量。 */
#define CHASSIS_VX_MAX_MPS        0.8f
#define CHASSIS_VY_MAX_MPS        0.8f
#define CHASSIS_WZ_MAX_RADPS      1.5f

/* ================================================================ */

/** @brief 设定底盘速度。轮速斜坡由 motor.c 完成，指令本身立即生效。
 *  @param vx_mps   前进速度 m/s（前+）
 *  @param vy_mps   横移速度 m/s（左+）
 *  @param wz_radps 旋转角速度 rad/s（俯视逆时针+）
 *  @retval 1 = 已接受；0 = 被拒绝（输入含 NaN/Inf，保持当前运动不变）。
 *  @note  自动退出残留的位置环（POS 与 VEL 互斥，VEL 优先）。
 *  @note  前提：四个电机已用 PID 指令配置增益（默认增益为 0 时闭环输出恒 0，
 *         这是 motor.h 的安全设计——底盘不会自己动）。 */
uint8_t Chassis_SetVelocity(float vx_mps, float vy_mps, float wz_radps);

/** @brief 平滑停止：目标归零并按斜坡降速（保持闭环）。
 *  @note  紧急停机请用串口 STOP 命令（立即退出闭环并滑行）。 */
void Chassis_Stop(void);

/** @brief 读取最近一次接受的底盘速度指令（x1000 整型，被裁剪后的值）。
 *  @note  任一输出指针可为 NULL；跨任务安全（内部 32 位整型）。 */
void Chassis_GetVelocityMilli(int32_t *vx_milli, int32_t *vy_milli, int32_t *wz_milli);

#ifdef __cplusplus
}
#endif

#endif /* __CHASSIS_H */
