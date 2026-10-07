/**
  ******************************************************************************
  * @file    chassis.c
  * @brief   4 轮麦克纳姆轮底盘运动学实现
  *
  * 运动学（X 形滚轮布局：底盘静止时俯视四个滚轮的轴线呈 X）：
  *   w1 左前 = +vx - vy - (L+W)*wz
  *   w2 右前 = +vx + vy + (L+W)*wz
  *   w3 左后 = +vx + vy - (L+W)*wz
  *   w4 右后 = +vx - vy + (L+W)*wz
  * 其中 L = CHASSIS_HALF_LENGTH_M，W = CHASSIS_HALF_WIDTH_M。
  * 求出的轮速是"轮子接地点线速度"(m/s)，再乘 cps_per_mps 换算成编码器计数/秒：
  *   cps_per_mps = MOTOR_COUNTS_PER_REV / (pi * 轮径)
  *
  * 为什么底盘方向标定在本模块而不是 motor.c 的 s_invert[]：
  *   - s_invert 同时镜像"PWM 输出 + 编码器测速"，用来修单电机的正方向；
  *   - "哪个电机装在哪个角、正 cps 对不对应车头方向"是底盘层面的映射，
  *     放在 s_corner_motor[] / s_mix[] / s_dir[] 三张表里，改表不改代码。
  ******************************************************************************
  */
#include "chassis.h"
#include "../motor/motor.h"

#include <math.h>

/* ==================== 布局映射表（方向/接线标定只改这里，见 参数.md） ==================== */

/** 轮位角 -> 电机编号（MotorId_t 下标）。默认 1左前=M1、2右前=M2、3左后=M3、4右后=M4。
 *  实车接线不同时改这张表即可，运动学公式不用动。注意表长必须等于 MOTOR_NUM。 */
static const uint8_t s_corner_motor[MOTOR_NUM] = { 0U, 1U, 2U, 3U };

/** X 形滚轮布局的运动学符号 {kx, ky, kw}（含义见文件头公式），按"轮位角"排列。
 *  若实车俯视滚轮呈 O 形（与 X 形相反），把所有 ky 取反即可。 */
static const float s_mix[MOTOR_NUM][3] =
{
  /*  kx     ky     kw      轮位角 */
  {  1.0f, -1.0f, -1.0f },    /* 1 左前 FL */
  {  1.0f,  1.0f,  1.0f },    /* 2 右前 FR */
  {  1.0f,  1.0f, -1.0f },    /* 3 左后 RL */
  {  1.0f, -1.0f,  1.0f }     /* 4 右后 RR */
};

/** 每个轮位的底盘方向符号（+1/-1）：整车测试时某轮推力方向反了就取反对应位。
 *  只改目标符号、不动 motor.c 的 s_invert（那是修电机本体正方向的，闭环同样安全，
 *  但两者不要混用——先按 参数.md 的流程用 s_dir 标定底盘方向）。 */
static const float s_dir[MOTOR_NUM] = { 1.0f, 1.0f, 1.0f, 1.0f };

/* ==================== 内部状态（只记录最近一次指令，供查询/遥测） ==================== */

static volatile int32_t s_vx_milli = 0;
static volatile int32_t s_vy_milli = 0;
static volatile int32_t s_wz_milli = 0;

/* ==================== 内部函数 ==================== */

/** 轮子线速度(m/s) -> 编码器计数/秒 的换算系数 */
static float chassis_cps_per_mps(void)
{
  return (float)MOTOR_COUNTS_PER_REV / (3.14159265f * CHASSIS_WHEEL_DIAM_M);
}

/** 四舍五入取整（符号正确）。直接截断会有 0.5cps 的系统性偏差，这里避免。 */
static int32_t chassis_round(float v)
{
  return (int32_t)((v >= 0.0f) ? (v + 0.5f) : (v - 0.5f));
}

/* ==================== 对外接口 ==================== */

uint8_t Chassis_SetVelocity(float vx_mps, float vy_mps, float wz_radps)
{
  const float half_diag   = CHASSIS_HALF_LENGTH_M + CHASSIS_HALF_WIDTH_M;
  const float cps_per_mps = chassis_cps_per_mps();
  float wheel_cps[MOTOR_NUM];
  float max_abs;
  float vx, vy, wz;
  uint32_t corner, i;

  /* 0. 非法输入直接拒绝（保持当前运动），由调用方决定如何提示 */
  if ((!isfinite(vx_mps)) || (!isfinite(vy_mps)) || (!isfinite(wz_radps)))
  {
    return 0U;
  }

  /* ---- 1. 输入限幅：静默裁剪，便于上位机连续流式发送 ---- */
  vx = vx_mps;
  vy = vy_mps;
  wz = wz_radps;
  if (vx >  (float)CHASSIS_VX_MAX_MPS)   { vx =  (float)CHASSIS_VX_MAX_MPS; }
  if (vx < -(float)CHASSIS_VX_MAX_MPS)   { vx = -(float)CHASSIS_VX_MAX_MPS; }
  if (vy >  (float)CHASSIS_VY_MAX_MPS)   { vy =  (float)CHASSIS_VY_MAX_MPS; }
  if (vy < -(float)CHASSIS_VY_MAX_MPS)   { vy = -(float)CHASSIS_VY_MAX_MPS; }
  if (wz >  (float)CHASSIS_WZ_MAX_RADPS) { wz =  (float)CHASSIS_WZ_MAX_RADPS; }
  if (wz < -(float)CHASSIS_WZ_MAX_RADPS) { wz = -(float)CHASSIS_WZ_MAX_RADPS; }

  /* ---- 2. 运动学：轮位角 -> 轮子线速度(m/s) -> 轮速(cps)，按下发电机的编号存放 ---- */
  for (corner = 0U; corner < MOTOR_NUM; corner++)
  {
    float v_lin = (s_mix[corner][0] * vx) +
                  (s_mix[corner][1] * vy) +
                  (s_mix[corner][2] * half_diag * wz);
    float cps   = s_dir[corner] * v_lin * cps_per_mps;

    wheel_cps[s_corner_motor[corner]] = cps;
  }

  /* ---- 3. 归一化：任一轮超出速度上限时全体等比缩小，保持合成运动方向不变 ----
     （MOTOR_MAX_SPEED_CPS 是位置环输出的速度天花板，这里复用它做全轮上限） */
  max_abs = 0.0f;
  for (i = 0U; i < MOTOR_NUM; i++)
  {
    float a = fabsf(wheel_cps[i]);
    if (a > max_abs)
    {
      max_abs = a;
    }
  }
  if (max_abs > (float)MOTOR_MAX_SPEED_CPS)
  {
    const float scale = (float)MOTOR_MAX_SPEED_CPS / max_abs;

    for (i = 0U; i < MOTOR_NUM; i++)
    {
      wheel_cps[i] *= scale;
    }
  }

  /* ---- 4. 下发：先退出残留的位置环（POS/VEL 互斥），再设四轮速度目标 ----
     只有位置环真的在用才发"退出"，避免每条 VEL 都白白清一次 PID 积分。 */
  for (i = 0U; i < MOTOR_NUM; i++)
  {
    if (Motor_GetMode((MotorId_t)i) == 2U)
    {
      Motor_EnablePositionLoop((MotorId_t)i, 0U);
    }
    Motor_SetTargetSpeed((MotorId_t)i, chassis_round(wheel_cps[i]));
  }

  /* ---- 5. 记录最近一次（裁剪后的）指令，供 Chassis_GetVelocityMilli 查询 ---- */
  s_vx_milli = chassis_round(vx * 1000.0f);
  s_vy_milli = chassis_round(vy * 1000.0f);
  s_wz_milli = chassis_round(wz * 1000.0f);

  return 1U;
}

void Chassis_Stop(void)
{
  (void)Chassis_SetVelocity(0.0f, 0.0f, 0.0f);
}

void Chassis_GetVelocityMilli(int32_t *vx_milli, int32_t *vy_milli, int32_t *wz_milli)
{
  if (vx_milli != NULL) { *vx_milli = s_vx_milli; }
  if (vy_milli != NULL) { *vy_milli = s_vy_milli; }
  if (wz_milli != NULL) { *wz_milli = s_wz_milli; }
}
