/**
  ******************************************************************************
  * @file    controller.h
  * @brief   通用 PID 库（可复用的独立模块，不含任何硬件依赖）
  * @note    吸收自旧工程 ex/user/motor/controller.c，并做了 3 处整理：
  *            1) 删掉旧版"声明了但从未实现"的 PID_Proportional_On_Measurement
  *               和 PID_ErrorHandle 两个改进位（避免看代码的人以为能用）；
  *            2) 补了 PID_SetGains()，让串口命令能在线改增益；
  *            3) 抗积分饱和（输出饱和时取消本次积分）写成显式注释的独立步骤。
  *
  * 单位说明：本库不关心单位，只要保证 measure / ref / MaxOut 三者同量纲
  *   （例如：速度环用"编码器计数/秒"，位置环用"度 -> 计数/秒"）。
  ******************************************************************************
  */
#ifndef __CONTROLLER_H
#define __CONTROLLER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/** PID 改进项开关（按位或，写进 PID_Config_t.Improve） */
typedef enum
{
  PID_IMPROVE_NONE               = 0U,
  PID_Integral_Limit             = 1U << 0,  /* 积分项单独限幅 */
  PID_Derivative_On_Measurement  = 1U << 1,  /* 微分先行：对测量值微分，抑制给定跳变冲击 */
  PID_Trapezoid_Integral         = 1U << 2,  /* 梯形积分：用 (err+last_err)/2 提高精度 */
  PID_OutputFilter               = 1U << 3,  /* 输出一阶低通 */
  PID_ChangingIntegrationRate    = 1U << 4,  /* 变积分速率：误差大时抑制/关闭积分（参考 CoefA/CoefB）*/
  PID_DerivativeFilter           = 1U << 5   /* 微分一阶低通 */
} PID_Improve_e;

/** PID 配置（只在初始化时读一次） */
typedef struct
{
  float Kp;
  float Ki;
  float Kd;
  float MaxOut;              /* 输出限幅（正负对称） */
  float DeadBand;            /* 误差死区：|err| 小于它时输出 0 并清积分 */
  PID_Improve_e Improve;     /* 上面的改进位组合 */
  float IntegralLimit;       /* 积分项限幅（PID_Integral_Limit 生效时） */
  float CoefA;               /* 变积分速率：误差 > CoefA+CoefB 时不积分 */
  float CoefB;               /* 变积分速率：误差 > CoefB 时按斜率衰减积分 */
  float Output_LPF_RC;       /* 输出低通时间常数（秒），0 = 不滤波 */
  float Derivative_LPF_RC;   /* 微分低通时间常数（秒），0 = 不滤波 */
} PID_Config_t;

/** PID 运行实例（含全部中间状态） */
typedef struct
{
  /* 配置副本 */
  float Kp;
  float Ki;
  float Kd;
  float MaxOut;
  float DeadBand;
  PID_Improve_e Improve;
  float IntegralLimit;
  float CoefA;
  float CoefB;
  float Output_LPF_RC;
  float Derivative_LPF_RC;

  /* 运行状态 */
  float Measure;
  float Last_Measure;
  float Ref;
  float Err;
  float Last_Err;
  float Pout;
  float Iout;
  float Dout;
  float ITerm;
  float Output;
  float Last_Output;
  float Last_Dout;
  uint8_t initialized;       /* 第一次计算时用当前值初始化"上次值"，避免微分冲击 */
} PID_t;

/** @brief 按配置初始化（会把内部状态全部清零）。 */
void PID_Init(PID_t *pid, const PID_Config_t *config);

/** @brief 清零运行状态（保留增益配置），换目标/重新使能时调用。 */
void PID_Reset(PID_t *pid);

/** @brief 在线修改增益（其余配置不变）。 */
void PID_SetGains(PID_t *pid, float kp, float ki, float kd);

/** @brief 计算一步。
 *  @param measure 当前测量值
 *  @param ref     目标值
 *  @param dt_s    距上次调用的时间（秒），必须 > 0
 *  @return 限幅后的输出；参数非法时返回 0。 */
float PID_Calc(PID_t *pid, float measure, float ref, float dt_s);

#ifdef __cplusplus
}
#endif

#endif /* __CONTROLLER_H */
