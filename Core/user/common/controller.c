/**
  ******************************************************************************
  * @file    controller.c
  * @brief   通用 PID 库实现（逻辑来自旧工程 ex/user/motor/controller.c）
  *
  * 一次 PID_Calc 的完整流程（顺序不能乱）：
  *   1. 算误差 err = ref - measure；首次运行用当前值填充"上次值"
  *   2. 死区判断：|err| <= DeadBand -> 输出 0、清积分，直接跳到第 8 步
  *   3. P 项 = Kp * err
  *   4. I 项 = Ki * err * dt（可选梯形积分）
  *   5. 变积分速率：误差太大时把 I 项衰减甚至清零（防启停超调）
  *   6. D 项 = Kd * d(err)/dt，或"微分先行"= -Kd * d(measure)/dt（可选低通）
  *   7. 积分累加 + 限幅 + **抗积分饱和**：如果叠加后输出已经饱和、而且积分还在
  *      往饱和方向推，就撤销本次积分（这是比"只限幅"更强的一招）
  *   8. 输出 = P + I + D，可选输出低通，最后限幅，并保存"上次值"
  ******************************************************************************
  */
#include "controller.h"
#include <math.h>
#include <string.h>

static float LimitFloat(float value, float min, float max)
{
  if (value < min) { return min; }
  if (value > max) { return max; }
  return value;
}

void PID_Reset(PID_t *pid)
{
  if (pid == NULL)
  {
    return;
  }

  pid->Measure      = 0.0f;
  pid->Last_Measure = 0.0f;
  pid->Ref          = 0.0f;
  pid->Err          = 0.0f;
  pid->Last_Err     = 0.0f;
  pid->Pout         = 0.0f;
  pid->Iout         = 0.0f;
  pid->Dout         = 0.0f;
  pid->ITerm        = 0.0f;
  pid->Output       = 0.0f;
  pid->Last_Output  = 0.0f;
  pid->Last_Dout    = 0.0f;
  pid->initialized  = 0U;
}

void PID_Init(PID_t *pid, const PID_Config_t *config)
{
  if ((pid == NULL) || (config == NULL))
  {
    return;
  }

  memset(pid, 0, sizeof(*pid));

  pid->Kp              = config->Kp;
  pid->Ki              = config->Ki;
  pid->Kd              = config->Kd;
  pid->MaxOut          = config->MaxOut;
  pid->DeadBand        = config->DeadBand;
  pid->Improve         = config->Improve;
  pid->IntegralLimit   = config->IntegralLimit;
  pid->CoefA           = config->CoefA;
  pid->CoefB           = config->CoefB;
  pid->Output_LPF_RC   = config->Output_LPF_RC;
  pid->Derivative_LPF_RC = config->Derivative_LPF_RC;

  PID_Reset(pid);
}

void PID_SetGains(PID_t *pid, float kp, float ki, float kd)
{
  if (pid == NULL)
  {
    return;
  }

  pid->Kp = kp;
  pid->Ki = ki;
  pid->Kd = kd;
}

float PID_Calc(PID_t *pid, float measure, float ref, float dt_s)
{
  float candidate_i;
  float candidate_output;

  /* 参数非法直接返回 0，绝不产生 NaN 输出 */
  if ((pid == NULL) || (dt_s <= 0.000001f) || (pid->MaxOut <= 0.0f))
  {
    return 0.0f;
  }

  /* ---- 1. 误差 ---- */
  pid->Measure = measure;
  pid->Ref     = ref;
  pid->Err     = ref - measure;

  if (pid->initialized == 0U)
  {
    /* 第一次：把"上次值"设成当前值，避免 D 项在最开始产生一个巨大的冲击 */
    pid->Last_Err     = pid->Err;
    pid->Last_Measure = pid->Measure;
    pid->initialized  = 1U;
  }

  /* 死区：注意必须要求 DeadBand > 0 才算"启用死区"。
     旧工程写的是 fabsf(err) <= DeadBand，于是 DeadBand=0 时"误差恰好为 0 的那一拍"
     也会走进死区分支被强制清零，在目标附近产生抖动。这里加了 DeadBand > 0 的前置条件。 */
  if ((pid->DeadBand > 0.0f) && (fabsf(pid->Err) <= pid->DeadBand))
  {
    /* 死区内：不输出，并把积分放掉（防止在死区内偷偷攒积分，出去时冲一下） */
    pid->Pout   = 0.0f;
    pid->Dout   = 0.0f;
    pid->ITerm  = 0.0f;
    pid->Iout   = 0.0f;
    pid->Output = 0.0f;
  }
  else
  {
    /* ---- 3. P ---- */
    pid->Pout = pid->Kp * pid->Err;

    /* ---- 4. I（可选梯形积分） ---- */
    pid->ITerm = pid->Ki * pid->Err * dt_s;
    if ((pid->Improve & PID_Trapezoid_Integral) != 0U)
    {
      pid->ITerm = pid->Ki * (pid->Err + pid->Last_Err) * dt_s * 0.5f;
    }

    /* ---- 5. 变积分速率：误差很大时先抑制、再关闭积分 ---- */
    if ((pid->Improve & PID_ChangingIntegrationRate) != 0U)
    {
      float abs_error = fabsf(pid->Err);

      if ((pid->Err * pid->Iout) > 0.0f)   /* 只在积分方向与误差同向时才抑制 */
      {
        if ((abs_error > pid->CoefB) && (abs_error <= (pid->CoefA + pid->CoefB)))
        {
          if (pid->CoefA > 0.000001f)
          {
            pid->ITerm *= (pid->CoefA - abs_error + pid->CoefB) / pid->CoefA;
          }
        }
        else if (abs_error > (pid->CoefA + pid->CoefB))
        {
          pid->ITerm = 0.0f;
        }
      }
    }

    /* ---- 6. D：默认对误差微分；开了"微分先行"就对测量值微分 ---- */
    pid->Dout = pid->Kd * (pid->Err - pid->Last_Err) / dt_s;
    if ((pid->Improve & PID_Derivative_On_Measurement) != 0U)
    {
      pid->Dout = -pid->Kd * (pid->Measure - pid->Last_Measure) / dt_s;
    }
    if (((pid->Improve & PID_DerivativeFilter) != 0U) && (pid->Derivative_LPF_RC > 0.0f))
    {
      pid->Dout = (pid->Dout * dt_s / (pid->Derivative_LPF_RC + dt_s)) +
                  (pid->Last_Dout * pid->Derivative_LPF_RC / (pid->Derivative_LPF_RC + dt_s));
    }

    /* ---- 7. 积分累加 + 限幅 + 抗积分饱和 ---- */
    candidate_i = pid->Iout + pid->ITerm;

    if ((pid->Improve & PID_Integral_Limit) != 0U)
    {
      candidate_i = LimitFloat(candidate_i, -pid->IntegralLimit, pid->IntegralLimit);
    }

    candidate_output = pid->Pout + candidate_i + pid->Dout;

    /* 输出已经饱和、而这次积分还在朝饱和方向推 -> 撤销这次积分。
       （只做"积分限幅"的话，饱和期间积分会一直贴在限幅值上，退饱和慢一拍） */
    if (((candidate_output > pid->MaxOut) && (pid->Err > 0.0f)) ||
        ((candidate_output < -pid->MaxOut) && (pid->Err < 0.0f)))
    {
      candidate_i = pid->Iout;
    }

    pid->Iout   = candidate_i;
    pid->Output = pid->Pout + pid->Iout + pid->Dout;

    /* ---- 8. 输出低通 + 最终限幅 ---- */
    if (((pid->Improve & PID_OutputFilter) != 0U) && (pid->Output_LPF_RC > 0.0f))
    {
      pid->Output = (pid->Output * dt_s / (pid->Output_LPF_RC + dt_s)) +
                    (pid->Last_Output * pid->Output_LPF_RC / (pid->Output_LPF_RC + dt_s));
    }
    pid->Output = LimitFloat(pid->Output, -pid->MaxOut, pid->MaxOut);
  }

  /* 无论走哪个分支，都要把"上次值"更新掉，否则下一个周期的微分/积分是错的 */
  pid->Last_Measure = pid->Measure;
  pid->Last_Err     = pid->Err;
  pid->Last_Output  = pid->Output;
  pid->Last_Dout    = pid->Dout;

  return pid->Output;
}
