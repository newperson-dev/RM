/**
  ******************************************************************************
  * @file    comm.c
  * @brief   USART2 指令接收与调试回传实现（目录：Core/user/comm/）
  *
  * 收数据：DMA 循环缓冲 + 轮询 CNDTR（零中断，吸收自旧工程 ex/user/remotor/remotor.c）
  * 发数据：寄存器级写 DR（不碰 HAL 的 __HAL_LOCK，否则会和接收路径抢锁）
  * 解析：CommTask 里逐字节组行，行尾 \n 触发命令分发；命令直接调用控制模块 API。
  *
  * 为什么接收必须是"零中断"：
  *   HAL_UART_Receive_DMA() 会顺带打开 UART 的错误中断（CR3.EIE）。一旦开了 NVIC，
  *   ORE/NE/FE 任一触发就会进 HAL_UART_IRQHandler -> UART_EndRxTransfer()，
  *   它会把 CR3.DMAR 清掉 —— 接收 DMA 就此停摆（表现：串口突然永久收不到数据）。
  *   而且它清标志靠"读 SR + 读 DR"，读 DR 会从 DMA 手里抢走一个字节，造成数据错位。
  *   同一原因，DMA1_Channel6 的 NVIC 也关掉：Circular 模式下 TC 回调没有意义，
  *   反而多出每 512 字节一次的无效中断。
  ******************************************************************************
  */
#include "comm.h"
#include "../motor/motor.h"
#include "../arm/servo_arm.h"
#include "../chassis/chassis.h"
#include "usart.h"

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

#include <string.h>
#include <stdlib.h>

/* ========================== 内部状态 ========================== */

static uint8_t  s_dma_buf[COMM_DMA_BUF_SIZE];   /* DMA 循环缓冲 */
static uint16_t s_dma_last_wpos = 0U;           /* 上次已消费到的 DMA 写位置 */

/* 行缓冲：只在 CommTask 里使用，不需要保护 */
static char     s_line[160];
static uint16_t s_len = 0U;

/* 遥测：只在 CommTask 里读写 */
static uint8_t  s_telem_on     = 0U;
static uint32_t s_telem_period = COMM_TELEM_PERIOD_MS;

/* ========================== 寄存器级发送 ========================== */

static void tx_byte(uint8_t b)
{
  uint32_t guard = 2000000U;

  while (((USART2->SR & USART_SR_TXE) == 0U) && (guard != 0U))
  {
    guard--;
  }
  USART2->DR = (uint16_t)b;
}

static void tx_str(const char *s)
{
  while (*s != '\0')
  {
    tx_byte((uint8_t)(*s));
    s++;
  }
}

/* ========================== 行缓冲组装 ========================== */

static void line_reset(void)
{
  s_len     = 0U;
  s_line[0] = '\0';
}

static void line_putc(char c)
{
  if (s_len < (uint16_t)(sizeof(s_line) - 1U))
  {
    s_line[s_len] = c;
    s_len++;
    s_line[s_len] = '\0';
  }
}

static void line_puts(const char *s)
{
  while (*s != '\0')
  {
    line_putc(*s);
    s++;
  }
}

static void line_uint(uint32_t u)
{
  char tmp[11];
  int  n = 0;

  if (u == 0U)
  {
    line_putc('0');
    return;
  }
  while ((u != 0U) && (n < 10))
  {
    tmp[n] = (char)('0' + (u % 10U));
    n++;
    u /= 10U;
  }
  while (n > 0)
  {
    n--;
    line_putc(tmp[n]);
  }
}

static void line_int(int32_t v)
{
  if (v < 0)
  {
    line_putc('-');
    line_uint((uint32_t)(-v));
  }
  else
  {
    line_uint((uint32_t)v);
  }
}

/** 打印 xxx.y（输入是 x1000 的整数） */
static void line_fixed1(int32_t milli)
{
  int32_t ip;
  int32_t fp;

  if (milli < 0)
  {
    line_putc('-');
    milli = -milli;
  }
  ip = milli / 1000;
  fp = milli % 1000;

  line_int(ip);
  line_putc('.');
  line_putc((char)('0' + (fp / 100)));
}

static void line_flush(void)
{
  line_puts("\r\n");
  tx_str(s_line);
}

/* ========================== 参数解析 ========================== */

/** 取下一个以空格分隔的 token；没有则返回 NULL。
 *  用 strtok 而不是 strtok_r：newlib 的 strtok_r 需要 _POSIX_C_SOURCE 宏，
 *  本工程的 CMakeLists 是自动生成的、不宜修改。解析只在 CommTask 里发生，
 *  strtok 的静态状态不会与其他任务/中断冲突。 */
static char *arg_next(char **save)
{
  (void)save;
  return strtok(NULL, " \t");
}

/** 取下一个整数参数，成功返回 1 */
static uint8_t arg_int(char **save, int32_t *out)
{
  char *t = arg_next(save);

  if (t == NULL)
  {
    return 0U;
  }
  *out = (int32_t)strtol(t, NULL, 10);
  return 1U;
}

/** 解析定点数（支持 12 / 12.3 / -0.5），返回 x1000 的整数 */
static int32_t parse_milli(const char *s, uint8_t *ok)
{
  int32_t ip = 0;
  int32_t fp = 0;
  int32_t sign = 1;
  int     digits = 0;

  *ok = 0U;
  if (*s == '-')
  {
    sign = -1;
    s++;
  }
  while ((*s >= '0') && (*s <= '9'))
  {
    ip = (ip * 10) + (int32_t)(*s - '0');
    s++;
    *ok = 1U;
  }
  if (*s == '.')
  {
    s++;
    while ((*s >= '0') && (*s <= '9') && (digits < 3))
    {
      fp = (fp * 10) + (int32_t)(*s - '0');
      s++;
      digits++;
      *ok = 1U;
    }
    while (digits < 3)
    {
      fp *= 10;
      digits++;
    }
  }

  return (sign > 0) ? ((ip * 1000) + fp) : (-((ip * 1000) + fp));
}

/** 取一个定点数参数（x1000），成功返回 1 */
static uint8_t arg_milli(char **save, int32_t *out_milli)
{
  char   *t = arg_next(save);
  uint8_t ok;

  if (t == NULL)
  {
    return 0U;
  }
  *out_milli = parse_milli(t, &ok);
  return ok;
}

/** 参数是否为 1..n 的编号，返回 0 基下标；非法返回 0xFF */
static uint8_t arg_index(char **save, uint32_t n)
{
  int32_t v;

  if (arg_int(save, &v) == 0U)
  {
    return 0xFFU;
  }
  if ((v < 1) || ((uint32_t)v > n))
  {
    return 0xFFU;
  }
  return (uint8_t)(v - 1);
}

/* ========================== 遥测与信息 ========================== */

/** 打印 4 个电机的计数/速度/转速/角度/PWM */
static void comm_send_enc(void)
{
  uint8_t i;

  line_reset();
  line_puts("ENC");
  for (i = 0U; i < MOTOR_NUM; i++)
  {
    line_putc(' ');
    line_int(Motor_GetTotal((MotorId_t)i));
  }

  line_puts(" | SPD");
  for (i = 0U; i < MOTOR_NUM; i++)
  {
    line_putc(' ');
    line_fixed1(Motor_GetSpeedMilli((MotorId_t)i));
  }

  if (Motor_IsCalibrated() != 0U)
  {
    line_puts(" | RPM");
    for (i = 0U; i < MOTOR_NUM; i++)
    {
      line_putc(' ');
      line_fixed1(Motor_GetSpeedRpmMilli((MotorId_t)i));
    }
    line_puts(" | POS");
    for (i = 0U; i < MOTOR_NUM; i++)
    {
      line_putc(' ');
      line_fixed1(Motor_GetAngleMilliDeg((MotorId_t)i));
    }
  }

  line_puts(" | PWM");
  for (i = 0U; i < MOTOR_NUM; i++)
  {
    line_putc(' ');
    line_int(Motor_GetPwm((MotorId_t)i));
  }
  line_flush();
}

static void comm_send_servo(void)
{
  uint8_t i;

  line_reset();
  line_puts("SV");
  for (i = 0U; i < SERVO_NUM; i++)
  {
    line_putc(' ');
    line_fixed1(Servo_GetAngleMilli(i));
  }
  line_flush();
}

static void comm_send_info(void)
{
  uint8_t i;

  line_reset();
  line_puts("INFO CAL=");
  line_putc((Motor_IsCalibrated() != 0U) ? '1' : '0');
  line_puts(" CPR=");
  line_fixed1((int32_t)(Motor_GetCountsPerRev() * 1000.0f));
  line_puts(" MODE=");
  for (i = 0U; i < MOTOR_NUM; i++)
  {
    if (i != 0U)
    {
      line_putc(',');
    }
    line_int((int32_t)Motor_GetMode((MotorId_t)i));   /* 0=开环 1=速度环 2=位置环 */
  }
  line_puts(" RX=DMAPOLL");
  line_puts(" BUF=");
  line_int((int32_t)COMM_DMA_BUF_SIZE);
  line_puts(" POLL=");
  line_int((int32_t)COMM_DMA_POLL_MS);
  line_puts("ms");
  line_flush();
}

/* ========================== 命令表 ========================== */

static void comm_print_help(void)
{
  tx_str("== RM commands ==\r\n");
  tx_str("HELP | INFO | RXRST | STOP | ZERO <1-4>|ZERO\r\n");
  tx_str("M   <1-4> <pwm>            open-loop pwm (-3599..3599)\r\n");
  tx_str("SPD <1-4> <counts/s>       speed loop target (full 8580)\r\n");
  tx_str("SPDR<1-4> <rpm>            speed loop target (rpm, full 330)\r\n");
  tx_str("PID <1-4> <kp> <ki> <kd>   speed loop gains x1000\r\n");
  tx_str("PIDEN <1-4> <0|1>          speed loop enable\r\n");
  tx_str("POS <1-4> <deg>            position loop target (ZERO first)\r\n");
  tx_str("POSEN <1-4> <0|1>          position loop enable\r\n");
  tx_str("PIDP <1-4> <kp> <ki> <kd>  position loop gains x1000\r\n");
  tx_str("VEL <vx> <vy> <wz>        chassis twist (m/s, m/s, rad/s)\r\n");
  tx_str("S   <1-4> <deg>            servo jump  | SS <1-4> <deg> servo slew\r\n");
  tx_str("SP  <1-4> <us>             servo pulse (500-2500)\r\n");
  tx_str("SLEW <1-4> <us/s>  RELAX  ARMED\r\n");
  tx_str("TELEM <0|1> [period_ms]    telemetry\r\n");
  tx_str("ENC                        print once\r\n");
}

static void comm_handle_line(char *line)
{
  char   *save = NULL;
  char   *cmd;
  uint8_t idx;

  cmd = strtok(line, " \t");
  if (cmd == NULL)
  {
    return;
  }

  /* ------------------------- 系统类 ------------------------- */
  if (strcmp(cmd, "HELP") == 0)
  {
    comm_print_help();
  }
  else if (strcmp(cmd, "INFO") == 0)
  {
    comm_send_info();
  }
  else if (strcmp(cmd, "RXRST") == 0)
  {
    Comm_RxRestart();
    tx_str("OK RXRST\r\n");
  }
  else if (strcmp(cmd, "STOP") == 0)
  {
    Motor_StopAll();
    tx_str("OK STOP\r\n");
  }
  else if (strcmp(cmd, "ZERO") == 0)
  {
    int32_t v;

    /* ZERO        -> 全部清零
       ZERO <1-4>  -> 指定电机清零（位置环用之前必须先定零点） */
    if (arg_int(&save, &v) == 0U)
    {
      Motor_ResetAllCounts();
      tx_str("OK ZERO ALL\r\n");
    }
    else if ((v >= 1) && ((uint32_t)v <= MOTOR_NUM))
    {
      Motor_ResetCount((MotorId_t)(v - 1));
      tx_str("OK ZERO\r\n");
    }
    else
    {
      tx_str("ERR usage: ZERO [1-4]\r\n");
    }
  }

  /* ------------------------- 电机 ------------------------- */
  else if (strcmp(cmd, "M") == 0)
  {
    int32_t pwm;
    idx = arg_index(&save, MOTOR_NUM);
    if ((idx == 0xFFU) || (arg_int(&save, &pwm) == 0U))
    {
      tx_str("ERR usage: M <1-4> <pwm>\r\n");
      return;
    }
    Motor_SetPwm((MotorId_t)idx, pwm);
    line_reset();
    line_puts("OK M");
    line_int((int32_t)idx + 1);
    line_putc(' ');
    line_int(pwm);
    line_flush();
  }
  else if (strcmp(cmd, "SPD") == 0)
  {
    int32_t cps;
    idx = arg_index(&save, MOTOR_NUM);
    if ((idx == 0xFFU) || (arg_int(&save, &cps) == 0U))
    {
      tx_str("ERR usage: SPD <1-4> <counts/s>\r\n");
      return;
    }
    Motor_SetTargetSpeed((MotorId_t)idx, cps);
    tx_str("OK SPD\r\n");
  }
  else if (strcmp(cmd, "SPDR") == 0)
  {
    int32_t milli;
    idx = arg_index(&save, MOTOR_NUM);
    if ((idx == 0xFFU) || (arg_milli(&save, &milli) == 0U))
    {
      tx_str("ERR usage: SPDR <1-4> <rpm>\r\n");
      return;
    }
    if (Motor_SetTargetSpeedRpm((MotorId_t)idx, (float)milli * 0.001f) == 0U)
    {
      tx_str("ERR not calibrated (set PPR/GEAR in motor.h)\r\n");
      return;
    }
    tx_str("OK SPDR\r\n");
  }
  else if (strcmp(cmd, "PID") == 0)
  {
    int32_t kp, ki, kd;
    idx = arg_index(&save, MOTOR_NUM);
    if ((idx == 0xFFU) || (arg_int(&save, &kp) == 0U) ||
        (arg_int(&save, &ki) == 0U) || (arg_int(&save, &kd) == 0U))
    {
      tx_str("ERR usage: PID <1-4> <kp> <ki> <kd> (x1000)\r\n");
      return;
    }
    Motor_SetPidMilli((MotorId_t)idx, kp, ki, kd);
    tx_str("OK PID\r\n");
  }
  else if (strcmp(cmd, "PIDEN") == 0)
  {
    int32_t en;
    idx = arg_index(&save, MOTOR_NUM);
    if ((idx == 0xFFU) || (arg_int(&save, &en) == 0U))
    {
      tx_str("ERR usage: PIDEN <1-4> <0|1>\r\n");
      return;
    }
    Motor_EnableClosedLoop((MotorId_t)idx, (en != 0) ? 1U : 0U);
    tx_str("OK PIDEN\r\n");
  }
  else if (strcmp(cmd, "POS") == 0)
  {
    int32_t milli;
    idx = arg_index(&save, MOTOR_NUM);
    if ((idx == 0xFFU) || (arg_milli(&save, &milli) == 0U))
    {
      tx_str("ERR usage: POS <1-4> <deg>\r\n");
      return;
    }
    if (Motor_SetTargetAngle((MotorId_t)idx, (float)milli * 0.001f) == 0U)
    {
      tx_str("ERR not calibrated (set PPR/GEAR in motor.h)\r\n");
      return;
    }
    tx_str("OK POS\r\n");
  }
  else if (strcmp(cmd, "POSEN") == 0)
  {
    int32_t en;
    idx = arg_index(&save, MOTOR_NUM);
    if ((idx == 0xFFU) || (arg_int(&save, &en) == 0U))
    {
      tx_str("ERR usage: POSEN <1-4> <0|1>\r\n");
      return;
    }
    Motor_EnablePositionLoop((MotorId_t)idx, (en != 0) ? 1U : 0U);
    tx_str("OK POSEN\r\n");
  }
  else if (strcmp(cmd, "PIDP") == 0)
  {
    int32_t kp, ki, kd;
    idx = arg_index(&save, MOTOR_NUM);
    if ((idx == 0xFFU) || (arg_int(&save, &kp) == 0U) ||
        (arg_int(&save, &ki) == 0U) || (arg_int(&save, &kd) == 0U))
    {
      tx_str("ERR usage: PIDP <1-4> <kp> <ki> <kd> (x1000)\r\n");
      return;
    }
    Motor_SetPosPidMilli((MotorId_t)idx, kp, ki, kd);
    tx_str("OK PIDP\r\n");
  }

  /* ------------------------- 底盘（麦轮运动学） ------------------------- */
  else if (strcmp(cmd, "VEL") == 0)
  {
    int32_t vx, vy, wz;

    if ((arg_milli(&save, &vx) == 0U) || (arg_milli(&save, &vy) == 0U) ||
        (arg_milli(&save, &wz) == 0U))
    {
      tx_str("ERR usage: VEL <vx_mps> <vy_mps> <wz_radps>\r\n");
      return;
    }
    if (Chassis_SetVelocity((float)vx * 0.001f,
                            (float)vy * 0.001f,
                            (float)wz * 0.001f) == 0U)
    {
      tx_str("ERR VEL rejected (NaN/Inf)\r\n");
      return;
    }
    tx_str("OK VEL\r\n");
  }

  /* ------------------------- 舵机 ------------------------- */
  else if ((strcmp(cmd, "S") == 0) || (strcmp(cmd, "SS") == 0))
  {
    int32_t milli;
    uint8_t slew = (cmd[1] == 'S') ? 1U : 0U;   /* SS = 斜坡，S = 直接到 */

    idx = arg_index(&save, SERVO_NUM);
    if ((idx == 0xFFU) || (arg_milli(&save, &milli) == 0U))
    {
      tx_str("ERR usage: S/SS <1-4> <deg>\r\n");
      return;
    }
    if (slew != 0U)
    {
      Servo_SetTargetAngle(idx, (float)milli * 0.001f);
    }
    else
    {
      Servo_SetAngle(idx, (float)milli * 0.001f);
    }
    tx_str("OK\r\n");
  }
  else if (strcmp(cmd, "SP") == 0)
  {
    int32_t us;
    idx = arg_index(&save, SERVO_NUM);
    if ((idx == 0xFFU) || (arg_int(&save, &us) == 0U))
    {
      tx_str("ERR usage: SP <1-4> <us>\r\n");
      return;
    }
    Servo_SetPulseUs(idx, (uint32_t)us);
    tx_str("OK\r\n");
  }
  else if (strcmp(cmd, "SLEW") == 0)
  {
    int32_t us_per_s;
    idx = arg_index(&save, SERVO_NUM);
    if ((idx == 0xFFU) || (arg_int(&save, &us_per_s) == 0U) || (us_per_s < 0))
    {
      tx_str("ERR usage: SLEW <1-4> <us/s>\r\n");
      return;
    }
    Servo_SetSlew(idx, (uint32_t)us_per_s);
    tx_str("OK\r\n");
  }
  else if (strcmp(cmd, "RELAX") == 0)
  {
    Servo_Relax();
    tx_str("OK RELAX\r\n");
  }
  else if (strcmp(cmd, "ARMED") == 0)
  {
    Servo_Enable();
    tx_str("OK ARMED\r\n");
  }
  else if (strcmp(cmd, "TELEM") == 0)
  {
    int32_t on;
    int32_t period;

    if (arg_int(&save, &on) == 0U)
    {
      tx_str("ERR usage: TELEM <0|1> [period_ms]\r\n");
      return;
    }
    s_telem_on = (on != 0) ? 1U : 0U;
    if (arg_int(&save, &period) != 0U)
    {
      if (period < (int32_t)COMM_DMA_POLL_MS)
      {
        period = (int32_t)COMM_DMA_POLL_MS;
      }
      s_telem_period = (uint32_t)period;
    }
    tx_str((s_telem_on != 0U) ? "OK TELEM ON\r\n" : "OK TELEM OFF\r\n");
  }
  else if (strcmp(cmd, "ENC") == 0)
  {
    comm_send_enc();
  }
  else
  {
    line_reset();
    line_puts("ERR unknown: ");
    line_puts(cmd);
    line_flush();
  }
}

/* ========================== 行解析 ========================== */

/** 逐字节组行；'\n' 表示一行结束，退格用于手工敲错时修正 */
static void comm_feed_byte(char c, char *line, uint16_t *len)
{
  if (c == '\r')
  {
    return;
  }
  if (c == '\n')
  {
    if (*len > 0U)
    {
      line[*len] = '\0';
      comm_handle_line(line);
    }
    *len = 0U;
  }
  else if ((c == '\b') || (c == 0x7F))
  {
    if (*len > 0U)
    {
      (*len)--;
    }
  }
  else if (*len < (uint16_t)(COMM_LINE_MAX - 1U))
  {
    line[*len] = c;
    (*len)++;
  }
  else
  {
    *len = 0U;
    Comm_LogLine("ERR line too long");
  }
}

/* ========================== DMA 写指针 ========================== */

/** 读取 DMA 已搬运字节数 -> 写指针（缓冲区中的下一个写入位置） */
static uint16_t comm_dma_wpos(void)
{
  uint16_t ndtr;

  if ((huart2.hdmarx == NULL) || (huart2.hdmarx->Instance == NULL))
  {
    return 0U;
  }

  /* 只读寄存器，不触发任何回调/中断 */
  ndtr = (uint16_t)(huart2.hdmarx->Instance->CNDTR);
  if (ndtr > COMM_DMA_BUF_SIZE)
  {
    ndtr = COMM_DMA_BUF_SIZE;
  }
  return (uint16_t)((COMM_DMA_BUF_SIZE - ndtr) & (COMM_DMA_BUF_SIZE - 1U));
}

/* ========================== 初始化 ========================== */

/** 关掉两个不该有的中断源（纯轮询方案，理由见文件头注释） */
static void comm_disable_irqs(void)
{
  HAL_NVIC_DisableIRQ(USART2_IRQn);
  HAL_NVIC_DisableIRQ(DMA1_Channel6_IRQn);
}

void Comm_Init(void)
{
  /* 先关中断源：即使下面的 DMA 装载失败，也不会出现"半开中断"的状态 */
  comm_disable_irqs();

  if (HAL_UART_Receive_DMA(&huart2, s_dma_buf, COMM_DMA_BUF_SIZE) != HAL_OK)
  {
    /* 最常见原因：MX 里没给 USART2_RX 配 DMA，或 DMA 时钟没使能 */
    Comm_LogLine("FATAL: USART2 RX DMA not ready (check MX DMA settings)");
    return;
  }

  s_dma_last_wpos = comm_dma_wpos();
}

void Comm_RxRestart(void)
{
  /* HAL_UART_AbortReceive 会停掉 DMA 并清掉错误标志（含 ORE），
     然后重新装载一次循环接收 —— 对应旧工程的 Remote_Restart()。 */
  (void)HAL_UART_AbortReceive(&huart2);

  if (HAL_UART_Receive_DMA(&huart2, s_dma_buf, COMM_DMA_BUF_SIZE) != HAL_OK)
  {
    Comm_LogLine("ERR: RX DMA restart failed");
    return;
  }

  s_dma_last_wpos = comm_dma_wpos();
  comm_disable_irqs();
}

void Comm_LogLine(const char *s)
{
  tx_str(s);
  tx_str("\r\n");
}

void Comm_LogInt(const char *tag, int32_t value)
{
  char     tmp[12];
  int      n = 0;
  uint32_t u;

  tx_str(tag);
  tx_str(" = ");
  if (value < 0)
  {
    tx_byte((uint8_t)'-');
    u = (uint32_t)(-value);
  }
  else
  {
    u = (uint32_t)value;
  }
  if (u == 0U)
  {
    tx_byte((uint8_t)'0');
  }
  while ((u != 0U) && (n < 11))
  {
    tmp[n] = (char)('0' + (u % 10U));
    n++;
    u /= 10U;
  }
  while (n > 0)
  {
    n--;
    tx_byte((uint8_t)tmp[n]);
  }
  tx_str("\r\n");
}

/* ========================== FreeRTOS 任务体 ========================== */

void StartCommTask(void const *argument)
{
  char       line[COMM_LINE_MAX];
  uint16_t   len        = 0U;
  TickType_t last_telem = 0U;

  (void)argument;

  last_telem = xTaskGetTickCount();

  for (;;)
  {
    uint16_t wpos;
    uint16_t pending;

    /* ---- 1. 把 DMA 循环缓冲里的新字节全部喂给行解析器 ---- */
    if ((huart2.hdmarx != NULL) && (huart2.hdmarx->Instance != NULL))
    {
      wpos    = comm_dma_wpos();
      /* 无符号回绕减法天然支持 DMA 指针绕圈 */
      pending = (uint16_t)((wpos - s_dma_last_wpos) & (COMM_DMA_BUF_SIZE - 1U));

      while (pending-- > 0U)
      {
        char c = (char)s_dma_buf[s_dma_last_wpos];
        s_dma_last_wpos = (uint16_t)((s_dma_last_wpos + 1U) & (COMM_DMA_BUF_SIZE - 1U));
        comm_feed_byte(c, line, &len);
      }
    }

    /* ---- 2. 遥测 ---- */
    if (s_telem_on != 0U)
    {
      TickType_t now = xTaskGetTickCount();
      if ((now - last_telem) >= pdMS_TO_TICKS(s_telem_period))
      {
        last_telem = now;
        comm_send_enc();
        comm_send_servo();
      }
    }

    /* ---- 3. 让出 CPU 到下一个轮询周期 ---- */
    osDelay(COMM_DMA_POLL_MS);
  }
}
