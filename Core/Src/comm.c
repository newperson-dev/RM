/**
  ******************************************************************************
  * @file    comm.c
  * @brief   USART2 指令接收与调试回传实现（用户代码）
  *
  * 关键设计：
  *   1) 中断里只做"把字节塞进 StreamBuffer"，不解析、不打印；
  *   2) CommTask 里做状态机组帧 + 命令分发，解析结果直接调 Motor_/Servo_ 接口；
  *   3) 发送走寄存器级（不用 HAL_UART_Transmit），避免 HAL 的 __HAL_LOCK
  *      与中断里的 HAL_UART_Receive_IT 抢锁，导致收数悄悄停掉；
  *   4) 只用整数解析（自己写 parse_milli），不引入 newlib 的 %f 扫描，
  *      CommTask 的栈可以开得很小。
  ******************************************************************************
  */
#include "comm.h"
#include "usart.h"
#include "motor.h"
#include "servo_arm.h"

#include "FreeRTOS.h"
#include "task.h"
#include "stream_buffer.h"
#include "cmsis_os.h"

#include <string.h>
#include <stdlib.h>

/* ========================== 内部状态 ========================== */

static StreamBufferHandle_t s_rx_sb   = NULL;
static uint8_t              s_rx_byte = 0U;

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

/* ========================== 遥测 ========================== */

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

/* ========================== 命令表 ========================== */

static void comm_print_help(void)
{
  tx_str("== RM arm/comm commands ==\r\n");
  tx_str("HELP                       this\r\n");
  tx_str("STOP                       stop all motors\r\n");
  tx_str("M   <1-4> <pwm>            open-loop pwm (-ARR..+ARR)\r\n");
  tx_str("SPD <1-4> <counts/s>       closed-loop target speed\r\n");
  tx_str("PID <1-4> <kp> <ki> <kd>   gains x1000 (PID 1 1200 30 0)\r\n");
  tx_str("PIDEN <1-4> <0|1>          closed-loop enable\r\n");
  tx_str("S   <1-4> <deg>            servo jump to angle\r\n");
  tx_str("SS  <1-4> <deg>            servo slew to angle\r\n");
  tx_str("SP  <1-4> <us>             servo jump to pulse (500-2500)\r\n");
  tx_str("SLEW <1-4> <us/s>          servo slew rate\r\n");
  tx_str("RELAX | ARMED              servo output off / on\r\n");
  tx_str("TELEM <0|1> [period_ms]    telemetry\r\n");
  tx_str("ENC                        print encoder + speed once\r\n");
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

  if (strcmp(cmd, "HELP") == 0)
  {
    comm_print_help();
  }
  else if (strcmp(cmd, "STOP") == 0)
  {
    Motor_StopAll();
    tx_str("OK STOP\r\n");
  }
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
  else if ((strcmp(cmd, "S") == 0) || (strcmp(cmd, "SS") == 0))
  {
    char   *t;
    uint8_t ok;
    int32_t milli;
    uint8_t slew = (cmd[1] == 'S') ? 1U : 0U;

    idx = arg_index(&save, SERVO_NUM);
    t   = arg_next(&save);
    if ((idx == 0xFFU) || (t == NULL))
    {
      tx_str("ERR usage: S/SS <1-4> <deg>\r\n");
      return;
    }
    milli = parse_milli(t, &ok);
    if (ok == 0U)
    {
      tx_str("ERR bad angle\r\n");
      return;
    }
    if (slew != 0U)
    {
      Servo_SetTargetAngle(idx, (float)milli / 1000.0f);
    }
    else
    {
      Servo_SetAngle(idx, (float)milli / 1000.0f);
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
      if (period < 10)
      {
        period = 10;
      }
      s_telem_period = (uint32_t)period;
    }
    tx_str(s_telem_on != 0U ? "OK TELEM ON\r\n" : "OK TELEM OFF\r\n");
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

/* ========================== 对外接口 ========================== */

void Comm_Init(void)
{
  s_rx_sb = xStreamBufferCreate(COMM_RX_SB_SIZE, 1U);

  /* CubeMX 默认给 USART2 的中断优先级是 0，优先级 0 的中断里不能调 FreeRTOS API，
     这里统一改成 5（= configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY）。 */
  HAL_NVIC_SetPriority(USART2_IRQn, COMM_IRQ_PRIORITY, 0U);
  HAL_NVIC_EnableIRQ(USART2_IRQn);

  (void)HAL_UART_Receive_IT(&huart2, &s_rx_byte, 1U);
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

/* ========================== 中断回调 ========================== */

/* 覆盖 HAL 的 __weak 回调。收发都只在这一个地方接触 USART2 的接收路径。 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    BaseType_t woken = pdFALSE;

    if (s_rx_sb != NULL)
    {
      (void)xStreamBufferSendFromISR(s_rx_sb, &s_rx_byte, 1U, &woken);
    }

    /* 调度器还没启动时不能触发 PendSV（pxCurrentTCB 还没建立） */
    if ((woken != pdFALSE) &&
        (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED))
    {
      portYIELD_FROM_ISR(woken);
    }

    (void)HAL_UART_Receive_IT(&huart2, &s_rx_byte, 1U);
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    __HAL_UART_CLEAR_OREFLAG(huart);
    __HAL_UART_CLEAR_PEFLAG(huart);
    __HAL_UART_CLEAR_FEFLAG(huart);
    huart->ErrorCode = HAL_UART_ERROR_NONE;
    (void)HAL_UART_Receive_IT(&huart2, &s_rx_byte, 1U);
  }
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
    uint8_t buf[32];
    size_t  n;
    size_t  i;

    n = xStreamBufferReceive(s_rx_sb, buf, sizeof(buf), pdMS_TO_TICKS(10));

    for (i = 0U; i < n; i++)
    {
      char c = (char)buf[i];

      if (c == '\r')
      {
        continue;
      }
      if (c == '\n')
      {
        if (len > 0U)
        {
          line[len] = '\0';
          comm_handle_line(line);
          len = 0U;
        }
      }
      else if ((c == '\b') || (c == 0x7F))
      {
        if (len > 0U)
        {
          len--;
        }
      }
      else if (len < (uint16_t)(COMM_LINE_MAX - 1U))
      {
        line[len] = c;
        len++;
      }
      else
      {
        len = 0U;
        Comm_LogLine("ERR line too long");
      }
    }

    /* 遥测 */
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
  }
}
