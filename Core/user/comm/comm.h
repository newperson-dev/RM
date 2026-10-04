/**
  ******************************************************************************
  * @file    comm.h
  * @brief   USART2 指令接收与调试回传（目录：Core/user/comm/）
  *
  * 收数据方案：**DMA 循环缓冲 + 轮询（零中断）**，吸收了旧工程 ex/user/remotor 的思路：
  *   - USART2_RX 由 DMA1 Channel6 在 Circular 模式下持续搬运到 512B 环形缓冲，
  *     全程不产生任何接收中断（USART2 与 DMA1_Channel6 的 NVIC 都被主动关闭）；
  *   - CommTask 每 5ms 读一次 DMA 的 CNDTR 寄存器推算写指针，把新字节喂进行解析器。
  *   - 好处：115200 波特下省掉约 11500 次中断/秒；抗突发能力强（512B 缓冲）。
  *   - 代价：命令响应延迟 ≤5ms（COMM_DMA_POLL_MS 可调），人敲命令完全无感。
  *   - 注意：千万不要在 DMA 模式下打开 USART2 中断。HAL 的错误中断处理会执行
  *     UART_EndRxTransfer() 关掉接收 DMA，而且它"读 SR + 读 DR"的清标志动作
  *     会从 DMA 手里抢走一个字节。详见 docs/工程规范与调试手册.md。
  *
  * 命令表（ASCII，行尾 \r\n 或 \n，大写）：
  *   HELP                        打印命令表
  *   INFO                        打印计数每圈 / 各电机模式 / 接收缓冲与轮询周期
  *   RXRST                       重新装载接收 DMA（异常恢复）
  *   STOP                        全部电机停机
  *   M    <1-4> <pwm>            开环 PWM，-3599..+3599（满量程 = TIM1 的 ARR）
  *   SPD  <1-4> <counts/s>       速度环：目标速度（编码器计数/秒，满量程 8580）
  *   SPDR <1-4> <rpm>            速度环：目标转速（RPM，0~330 对应空载）
  *   PID  <1-4> <kp> <ki> <kd>   速度环增益 ×1000（PID 1 1200 30 0 -> kp=1.2 ki=0.03）
  *   PIDEN<1-4> <0|1>            速度环开关
  *   POS  <1-4> <deg>            位置环：目标角度（会自动打开位置环，需先 ZERO）
  *   POSEN<1-4> <0|1>            位置环开关
  *   PIDP <1-4> <kp> <ki> <kd>   位置环增益 ×1000
  *   ZERO <1-4> | ZERO           编码器计数清零（位置环用之前必须清零点）
  *   S    <1-4> <deg>            舵机直接到角度     SS <1-4> <deg> 舵机斜坡到角度
  *   SP   <1-4> <us>             舵机直接到脉宽（500-2500）
  *   SLEW <1-4> <us/s>           舵机斜坡速度
  *   RELAX | ARMED               舵机卸力 / 恢复输出
  *   TELEM <0|1> [period_ms]     遥测开关（默认 100ms 一帧）
  *   ENC                         立即打印一次计数/速度/RPM/角度/PWM
  ******************************************************************************
  */
#ifndef __COMM_H
#define __COMM_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "main.h"

/** DMA 循环缓冲大小（字节，必须是 2 的幂，便于用掩码处理回绕） */
#define COMM_DMA_BUF_SIZE    512U

/** 轮询周期（ms）：越小越及时，越大越省 CPU。
 *  5ms 时 115200 波特下每轮最多约 58 字节，512B 缓冲足够。 */
#define COMM_DMA_POLL_MS     5U

/** 单行命令最大长度（含结尾 0） */
#define COMM_LINE_MAX        80U

/** 遥测默认周期（ms），需 >= COMM_DMA_POLL_MS */
#define COMM_TELEM_PERIOD_MS 100U

/** @brief 初始化接收 DMA 并关闭相关 NVIC（纯轮询、零中断）。调度器启动前调用。 */
void Comm_Init(void);

/** @brief 发送一行字符串（自动补 \r\n）。任务上下文调用，寄存器级发送，不占用 HAL 锁。 */
void Comm_LogLine(const char *s);

/** @brief 发送 "标签 = 整数" 一行（调试用）。 */
void Comm_LogInt(const char *tag, int32_t value);

/** @brief 重新装载接收 DMA（异常恢复用）。
 *  典型用途：串口长期收不到数据时发一条 RXRST 命令，或上层定时自愈。 */
void Comm_RxRestart(void);

/** @brief FreeRTOS 任务体：定义强符号，任务本身在 freertos.c 里创建。 */
void StartCommTask(void const *argument);

#ifdef __cplusplus
}
#endif

#endif /* __COMM_H */
