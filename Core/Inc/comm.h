/**
  ******************************************************************************
  * @file    comm.h
  * @brief   USART2 指令接收（中断收 + CommTask 解析）+ 调试信息回传
  * @note    用户代码，不由 CubeMX 管理。
  *
  * 数据流：
  *   USART2_IRQHandler -> HAL_UART_RxCpltCallback -> StreamBuffer
  *                                               -> CommTask 解析
  *                                               -> Motor_/Servo_ 接口
  *
  * CubeMX 需要做的配置：
  *   - USART2 参数保持 115200 / 8N1（或按你的对端改）；
  *   - NVIC 里勾上 "USART2 global interrupt"（否则中断不触发）。
  *     中断优先级由 Comm_Init() 统一改写成 5（>= configMAX_SYSCALL_INTERRUPT_PRIORITY），
  *     因为 CubeMX 默认给的是 0，优先级 0 的中断里不能调用 FreeRTOS API。
  *
  * 串口命令（ASCII，行尾 \r\n 或 \n，大写）：
  *   HELP                     打印命令表
  *   STOP                     全部电机停机（PWM=0，退闭环）
  *   M    <1-4> <pwm>         开环 PWM，范围 -满量程..+满量程
  *   SPD  <1-4> <counts/s>    闭环目标速度（需先用 PID 设增益）
  *   PID  <1-4> <kp> <ki> <kd> 增益 x1000，例如 PID 1 1200 30 0 表示 kp=1.2
  *   PIDEN<1-4> <0|1>         闭环开关
  *   S    <1-4> <deg>         舵机直接到角度（无斜坡）
  *   SS   <1-4> <deg>         舵机斜坡到角度
  *   SP   <1-4> <us>          舵机直接到脉宽（500~2500）
  *   SLEW <1-4> <us/s>        舵机斜坡速度
  *   RELAX / ARMED            舵机卸力 / 恢复输出
  *   TELEM <0|1> [period_ms]  遥测开关，默认 100ms 一帧
  *   ENC                      立即打印一次编码器计数与速度
  ******************************************************************************
  */
#ifndef __COMM_H
#define __COMM_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "main.h"

/** 中断收缓冲区（字节数） */
#define COMM_RX_SB_SIZE      128U

/** USART2 中断优先级：必须 >= 5（configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY） */
#define COMM_IRQ_PRIORITY    5U

/** 单行命令最大长度（含结尾 0） */
#define COMM_LINE_MAX        80U

/** 遥测默认周期（ms） */
#define COMM_TELEM_PERIOD_MS 100U

/** @brief 创建接收缓冲、配置 NVIC、启动第一次接收。调度器启动前调用。 */
void Comm_Init(void);

/** @brief 发送一行字符串（自动补 \r\n）。任务上下文调用，寄存器级发送，不占用 HAL 锁。
 *  @note 115200 下一行 100 字节约 8.7ms 阻塞，别在高优先级任务里刷长行。 */
void Comm_LogLine(const char *s);

/** @brief 发送 "标签 = 整数" 一行（调试用）。 */
void Comm_LogInt(const char *tag, int32_t value);

/** @brief FreeRTOS 任务体：定义强符号，任务本身在 freertos.c 里创建。 */
void StartCommTask(void const *argument);

#ifdef __cplusplus
}
#endif

#endif /* __COMM_H */
