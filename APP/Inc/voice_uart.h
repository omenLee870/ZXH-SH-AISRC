/**
 * @file    voice_uart.h
 * @brief   语音芯片 YT5116 UART 通信模块
 * @details 协议概要：
 *          - 波特率：9600 bps, 8-N-1
 *          - 硬件：USART4, PA0(TX, AF4), PA1(RX, AF4)
 *          - 帧格式：固定 8 字节
 *            [帧头 1B] [命令 1B] [数据 5B] [校验 1B]
 *            校验 = 前 7 字节异或结果
 *          - 语音 → MCU：帧头 0xA5（识别结果上报）
 *          - MCU → 语音：帧头 0x5A（应答/控制）
 *
 * 唤醒机制：
 *          - PA4 配置为输入，检测上升沿（低→高）
 *          - 检测到上升沿后，MCU 主动发送唤醒帧唤醒语音芯片
 */

#ifndef __VOICE_UART_H__
#define __VOICE_UART_H__

#include "py32f0xx_hal.h"
#include <stdint.h>
#include "FreeRTOS.h"
#include "queue.h"

/* ===== 协议常量 ===== */
#define VOICE_UART_BAUDRATE         9600                        /**< 波特率           */
#define VOICE_UART_INSTANCE         USART4                      /**< 外设实例         */
#define VOICE_UART_AF               GPIO_AF4_USART4             /**< 复用功能         */

#define VOICE_FRAME_LEN             8                           /**< 帧长（字节）     */
#define VOICE_FRAME_HEADER_RX       0xA5                        /**< 语音→MCU 帧头   */
#define VOICE_FRAME_HEADER_TX       0x5A                        /**< MCU→语音 帧头   */

/* 应答状态码 */
#define VOICE_RESP_OK               0x00                        /**< 执行成功         */
#define VOICE_RESP_FAIL             0x01                        /**< 执行失败         */
#define VOICE_RESP_SAFETY           0x02                        /**< 安全限制（禁操） */

/* 控制指令 */
#define VOICE_CMD_WAKEUP            0xF0                        /**< 唤醒语音芯片     */
#define VOICE_CMD_SLEEP             0xF1                        /**< 退出唤醒         */

/* ===== 数据类型 ===== */

/**
 * @brief 语音识别帧结构体
 */
typedef struct
{
    uint8_t header;                                             /**< 帧头：0xA5      */
    uint8_t cmd;                                                /**< 命令码           */
    uint8_t data[5];                                            /**< 数据域           */
    uint8_t checksum;                                           /**< 校验字节         */
} VoiceFrame_t;

/* ===== 函数声明 ===== */

/**
 * @brief  初始化语音芯片 UART 通信接口
 * @note   配置 USART4 (PA0=TX, PA1=RX, AF4) + 9600-8-N-1
 *         使能 UART 接收中断（空闲中断 + 逐字节接收）
 */
void Voice_UART_Init(void);

/**
 * @brief  初始化 PA4 唤醒检测引脚
 * @note   配置 PA4 为输入 + 下降沿中断（检测别人拉高 PA4）
 */
void Voice_WakePin_Init(void);

/**
 * @brief  发送帧到语音芯片
 * @param  cmd      命令码
 * @param  respCode 应答/状态码（默认 0x00）
 * @param  pData    数据域指针（5 字节，可为 NULL 表示全零）
 * @note   自动计算并填充校验字节
 */
void Voice_SendFrame(uint8_t cmd, uint8_t respCode, const uint8_t *pData);

/**
 * @brief  计算帧校验值
 * @param  pFrame  帧数据指针（8 字节）
 * @return 前 7 字节的异或结果
 */
uint8_t Voice_CalcChecksum(const uint8_t *pFrame);

/**
 * @brief  校验接收帧是否有效
 * @param  pFrame  帧数据指针（8 字节）
 * @return 1 = 校验通过, 0 = 校验失败
 */
uint8_t Voice_VerifyFrame(const uint8_t *pFrame);

/**
 * @brief  UART 接收中断回调（由 HAL_UART_RxCpltCallback 调用）
 * @note   在 py32f072e_it.c 中调用本函数
 *         逐字节接收，组满 8 字节后送入 FreeRTOS 队列
 */
void Voice_UART_RxCallback(uint8_t byte);

/**
 * @brief  获取语音接收队列句柄
 * @return 队列句柄，供语音任务阻塞等待
 */
QueueHandle_t Voice_GetRxQueue(void);

/**
 * @brief  检查并清除唤醒标志
 * @return 1 = 有唤醒信号待处理, 0 = 无
 * @note   由 FreeRTOS 语音任务轮询调用
 */
uint8_t Voice_WakePending(void);

#endif /* __VOICE_UART_H__ */
