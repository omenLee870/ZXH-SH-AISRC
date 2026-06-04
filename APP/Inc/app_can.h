/**
 * @file    app_can.h
 * @brief   CAN 总线驱动层 —— 硬件抽象
 * @details 提供 CAN1 外设的初始化、收发、中断处理，与上层协议解耦。
 *
 *          硬件配置：
 *          - CAN1 外设
 *          - PA11 = CAN_RX (AF4)
 *          - PA12 = CAN_TX (AF4), 外接 TJA1050 收发器 → CANH / CANL
 *          - 时钟源：PLL = HSI 24MHz × 3 = 72MHz
 *          - 波特率：250Kbps (Prescaler=12, Seg1=17, Seg2=6, SJW=4)
 *
 *          参考：官方 CAN_ExtendedID_IT 例程
 *          E:\...\资料\PY32F072E_Firmware_V1.1.1\Projects\PY32F072E-STK\Example\CAN\CAN_ExtendedID_IT
 */

#ifndef __APP_CAN_H__
#define __APP_CAN_H__

#include "py32f0xx_hal.h"
#include <stdint.h>

/* ================================================================== */
/* CAN 总线硬件参数                                                     */
/*                                                                     */
/* 时钟树：                                                             */
/*   HSI 24MHz ─┬→ SYSCLK (24MHz, FLASH_LATENCY_0)                       */
/*             └→ PLL ×1(/1) ×3 = 72MHz → CAN 时钟                     */
/*                                                                     */
/*   官方例程使用 HSE 24MHz，本项目按硬件实际保留 PLL 72MHz。              */
/*   下面这组参数保持 250Kbps，并与此前 J-Link 看到的寄存器值一致：        */
/*   波特率 = 72MHz / 12 / (1 + 17 + 6) = 250Kbps                       */
/* ================================================================== */
#define CAN_PRESCALER  12U
#define CAN_SJW        4U
#define CAN_SEG1       17U
#define CAN_SEG2       6U

/* ================================================================== */
/* API 声明                                                            */
/* ================================================================== */

/**
 * @brief  初始化 CAN1 总线
 * @note   内部流程：
 *         1. HAL_CAN_Init(&CanHandle) → 触发 HAL_CAN_MspInit() 回调
 *            → 配置 CAN 时钟源(PLL)、PA11/PA12 引脚(AF4)、NVIC 中断
 *         2. 配置接收滤波器（扩展帧 ID）
 *         3. HAL_CAN_Start() 启动总线参与通信
 *         4. __HAL_CAN_ENABLE_IT() 使能接收完成与 PTB 发送完成中断
 */
void APP_CAN_Init(void);

/**
 * @brief  发送一条 CAN 扩展帧
 * @param  id   29-bit 扩展帧 ID (0x00000000 ~ 0x1FFFFFFF)
 * @param  data 数据缓冲区指针（至少 len 字节有效）
 * @param  len  数据长度 (1~8，超过自动截断为 8)
 * @return 0 = 成功, -1 = 发送失败（PTB FIFO 满或硬件错误）
 *
 * @note   发送流程（按官方例程）：
 *         1. HAL_CAN_AddMessageToTxFifo() → 写入 PTB（主发送缓冲）
 *         2. HAL_CAN_ActivateTxRequest()  → 激活发送请求
 */
int APP_CAN_Send(uint32_t id, uint8_t *data, uint8_t len);

/**
 * @brief  获取 CAN 全局句柄指针
 * @return 指向全局 CanHandle 的指针
 * @note   供 CAN_IRQHandler() 使用（app_can.c 内部自用，外部不需要调）
 */
CAN_HandleTypeDef *APP_CAN_GetHandle(void);

#endif /* __APP_CAN_H__ */
