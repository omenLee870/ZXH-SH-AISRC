/**
 * @file    app_uart.h
 * @brief   调试串口模块
 * @details 初始化 USART2 (PA2=TX, PA3=RX, 115200-8-N-1)
 *          导出 DebugUartHandle 供 retarget.c 中的 _write 使用
 */

#ifndef __APP_UART_H__
#define __APP_UART_H__

#include "py32f0xx_hal.h"
#include "app_debug.h" 

/**
 * @brief 调试串口句柄
 * @note  外部模块（retarget.c）通过此句柄调用 HAL_UART_Transmit
 */
extern UART_HandleTypeDef DebugUartHandle;

/**
 * @brief  初始化调试串口（受 DBG_ENABLE 宏控制）
 * @note   使用 USART1, PB6(TX, AF0), PB7(RX, AF0), 115200-8N1
 *         DBG_ENABLE=0 时此函数为空，编译器会优化掉
 */
#if DBG_ENABLE
void App_UART_Init(void);
#else
#define App_UART_Init()  ((void)0)
#endif

#endif /* __APP_UART_H__ */
