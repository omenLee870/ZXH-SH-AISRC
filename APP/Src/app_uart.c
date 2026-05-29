/**
 * @file    app_uart.c
 * @brief   调试串口 USART2 初始化实现
 * @details 硬件配置：
 *          - USART2, 115200 bps, 8-N-1, 无流控
 *          - PA2 = TX (AF1)
 *          - PA3 = RX (AF1)
 * @note    与 PY32F072Exx Start Kit BSP 一致
 */

#include "app_uart.h"
#include "app_debug.h"

#if DBG_ENABLE

UART_HandleTypeDef DebugUartHandle;

/**
 * @brief  初始化调试串口 USART1
 * @note   步骤：
 *         1. 使能 USART1 时钟
 *         2. 配置 PB6/PB7 为复用推挽 (AF0 = USART1)
 *         3. 配置 USART1 参数 (115200-8-N-1)
 *         4. 关闭 stdout 行缓冲
 */
void App_UART_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /******************** 使能外设时钟 ********************/
    __HAL_RCC_USART1_CLK_ENABLE();

    /******************** 配置 USART1 TX: PB6 ********************/
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;         /**< 复用推挽输出   */
    GPIO_InitStruct.Pull      = GPIO_PULLUP;              /**< 上拉（空闲高） */
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;     /**< 高速驱动       */

    GPIO_InitStruct.Pin       = GPIO_PIN_6;               /**< PB6            */
    GPIO_InitStruct.Alternate = GPIO_AF0_USART1;          /**< AF0 → USART1   */
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /******************** 配置 USART1 RX: PB7 ********************/
    GPIO_InitStruct.Pin       = GPIO_PIN_7;               /**< PB7            */
    GPIO_InitStruct.Alternate = GPIO_AF0_USART1;          /**< AF0 → USART1   */
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /******************** 配置 USART1 参数 ********************/
    DebugUartHandle.Instance          = USART1;
    DebugUartHandle.Init.BaudRate     = 115200;            /**< 波特率         */
    DebugUartHandle.Init.WordLength   = UART_WORDLENGTH_8B;/**< 8 位数据       */
    DebugUartHandle.Init.StopBits     = UART_STOPBITS_1;   /**< 1 位停止       */
    DebugUartHandle.Init.Parity       = UART_PARITY_NONE;  /**< 无校验         */
    DebugUartHandle.Init.HwFlowCtl    = UART_HWCONTROL_NONE;/**< 无硬件流控    */
    DebugUartHandle.Init.Mode         = UART_MODE_TX_RX;   /**< 全双工         */
    DebugUartHandle.Init.OverSampling = UART_OVERSAMPLING_16;/**< 16 倍过采样  */
    HAL_UART_Init(&DebugUartHandle);

    /******************** 关闭 stdout 行缓冲 ********************/
    /** @note  默认 stdio 是行缓冲，即遇到 \n 才真正输出。
     *         _IONBF = 无缓冲，每字节立即发送，调试时避免丢失延迟信息。
     */
    setvbuf(stdout, NULL, _IONBF, 0);
}

#endif /* DBG_ENABLE */
