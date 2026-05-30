/**
 * @file    voice_uart.c
 * @brief   语音芯片 YT5116 UART 通信实现
 * @details 硬件配置：
 *          - USART4, 9600 bps, 8-N-1, 无流控
 *          - PA0 = TX (AF4)
 *          - PA1 = RX (AF4)
 *          - PA4 = 唤醒检测（输入，中断模式）
 */

#include "voice_uart.h"
#include "app_debug.h"

/* ===== 模块级变量 ===== */
static UART_HandleTypeDef VoiceUartHandle;
static uint8_t  rxBuffer[VOICE_FRAME_LEN];                      /**< 接收缓冲区            */
static uint8_t  rxIndex = 0;                                    /**< 缓冲区写入位置        */
static QueueHandle_t voiceRxQueue = NULL;                       /**< FreeRTOS 接收消息队列  */

/* ===== UART 初始化 ===== */

/**
 * @brief  初始化语音芯片 UART 通信接口（USART4）
 */
void Voice_UART_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /******************** 使能外设时钟 ********************/
    __HAL_RCC_USART4_CLK_ENABLE();

    /******************** 配置 USART4 TX: PA0 ********************/
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;         /**< 复用推挽输出   */
    GPIO_InitStruct.Pull      = GPIO_PULLUP;              /**< 上拉（空闲高） */
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;     /**< 高速驱动       */

    GPIO_InitStruct.Pin       = GPIO_PIN_0;               /**< PA0            */
    GPIO_InitStruct.Alternate = GPIO_AF4_USART4;          /**< AF4 → USART4   */
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /******************** 配置 USART4 RX: PA1 ********************/
    GPIO_InitStruct.Pin       = GPIO_PIN_1;               /**< PA1            */
    GPIO_InitStruct.Alternate = GPIO_AF4_USART4;          /**< AF4 → USART4   */
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /******************** 配置 USART4 参数 ********************/
    VoiceUartHandle.Instance          = USART4;
    VoiceUartHandle.Init.BaudRate     = VOICE_UART_BAUDRATE;/**< 9600 bps       */
    VoiceUartHandle.Init.WordLength   = UART_WORDLENGTH_8B;/**< 8 位数据       */
    VoiceUartHandle.Init.StopBits     = UART_STOPBITS_1;   /**< 1 位停止       */
    VoiceUartHandle.Init.Parity       = UART_PARITY_NONE;  /**< 无校验         */
    VoiceUartHandle.Init.HwFlowCtl    = UART_HWCONTROL_NONE;/**< 无硬件流控    */
    VoiceUartHandle.Init.Mode         = UART_MODE_TX_RX;   /**< 全双工         */
    VoiceUartHandle.Init.OverSampling = UART_OVERSAMPLING_16;/**< 16 倍过采样  */
    HAL_UART_Init(&VoiceUartHandle);

    /******************** 创建接收消息队列 ********************/
    /** @note  队列长度 8，存放整帧 VoiceFrame_t，
     *         一个队列元素 = 8 字节，总共 64 字节开销
     */
    voiceRxQueue = xQueueCreate(8, sizeof(VoiceFrame_t));
    if (voiceRxQueue == NULL)
    {
        LOG_ERR("Voice UART queue create failed!");
        return;
    }

    /******************** 启动 UART 逐字节接收中断 ********************/
    /** @note  使能 NVIC 上的 USART4 中断，
     *         然后在 ISR 中每收到一字节就重新触发下一次接收
     */
    HAL_NVIC_SetPriority(USART3_4_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(USART3_4_IRQn);

    /** @note  HAL_UART_Receive_IT 启动首次单字节接收，
     *         收完触发 HAL_UART_RxCpltCallback
     */
    HAL_UART_Receive_IT(&VoiceUartHandle, rxBuffer + rxIndex, 1);

    LOG_INFO("Voice UART USART4 PA0(TX) PA1(RX) @ 9600bps ready");
}

/* ===== PA4 唤醒检测 ===== */
/**
 * @brief  初始化 PA4 唤醒检测引脚
 * @note   配置为输入 + 上升沿中断，检测外部唤醒信号
 */
void Voice_WakePin_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /** @note  PA4 时钟已在 main.c 中统一使能，此处不再重复开 GPIOA 时钟 */

    GPIO_InitStruct.Pin   = GPIO_PIN_4;               /**< PA4            */
    GPIO_InitStruct.Mode  = GPIO_MODE_IT_RISING;      /**< 上升沿中断     */
    GPIO_InitStruct.Pull  = GPIO_PULLDOWN;            /**< 下拉（空闲低） */
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /** @note  使能 EXTI4 中断（PA4 映射到 EXTI4_15_IRQn 中断线） */
    HAL_NVIC_SetPriority(EXTI4_15_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

    LOG_INFO("Voice wake pin PA4 (rising edge) configured");
}

/* ===== 帧收发 ===== */
/**
 * @brief  计算帧校验值
 * @details 对前 7 字节逐字节异或，结果即为第 8 字节（校验位）
 */
uint8_t Voice_CalcChecksum(const uint8_t *pFrame)
{
    uint8_t checksum = 0;
    uint8_t i;

    for (i = 0; i < (VOICE_FRAME_LEN - 1); i++)
    {
        checksum ^= pFrame[i];
    }
    return checksum;
}

/**
 * @brief  校验接收帧是否有效
 */
uint8_t Voice_VerifyFrame(const uint8_t *pFrame)
{
    uint8_t calc = Voice_CalcChecksum(pFrame);

    if (calc == pFrame[VOICE_FRAME_LEN - 1])
    {
        return 1;                                           /**< 校验通过       */
    }
    LOG_WARN("Voice frame checksum error: calc=0x%02X recv=0x%02X",
             calc, pFrame[VOICE_FRAME_LEN - 1]);
    return 0;                                               /**< 校验失败       */
}

/**
 * @brief  发送帧到语音芯片
 */
void Voice_SendFrame(uint8_t cmd, uint8_t respCode, const uint8_t *pData)
{
    uint8_t frame[VOICE_FRAME_LEN];
    uint8_t i;

    /******************** 组装帧 ********************/
    frame[0] = VOICE_FRAME_HEADER_TX;                       /**< 帧头 0x5A      */
    frame[1] = cmd;                                          /**< 命令码         */
    frame[2] = respCode;                                     /**< 应答/状态码    */

    if (pData != NULL)
    {
        for (i = 0; i < 5; i++)
        {
            frame[3 + i] = pData[i];                        /**< 数据域         */
        }
    }
    else
    {
        for (i = 3; i < 7; i++)
        {
            frame[i] = 0x00;                                 /**< 数据域填零     */
        }
    }

    frame[7] = Voice_CalcChecksum(frame);                    /**< 校验字节       */

    /******************** 发送 ********************/
    HAL_UART_Transmit(&VoiceUartHandle, frame, VOICE_FRAME_LEN, 100);

    LOG_DBG("Voice TX: %02X %02X %02X %02X %02X %02X %02X %02X",
            frame[0], frame[1], frame[2], frame[3],
            frame[4], frame[5], frame[6], frame[7]);
}

/* ===== PA4 唤醒标志 ===== */
static volatile uint8_t voice_wake_pending = 0;              /**< 中断通知任务用   */

/**
 * @brief  检查并清除唤醒标志
 * @return 1 = 有唤醒信号待处理, 0 = 无
 * @note   由 FreeRTOS 语音任务轮询调用
 */
uint8_t Voice_WakePending(void)
{
    if (voice_wake_pending)
    {
        voice_wake_pending = 0;
        return 1;
    }
    return 0;
}

/**
 * @brief  获取语音接收队列句柄
 * @return 队列句柄，供语音任务阻塞等待
 */
QueueHandle_t Voice_GetRxQueue(void)
{
    return voiceRxQueue;
}

/* ======================================================================== */
/*                    HAL 回调函数（覆盖 HAL 库弱符号）                      */
/* ======================================================================== */

/**
 * @brief  HAL UART 接收完成回调（覆盖 HAL 弱符号）
 * @param  huart  触发回调的 UART 句柄
 * @note   由 USART3_4_IRQHandler → HAL_UART_IRQHandler 中断链调用
 *         每收到一个字节触发一次，组满 8 字节后校验并入队
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    /******************** 仅处理语音 UART ********************/
    if (huart->Instance != USART4)
    {
        return;
    }

    /******************** 字节计数 +1 ********************/
    rxIndex++;

    /******************** 组满一帧？ ********************/
    if (rxIndex >= VOICE_FRAME_LEN)
    {
        /******************** 校验帧头 ********************/
        if (rxBuffer[0] == VOICE_FRAME_HEADER_RX)
        {
            /******************** 校验和 → 入队 ********************/
            if (Voice_VerifyFrame(rxBuffer))
            {
                VoiceFrame_t frame;
                frame.header   = rxBuffer[0];
                frame.cmd      = rxBuffer[1];
                frame.data[0]  = rxBuffer[2];
                frame.data[1]  = rxBuffer[3];
                frame.data[2]  = rxBuffer[4];
                frame.data[3]  = rxBuffer[5];
                frame.data[4]  = rxBuffer[6];
                frame.checksum = rxBuffer[7];

                LOG_DBG("Voice RX: %02X %02X %02X %02X %02X %02X %02X %02X",
                        rxBuffer[0], rxBuffer[1], rxBuffer[2], rxBuffer[3],
                        rxBuffer[4], rxBuffer[5], rxBuffer[6], rxBuffer[7]);

                BaseType_t xHigherPriorityTaskWoken = pdFALSE;
                if (xQueueSendFromISR(voiceRxQueue, &frame, &xHigherPriorityTaskWoken) != pdPASS)
                {
                    LOG_WARN("Voice queue full, frame dropped!");
                }
                portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
            }
        }
        else
        {
            LOG_WARN("Voice frame header error: 0x%02X", rxBuffer[0]);
        }

        /******************** 无论成败，从头接收下一帧 ********************/
        rxIndex = 0;
    }

    /******************** 重新触发下一字节接收 ********************/
    HAL_UART_Receive_IT(&VoiceUartHandle, rxBuffer + rxIndex, 1);
}

/**
 * @brief  HAL GPIO EXTI 中断回调（覆盖 HAL 弱符号）
 * @param  GPIO_Pin  触发中断的引脚号
 * @note   由 EXTI4_15_IRQHandler → HAL_GPIO_EXTI_IRQHandler 中断链调用
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    /******************** PA4 上升沿 → 置唤醒标志 ********************/
    if (GPIO_Pin == GPIO_PIN_4)
    {
        voice_wake_pending = 1;
        LOG_DBG("PA4 wake signal detected");
    }
}

/* ======================================================================== */
/*                    中断服务函数（替代 py32f072e_it.c 中的默认弱实现）     */
/* ======================================================================== */

/**
 * @brief  EXTI4_15 中断服务函数
 * @note   覆盖启动文件中的 WEAK 弱符号
 *         PA4 上升沿触发 → HAL_GPIO_EXTI_IRQHandler → HAL_GPIO_EXTI_Callback
 */
void EXTI4_15_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_4);
}

/**
 * @brief  USART3/4 中断服务函数
 * @note   覆盖启动文件中的 WEAK 弱符号
 *         USART4 收发中断 → HAL_UART_IRQHandler → HAL_UART_RxCpltCallback
 */
void USART3_4_IRQHandler(void)
{
    HAL_UART_IRQHandler(&VoiceUartHandle);
}
