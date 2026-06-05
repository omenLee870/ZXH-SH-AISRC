/**
 * @file    app_can.c
 * @brief   CAN 总线驱动层实现
 * @details 本文件汇总 CAN 硬件相关的所有底层代码：
 *
 *          ┌─ HAL_CAN_MspInit()  ← 由 HAL_CAN_Init 自动回调
 *          │   配置 CAN 时钟源(PLL)、PA11/PA12 引脚(AF4)、NVIC 中断
 *          │
 *          ├─ CAN_IRQHandler()   ← 覆盖启动文件的 Default_Handler
 *          │   CAN 中断入口 → 调 HAL_CAN_IRQHandler(&CanHandle)
 *          │
 *          ├─ HAL_CAN_RxCpltCallback()  ← 由 HAL_CAN_IRQHandler 回调
 *          │   读取接收数据
 *          │
 *          ├─ APP_CAN_Init()     ← 上层调用，完成寄存器/滤波器/启动
 *          │
 *          └─ APP_CAN_Send()     ← 通用发送接口（扩展帧）
 *
 *          与上层协议解耦：不包含任何雷迈协议相关常量或逻辑。
 *          协议相关代码见 app_can_proto.h / app_can_proto.c
 *
 *          参考：
 *          - 官方 CAN_ExtendedID_IT 例程
 *          - voice_uart.c（本文件中 ISR 和回调的放置方式与之一致）
 */

#include "app_can.h"
#include "app_debug.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

/* ================================================================== */
/* 全局 CAN 句柄                                                       */
/* ================================================================== */
CAN_HandleTypeDef CanHandle;

#define CAN_RX_QUEUE_LENGTH     16          /**< 接收队列容量（帧数）     */
static QueueHandle_t s_canRxQueue = NULL;   /**< CAN 接收队列句柄         */

/* ================================================================== */
/* HAL_CAN_MspInit() — 硬件资源初始化                                   */
/* ================================================================== */

/**
 * @brief  CAN 硬件资源初始化（覆盖 HAL 库 __weak 弱符号）
 * @param  hcan CAN 句柄指针
 *
 * @note   本函数由 HAL_CAN_Init() 在 CAN 控制器复位之前调用。
 *         负责：
 *         1. 选择 PLL 作为 CAN 时钟源（72MHz）
 *         2. 使能 CAN1 + GPIOA 外设时钟
 *         3. 配置 PA11=CAN_RX (AF4), PA12=CAN_TX (AF4)
 *         4. 配置 NVIC：CAN 中断优先级 = 1, 子优先级 = 0
 */
void HAL_CAN_MspInit(CAN_HandleTypeDef *hcan)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /******************** CAN 时钟源 = PLL (72MHz) ********************/
    /**
     * 选择 PLL 输出作为 CAN 外设时钟。
     * PLL 在 main.c 的 APP_SystemClockConfig() 中已配置为 HSI×3=72MHz。
     * 这里与官方例程的差异只保留在时钟源：官方使用 HSE，本项目硬件使用 PLL。
     * 等待 PLL 就绪标志位（PLLRDY = 1）后再继续
     */
    __HAL_RCC_CAN_CONFIG(RCC_CANCLKSOURCE_PLL);
    while (__HAL_RCC_GET_FLAG(RCC_FLAG_PLLRDY) == RESET) {}

    /******************** 使能外设时钟 ********************/
    __HAL_RCC_CAN1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /******************** 配置 CAN 引脚 ********************/
    /**
     * PA11 = CAN_RX → 复用输入（AF4）
     * PA12 = CAN_TX → 复用推挽输出（AF4）
     *
     * 外部通过 TJA1050 转换为差分 CANH/CANL
     *
     * 引脚配置参数说明：
     *   - Mode  = AF_PP        : 复用功能推挽输出
     *   - Pull  = NOPULL       : TJA1050 内部已做偏置，MCU 侧不需要上下拉
     *   - Speed = VERY_HIGH    : CAN 250Kbps 信号边沿较陡，需要高速驱动能力
     */
    GPIO_InitStruct.Pin       = GPIO_PIN_11 | GPIO_PIN_12;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_CAN;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /******************** NVIC 中断配置 ********************/
    /**
     * 抢占优先级 = 1, 子优先级 = 0
     * 低于 SysTick(0) 和 HardFault(-1)，高于 FreeRTOS 管理中断(>5)
     * 确保实时接收 CAN 帧不丢失，同时不妨碍系统滴答
     */
    HAL_NVIC_SetPriority(CAN_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(CAN_IRQn);

    /* 创建 CAN 接收队列 */
    s_canRxQueue = xQueueCreate(CAN_RX_QUEUE_LENGTH, sizeof(CAN_RxFrame_t));
    if (s_canRxQueue == NULL)
    {
        LOG_ERR("CAN RX queue create failed");
        return;
    }
    LOG_DBG("CAN RX queue created, capacity=%d", CAN_RX_QUEUE_LENGTH);
}

/* ================================================================== */
/* APP_CAN_Init() — 上层初始化入口                                      */
/* ================================================================== */

/**
 * @brief  初始化 CAN1 总线（上层调用入口）
 *
 * @note   完整初始化流程（按官方例程顺序）：
 *         1. 填充 Init 结构体 → 设置位时序参数
 *         2. HAL_CAN_Init() → 内部触发 HAL_CAN_MspInit() → 复位控制器 → 写寄存器
 *         3. 配置接收滤波器 → 决定哪些帧进入 RX FIFO
 *         4. HAL_CAN_Start() → 启动 CAN 总线参与通信
 *         5. __HAL_CAN_ENABLE_IT() → 使能接收完成与 PTB 发送完成中断
 */
void APP_CAN_Init(void)
{
    CAN_FilterTypeDef CanFilter = {0};

    /******************** 1. 填充 CAN 初始化参数 ********************/
    /**
     * 位时序计算：
     *   CAN 时钟 = PLL = 72MHz
     *   tq（时间份额） = Prescaler / CAN_CLK = 12 / 72MHz = 166.7ns
     *   1 bit 时长 = tq × (1 + Seg1 + Seg2) = 166.7ns × 24 = 4μs
     *   波特率 = 1 / 4μs = 250Kbps
     *
     *   FrameFormat/Mode/过滤器/收发流程参考官方 CAN_ExtendedID_IT 例程；
     *   时钟源和位时序按本项目 PLL 72MHz 换算，保证目标仍为 250Kbps。
     *   FrameFormat = CAN_FRAME_CLASSIC: 经典 CAN（非 CAN-FD）
     *   Mode = CAN_MODE_NORMAL: 正常通信模式，报文经 PA12/TJA1050 发到外部 CAN 总线，
     *                           外部总线接收的合法报文经 PA11 回到 CAN 控制器。
     */
    CanHandle.Instance                    = CAN1;
    CanHandle.Init.FrameFormat            = CAN_FRAME_CLASSIC;
    CanHandle.Init.Mode                   = CAN_MODE_NORMAL;
    CanHandle.Init.Prescaler              = CAN_PRESCALER;
    CanHandle.Init.NominalSyncJumpWidth   = CAN_SJW;
    CanHandle.Init.NominalTimeSeg1        = CAN_SEG1;
    CanHandle.Init.NominalTimeSeg2        = CAN_SEG2;

    if (HAL_CAN_Init(&CanHandle) != HAL_OK)
    {
        LOG_ERR("CAN init failed");
        return;
    }

    /******************** 2. 配置接收滤波器 ********************/
    /**
     * 滤波器配置（匹配语音“打开大灯/远光”使用的扩展帧 ID）：
     *   - IdType = CAN_EXTENDED_ID      : 按扩展帧（29-bit ID）过滤
     *   - FilterChannel = CHANNEL_0     : 使用滤波器通道 0
     *   - FilterID = 0x18FF1D18         : 当前发送的 IVI->BCM 报文 ID
     *   - MaskID = 0x0                  : 该 HAL 中 0 表示对应 ID 位参与比较
     *   - FilterFormat/MaskFormat       : 全 F 表示接受所有 LLC 格式
     */
    CanFilter.IdType         = CAN_EXTENDED_ID;
    CanFilter.FilterChannel  = CAN_FILTER_CHANNEL_0;
    CanFilter.Rank           = CAN_FILTER_RANK_CHANNEL_NUMBER;
    CanFilter.FilterID       = 0x18FF1D18;
    CanFilter.FilterFormat   = 0xFFFFFFFF;
    CanFilter.MaskID         = 0x0;
    CanFilter.MaskFormat     = 0xFFFFFFFF;

    if (HAL_CAN_ConfigFilter(&CanHandle, &CanFilter) != HAL_OK)
    {
        LOG_ERR("CAN filter config failed");
        return;
    }

    /******************** 3. 启动 CAN 总线 ********************/
    if (HAL_CAN_Start(&CanHandle) != HAL_OK)
    {
        LOG_ERR("CAN start failed");
        return;
    }

    /******************** 4. 使能中断 ********************/
    __HAL_CAN_ENABLE_IT(&CanHandle, (CAN_IT_RX_COMPLETE | CAN_IT_TX_PTB_COMPLETE));

    LOG_INFO("CAN1 init OK, 250Kbps, extended ID, normal mode, PA11=RX/PA12=TX (TJA1050)");
}

/* ================================================================== */
/* APP_CAN_Send() — 通用发送接口                                       */
/* ================================================================== */

/**
 * @brief  发送一条 CAN 扩展帧
 *
 * @note   发送流程（按官方例程，两步必须都执行）：
 *         1. HAL_CAN_AddMessageToTxFifo(..., CAN_TX_FIFO_PTB)
 *            → 将帧数据写入 PTB（Primary Transmit Buffer, 主发送缓冲）
 *         2. HAL_CAN_ActivateTxRequest(..., CAN_TXFIFO_PTB_SEND)
 *            → 激活 PTB 发送请求，硬件自动仲裁并发出
 *
 *         PTB 只能存一帧，发完才能写下一帧。
 *         如果 PTB 还挂着上一帧未发完，AddMessageToTxFifo 会返回 HAL_ERROR。
 *
 *         len 超过 8 自动截断（CAN 经典帧最大 DLC = 8）
 */
int APP_CAN_Send(uint32_t id, uint8_t *data, uint8_t len)
{
    CAN_TxHeaderTypeDef CanTxHeader = {0};
    uint8_t dlc = (len > 8) ? 8 : len;

    /******************** 填充发送头 ********************/
    CanTxHeader.Identifier   = id;
    CanTxHeader.IdType       = CAN_EXTENDED_ID;
    CanTxHeader.TxFrameType  = CAN_DATA_FRAME;
    CanTxHeader.FrameFormat  = CAN_FRAME_CLASSIC;
    CanTxHeader.Handle       = 0x0;
    CanTxHeader.DataLength   = dlc;

    /******************** 写入 PTB ********************/
    if (HAL_CAN_AddMessageToTxFifo(&CanHandle, &CanTxHeader,
                                   data, CAN_TX_FIFO_PTB) != HAL_OK)
    {
        return -1;
    }

    /******************** 激活发送请求 ********************/
    if (HAL_CAN_ActivateTxRequest(&CanHandle, CAN_TXFIFO_PTB_SEND) != HAL_OK)
    {
        return -1;
    }

    return 0;
}

/**
 * @brief  获取 CAN 句柄指针
 */
CAN_HandleTypeDef *APP_CAN_GetHandle(void)
{
    return &CanHandle;
}

/* ================================================================== */
/* CAN 接收中断回调                                                    */
/* ================================================================== */

/**
 * @brief  CAN 接收完成回调（覆盖 HAL 库 __weak 弱符号）
 * @param  hcan CAN 句柄
 *
 * @note   中断调用链：
 *         CAN_IRQHandler()
 *           → HAL_CAN_IRQHandler(&CanHandle)
 *             → 判断中断源标志位 = CAN_IT_RX_COMPLETE
 *               → HAL_CAN_RxCpltCallback(hcan)  ← 到本函数
 *
 *          本函数在 **中断上下文** 中执行：
 *         - 不要调用 vTaskDelay / 等 FreeRTOS API（除非 FromISR 版本）
 *         - 当前仅打日志，后续可改为 FreeRTOS 队列投递给任务处理
 */
void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef CanRxHeader = {0};
    CAN_RxFrame_t frame = {0};
    BaseType_t pxHigherPriorityTaskWoken = pdFALSE;

    HAL_CAN_GetRxMessage(hcan, &CanRxHeader, frame.data);
    
    frame.id  = CanRxHeader.Identifier;
    frame.dlc = CanRxHeader.DataLength;

    /**
     * 从 ISR 投递到 FreeRTOS 队列，不在中断里处理业务逻辑。
     * 如果队列满则丢弃（CAN_RX_QUEUE_OVERFLOW 事件）。
     */
    xQueueSendFromISR(s_canRxQueue, &frame, &pxHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);
}

/* ================================================================== */
/* CAN 中断服务函数（覆盖启动文件 Default_Handler 弱符号）              */
/* ================================================================== */
/**
 * @brief  CAN 全局中断服务函数
 *
 * @note   本项目中 ISR 的放置约定（与 voice_uart.c 一致）：
 *         - voice_uart.c : USART2_IRQHandler, EXTI4_15_IRQHandler
 *         - app_can.c    : CAN_IRQHandler
 *         - py32f072e_it.c : SysTick_Handler, NMI_Handler 等系统 ISR
 *
 *         每个外设的 ISR 放在自己的 .c 文件里，不需要修改 py32f072e_it.c
 *         因为启动文件 startup_py32f072exx.s 中声明的都是 .weak 弱符号
 *         链接器会优先使用这里的强符号
 */
void CAN_IRQHandler(void)
{
    HAL_CAN_IRQHandler(&CanHandle);
}

/**
 * @brief  获取 CAN 接收队列句柄
 */
QueueHandle_t CAN_GetRxQueue(void)
{
    return s_canRxQueue;
}

/**
 * @brief  CAN 接收帧处理（从 CAN_RxTask 调用）
 * @param  frame 指向 CAN 接收帧的指针
 * @note
 */
void CAN_ProcessRxFrame(const CAN_RxFrame_t *frame)
{
    LOG_DBG("CAN RX id=0x%08lX len=%d %02X %02X %02X %02X %02X %02X %02X %02X",
            frame->id, frame->dlc,
            frame->data[0], frame->data[1], frame->data[2], frame->data[3],
            frame->data[4], frame->data[5], frame->data[6], frame->data[7]);
}
