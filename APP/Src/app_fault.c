#include "app_fault.h"
#include "app_uart.h"
#include "FreeRTOS.h"
#include "task.h"

#define APP_FAULT_UART_TIMEOUT_MS      10U    /* 单字节故障码发送超时时间，单位 ms。 */

/**
 * @brief  发送固定故障码字符串。
 * @param  code 需要输出的固定码字符串。
 * @retval 无。
 * @note   该函数故意不使用 LOG_ERR/printf，避免异常现场再次消耗较大栈空间。
 *         使用前提是调试串口已经完成 App_UART_Init() 初始化。
 */
void App_FaultCodeSend(const char *code)
{
    while ((code != NULL) && (*code != '\0'))
    {
        uint8_t txData = (uint8_t)(*code++);
        (void)HAL_UART_Transmit(&DebugUartHandle, &txData, 1, APP_FAULT_UART_TIMEOUT_MS);
    }
}

/**
 * @brief  FreeRTOS 动态内存申请失败 hook。
 * @retval 无。
 * @note   当 xTaskCreate、xQueueCreate 等接口从 FreeRTOS heap 申请内存失败时，
 *         内核会调用该函数。AAAAAAAA 表示 FreeRTOS heap 不足。
 */
void vApplicationMallocFailedHook(void)
{
    App_FaultCodeSend("AAAAAAAA\r\n");

    while (1)
    {
    }
}

/**
 * @brief  FreeRTOS 任务栈溢出 hook。
 * @param  xTask 发生栈溢出的任务句柄。
 * @param  pcTaskName 发生栈溢出的任务名。
 * @retval 无。
 * @note   BBBBBBBB 表示任务栈溢出。这里不打印任务名，避免异常现场访问更多数据。
 */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    (void)xTask;
    (void)pcTaskName;

    App_FaultCodeSend("BBBBBBBB\r\n");

    while (1)
    {
    }
}
