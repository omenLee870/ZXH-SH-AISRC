#ifndef __APP_FAULT_H__
#define __APP_FAULT_H__

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  发送固定故障码字符串。
 * @param  code 需要输出的固定码字符串，建议包含 "\r\n" 结尾。
 * @retval 无。
 * @note   该函数不使用 printf，不做格式化，只逐字节调用 HAL_UART_Transmit，
 *         用于 HardFault、FreeRTOS hook、APP_ErrorHandler 等异常路径。
 */
void App_FaultCodeSend(const char *code);

#ifdef __cplusplus
}
#endif

#endif /* __APP_FAULT_H__ */
