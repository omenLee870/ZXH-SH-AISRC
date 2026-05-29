/**
 * @file    retarget.c
 * @brief   标准 IO 到调试串口的重定向。
 * @details 将 printf 的输出流重定向到 USART1 调试串口。
 *
 * 调用链：
 *   printf("hello", ...)
 *     → C 库格式化字符串
 *       → Keil MicroLIB 调用 fputc()，GCC/newlib 调用 _write()
 *         → __io_putchar()
 *           → HAL_UART_Transmit(&DebugUartHandle, ...)
 *             → USART1 → PB6(TX) 引脚
 *
 * @note    Keil ARMCC5 + MicroLIB 的 printf 默认通过 fputc 输出字符，
 *          仅实现 _write 会导致 printf 没有被正确接到串口。
 */

#include "app_uart.h"
#include <stdio.h>

#define RETARGET_UART_TIMEOUT_MS        10U     /* 单字节串口发送超时，单位 ms，避免日志异常时长时间阻塞主循环。 */

#if DBG_ENABLE

#if defined(__CC_ARM)
#pragma import(__use_no_semihosting)

struct __FILE
{
    int handle;                                  /* Keil C 库文件句柄占位，裸机工程不使用实际文件系统。 */
};

FILE __stdout;                                  /* Keil printf 输出流对象，fputc 会把该流写到 USART1。 */
#endif

/**
 * @brief  发送单个字符到调试串口
 * @param  ch  要发送的字符（ASCII 码）
 * @return 返回原始字符，符合 printf 底层字符输出接口约定。
 * @note   所有标准输出重定向最终都收敛到这里，便于统一控制超时时间和串口句柄。
 */
int __io_putchar(int ch)
{
    uint8_t txData = (uint8_t)ch;                /* HAL_UART_Transmit 需要 uint8_t 缓冲区地址。 */

    (void)HAL_UART_Transmit(&DebugUartHandle, &txData, 1, RETARGET_UART_TIMEOUT_MS);
    return ch;
}

/**
 * @brief  Keil ARMCC5/MicroLIB 的 printf 字符输出入口。
 * @param  ch C 库格式化后输出的单个字符。
 * @param  f  输出流对象，裸机工程中不区分 stdout/stderr。
 * @return 返回原始字符，表示该字符已交给底层输出函数处理。
 * @note   本工程使用 Keil MicroLIB，printf("A\r\n") 会优先走该函数，而不是 newlib 的 _write。
 */
int fputc(int ch, FILE *f)
{
    (void)f;                                     /* 裸机串口输出不使用文件流对象。 */
    return __io_putchar(ch);
}

/**
 * @brief  newlib 底层写接口（覆盖弱符号）
 * @param  file  文件描述符（裸机忽略）
 * @param  ptr   数据缓冲区指针
 * @param  len   要写入的字节数
 * @return 实际写入的字节数
 */
int _write(int file, char *ptr, int len)
{
    int i;

    (void)file;                                  /* 裸机不使用文件描述符。 */

    for (i = 0; i < len; i++)
    {
        __io_putchar(*ptr++);                    /* 逐字节发到 USART1。 */
    }

    return len;
}

#if defined(__CC_ARM)
/**
 * @brief  关闭半主机退出路径。
 * @param  x C 库传入的退出码，裸机工程不使用。
 * @retval 无。
 * @note   如果 C 库异常进入 exit 路径，这里用死循环留在现场，避免触发半主机 BKPT 后跑飞。
 */
void _sys_exit(int x)
{
    (void)x;

    while (1)
    {
    }
}
#endif

#endif /* DBG_ENABLE */
