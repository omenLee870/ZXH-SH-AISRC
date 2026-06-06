/**
 * @file    app_iwdg.c
 * @brief   独立看门狗驱动实现
 * @details IWDG 由 LSI（约 40kHz）驱动，与系统时钟完全解耦。
 *          即使主晶振停振或 PLL 失锁，看门狗依然能复位 MCU。
 *
 *          预分频 = 64 → 40kHz / 64 = 625Hz = 1.6ms/tick
 *          重载值 = 625 → 625 × 1.6ms ≈ 1000ms = 1 秒超时
 */

#include "py32f0xx_hal.h"

/**
 * @brief  初始化独立看门狗
 * @note
 */
void APP_IWDG_Init(void)
{
    /* 使能 LSI（独立看门狗时钟源） */
    RCC->CSR |= RCC_CSR_LSION;
    while ((RCC->CSR & RCC_CSR_LSIRDY) == 0U)
    {
        /* 等待 LSI 就绪 */
    }

    /* 启动 IWDG*/
    IWDG->KR = 0xCCCCU;

    /* 解锁 IWDG 配置寄存器，允许写 PR/RLR。 */
    IWDG->KR = 0x5555U;

    /* 预分频 = 64。 */
    IWDG->PR = IWDG_PRESCALER_64;

    /* 重载值 = 625，约 1s 级别超时，具体时间受 LSI 实际频率影响。 */
    IWDG->RLR = 625U;

    /* 等待预分频和重载寄存器更新完成。 */
    while (IWDG->SR != 0U)
    {
    }

    /* 初始化完成后立即喂一次狗，避免启动阶段还没进入喂狗任务就复位。 */
    IWDG->KR = 0xAAAAU;
}

/**
 * @brief  喂狗（重置计数器）
 * @note
 */
void APP_IWDG_Feed(void)
{
    IWDG->KR = 0xAAAAU;
}
