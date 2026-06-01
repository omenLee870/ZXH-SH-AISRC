/**
 * @file    app_voice.h
 * @brief   语音业务模块
 * @details 职责：
 *          - 初始化语音 UART + PA4 唤醒检测
 *          - 命令码分发与结果映射
 *          - 通过 Voice_SendFrame() 回复语音芯片
 * @note    voice_uart 负责串口收发和帧校验，app_voice 负责业务逻辑。
 */

#ifndef __APP_VOICE_H__
#define __APP_VOICE_H__

#include "voice_uart.h"

/* ===== 语音识别命令码，对应协议表 A5 xx 00 00 00 00 00 checksum ===== */
#define APP_VOICE_CMD_WAKEUP             0xF3U   /* 唤醒语音芯片。          */
#define APP_VOICE_CMD_WAKE_WORD          0x00U   /* 你好盛昊/你好小通。     */
#define APP_VOICE_CMD_WASHER_ON          0x01U   /* 打开雨刮喷水。          */
#define APP_VOICE_CMD_LEFT_TURN_ON       0x02U   /* 打开左转灯。           */
#define APP_VOICE_CMD_LEFT_TURN_OFF      0x03U   /* 关闭左转灯。           */
#define APP_VOICE_CMD_RIGHT_TURN_ON      0x04U   /* 打开右转灯。           */
#define APP_VOICE_CMD_RIGHT_TURN_OFF     0x05U   /* 关闭右转灯。           */
#define APP_VOICE_CMD_LOW_BEAM_ON        0x06U   /* 打开大灯/近光灯。       */
#define APP_VOICE_CMD_LOW_BEAM_OFF       0x07U   /* 关闭大灯。             */
#define APP_VOICE_CMD_HIGH_BEAM_ON       0x08U   /* 打开远光灯。           */
#define APP_VOICE_CMD_HIGH_BEAM_OFF      0x09U   /* 关闭远光灯。           */
#define APP_VOICE_CMD_WIPER_OFF          0x0AU   /* 关闭雨刮。             */
#define APP_VOICE_CMD_WIPER_INT          0x0BU   /* 雨刮间歇。             */
#define APP_VOICE_CMD_WIPER_ON           0x0CU   /* 打开雨刮。             */
#define APP_VOICE_CMD_WIPER_HIGH         0x0DU   /* 雨刮高速。             */

/* ===== 车辆控制执行结果 ===== */
typedef enum
{
    APP_VOICE_RESULT_OK     = 0,    /* 命令执行成功，回复 0x00。 */
    APP_VOICE_RESULT_FAIL,          /* 命令执行失败，回复 0x01。 */
    APP_VOICE_RESULT_SAFETY         /* 当前状态不允许语音操作，回复 0x02。 */
} AppVoiceResult_t;

/* ===== 函数声明 ===== */

/**
 * @brief  初始化语音业务模块。
 * @retval 无。
 * @note   该函数应在创建语音任务前调用，内部完成语音 UART、唤醒脚等底层资源初始化。
 */
void App_VoiceInit(void);

/**
 * @brief  处理一帧语音识别命令。
 * @param  pFrame 语音 UART 层解析出的完整识别帧。
 * @retval 无。
 * @note   该函数由 App_VoiceTask 调用，根据命令码分发到车辆控制/CAN 控制层，
 *         并通过 Voice_SendFrame() 回传执行结果。
 */
void App_VoiceProcessFrame(const VoiceFrame_t *pFrame);

#endif /* __APP_VOICE_H__ */
