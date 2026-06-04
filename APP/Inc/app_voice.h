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
#define APP_VOICE_CMD_PARKING_LIGHT_ON   0x0EU   /* 打开小灯/示廓灯。        */
#define APP_VOICE_CMD_PARKING_LIGHT_OFF  0x0FU   /* 关闭小灯/示廓灯。        */
#define APP_VOICE_CMD_REAR_FOG_ON        0x10U   /* 打开后雾灯。             */
#define APP_VOICE_CMD_REAR_FOG_OFF       0x11U   /* 关闭后雾灯。             */
#define APP_VOICE_CMD_HAZARD_ON          0x12U   /* 打开双闪/危险报警灯。     */
#define APP_VOICE_CMD_HAZARD_OFF         0x13U   /* 关闭双闪/危险报警灯。     */
#define APP_VOICE_CMD_TRUNK_UNLOCK       0x14U   /* 打开后备箱。             */
#define APP_VOICE_CMD_MIRROR_UNFOLD      0x15U   /* 展开后视镜。             */
#define APP_VOICE_CMD_MIRROR_FOLD        0x16U   /* 折叠后视镜。             */
#define APP_VOICE_CMD_READING_LIGHT_ON   0x17U   /* 打开室内灯/阅读灯。       */
#define APP_VOICE_CMD_READING_LIGHT_OFF  0x18U   /* 关闭室内灯/阅读灯。       */
#define APP_VOICE_CMD_SUNROOF_FAN_ON     0x19U   /* 打开天窗风扇（默认1档）。  */
#define APP_VOICE_CMD_SUNROOF_FAN_OFF    0x1AU   /* 关闭天窗风扇。            */
#define APP_VOICE_CMD_SUNROOF_FAN_L1     0x1BU   /* 风扇一档（最小）。         */
#define APP_VOICE_CMD_SUNROOF_FAN_L2     0x1CU   /* 风扇二档。               */
#define APP_VOICE_CMD_SUNROOF_FAN_L3     0x1DU   /* 风扇三档（最大）。         */
#define APP_VOICE_CMD_AC_HEAT_ON         0x1EU   /* 打开空调制热。            */
#define APP_VOICE_CMD_AC_COOL_ON         0x1FU   /* 打开空调制冷。            */
#define APP_VOICE_CMD_AC_OFF             0x20U   /* 关闭空调。               */
#define APP_VOICE_CMD_AC_LEVEL1          0x21U   /* 空调一档（最小）。         */
#define APP_VOICE_CMD_AC_LEVEL2          0x22U   /* 空调二档。               */
#define APP_VOICE_CMD_AC_LEVEL3          0x23U   /* 空调三档（最大）。         */
#define APP_VOICE_CMD_WEATHER_QUERY      0x24U   /* 查询今天天气。            */
#define APP_VOICE_CMD_DATE_QUERY         0x25U   /* 查询今天日期。            */
#define APP_VOICE_CMD_TIME_QUERY         0x26U   /* 查询当前时间。            */
#define APP_VOICE_CMD_VOLUME_UP          0x27U   /* 增大音量。               */
#define APP_VOICE_CMD_VOLUME_DOWN        0x28U   /* 减小音量。               */
#define APP_VOICE_CMD_VOLUME_MAX         0x29U   /* 最大音量。               */
#define APP_VOICE_CMD_VOLUME_MIN         0x2AU   /* 最小音量。               */
#define APP_VOICE_CMD_DATA_WAKEUP        0xF0U   /* 数据唤醒（MCU 先发 A5 F0）。 */
#define APP_VOICE_CMD_DATA_EXIT_WAKEUP   0xF1U   /* 数据退出唤醒。            */
#define APP_VOICE_CMD_15S_EXIT_WAKEUP    0xF2U   /* 15 秒超时退出唤醒。       */
#define APP_VOICE_CMD_WAKEUP             0xF3U   /* 唤醒语音芯片。          */

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
