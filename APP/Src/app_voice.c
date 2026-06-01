/**
 * @file    app_voice.c
 * @brief   语音业务模块实现
 * @details 命令码 → 车辆控制 → 回复语音芯片
 * @note    当前 CAN 总线未实现，所有车辆控制命令暂时返回成功。
 *          CAN 模块完成后，替换 App_VoiceExecuteVehicleCmd() 内的逻辑。
 */

#include "app_voice.h"
#include "app_debug.h"

#define LOG_TAG "voice"

/**
 * @brief  初始化语音业务模块。
 * @retval 无。
 * @note   Voice_UART_Init() 会创建语音接收队列，
 *         所以必须在 App_VoiceTask 调用 Voice_GetRxQueue() 之前执行。
 */
void App_VoiceInit(void)
{
    Voice_UART_Init();
    Voice_WakePin_Init();
}

/**
 * @brief  执行车辆控制命令。
 * @param  cmd 语音命令码。
 * @retval AppVoiceResult_t 执行结果。
 * @note   当前先保留为业务映射入口，
 *         后续这里对接 CAN 发送、状态确认和超时判断。
 */
static AppVoiceResult_t App_VoiceExecuteVehicleCmd(uint8_t cmd)
{
    switch (cmd)
    {
        case APP_VOICE_CMD_WAKEUP:
            return APP_VOICE_RESULT_OK;
        case APP_VOICE_CMD_WAKE_WORD:
            return APP_VOICE_RESULT_OK;

        case APP_VOICE_CMD_WASHER_ON:
        case APP_VOICE_CMD_LEFT_TURN_ON:
        case APP_VOICE_CMD_LEFT_TURN_OFF:
        case APP_VOICE_CMD_RIGHT_TURN_ON:
        case APP_VOICE_CMD_RIGHT_TURN_OFF:
        case APP_VOICE_CMD_LOW_BEAM_ON:
        case APP_VOICE_CMD_HIGH_BEAM_ON:
        case APP_VOICE_CMD_HIGH_BEAM_OFF:
        case APP_VOICE_CMD_WIPER_OFF:
        case APP_VOICE_CMD_WIPER_INT:
        case APP_VOICE_CMD_WIPER_ON:
        case APP_VOICE_CMD_WIPER_HIGH:
            /* TODO: 后续替换为 CAN 控制函数，并根据 CAN 应答返回 OK/FAIL。 */
            return APP_VOICE_RESULT_OK;

        case APP_VOICE_CMD_LOW_BEAM_OFF:
            /* 示例：如果关闭大灯需要安全条件判断，可在这里返回 SAFETY。 */
            return APP_VOICE_RESULT_OK;

        default:
            LOG_WARN("Unknown voice cmd: 0x%02X", cmd);
            return APP_VOICE_RESULT_FAIL;
    }
}

/**
 * @brief  根据执行结果回复语音模块。
 * @param  cmd      原始语音命令码。
 * @param  result   车辆控制执行结果。
 * @retval 无。
 * @note   Voice_SendFrame() 内部会组帧并调用 Voice_CalcChecksum() 生成校验值。
 */
static void App_VoiceReplyResult(uint8_t cmd, AppVoiceResult_t result)
{
    uint8_t respCode = VOICE_RESP_FAIL;

    if (result == APP_VOICE_RESULT_OK)
    {
        respCode = VOICE_RESP_OK;
    }
    else if (result == APP_VOICE_RESULT_SAFETY)
    {
        respCode = VOICE_RESP_SAFETY;
    }

    Voice_SendFrame(cmd, respCode, NULL);
}

/**
 * @brief  处理一帧语音识别命令。
 * @param  pFrame 语音 UART 层解析出的完整识别帧。
 * @retval 无。
 */
void App_VoiceProcessFrame(const VoiceFrame_t *pFrame)
{
    AppVoiceResult_t result;

    if (pFrame == NULL)
    {
        return;
    }

    LOG_INFO("Voice cmd: 0x%02X", pFrame->cmd);

    result = App_VoiceExecuteVehicleCmd(pFrame->cmd);
    App_VoiceReplyResult(pFrame->cmd, result);
}
