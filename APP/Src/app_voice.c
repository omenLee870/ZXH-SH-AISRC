/**
 * @file    app_voice.c
 * @brief   语音业务模块实现
 * @details 命令码 → 车辆控制 → 回复语音芯片
 * @note    当前 CAN 总线未实现，所有车辆控制命令暂时返回成功。
 *          CAN 模块完成后，替换 App_VoiceExecuteVehicleCmd() 内的逻辑。
 */

#include "app_voice.h"
#include "app_debug.h"
#include "app_vehicle.h"

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

/* ===== 语音命令 → 控制命令查表映射 ===== */
/**
 * @brief  映射表：语音芯片原始命令码 → 内部统一控制命令。
 * @note   增删命令只需在此数组增删一行，无需改动其他代码。
 */
static const struct
{
    uint8_t             voiceCmd;       /* 语音芯片发来的原始值。       */
    AppVehicleCommand_t vehicleCmd;     /* 内部统一命令编号。           */
} s_voiceCmdMap[] =
{
    { APP_VOICE_CMD_WAKEUP,          APP_VEHICLE_CMD_WAKEUP          },
    { APP_VOICE_CMD_WAKE_WORD,       APP_VEHICLE_CMD_WAKE_WORD       },
    { APP_VOICE_CMD_WASHER_ON,       APP_VEHICLE_CMD_WASHER_ON       },
    { APP_VOICE_CMD_LEFT_TURN_ON,    APP_VEHICLE_CMD_LEFT_TURN_ON    },
    { APP_VOICE_CMD_LEFT_TURN_OFF,   APP_VEHICLE_CMD_LEFT_TURN_OFF   },
    { APP_VOICE_CMD_RIGHT_TURN_ON,   APP_VEHICLE_CMD_RIGHT_TURN_ON   },
    { APP_VOICE_CMD_RIGHT_TURN_OFF,  APP_VEHICLE_CMD_RIGHT_TURN_OFF  },
    { APP_VOICE_CMD_LOW_BEAM_ON,     APP_VEHICLE_CMD_LOW_BEAM_ON     },
    { APP_VOICE_CMD_LOW_BEAM_OFF,    APP_VEHICLE_CMD_LOW_BEAM_OFF    },
    { APP_VOICE_CMD_HIGH_BEAM_ON,    APP_VEHICLE_CMD_HIGH_BEAM_ON    },
    { APP_VOICE_CMD_HIGH_BEAM_OFF,   APP_VEHICLE_CMD_HIGH_BEAM_OFF   },
    { APP_VOICE_CMD_WIPER_OFF,       APP_VEHICLE_CMD_WIPER_OFF       },
    { APP_VOICE_CMD_WIPER_INT,       APP_VEHICLE_CMD_WIPER_INTERVAL  },
    { APP_VOICE_CMD_WIPER_ON,        APP_VEHICLE_CMD_WIPER_ON        },
    { APP_VOICE_CMD_WIPER_HIGH,      APP_VEHICLE_CMD_WIPER_HIGH      },
};

#define VOICE_CMD_MAP_SIZE  (sizeof(s_voiceCmdMap) / sizeof(s_voiceCmdMap[0]))

/**
 * @brief  查表：语音命令 → 控制命令。
 * @param  voiceCmd  语音芯片命令码（0x00~0x0D）。
 * @param  pOut      输出：对应的车辆控制命令。
 * @return 1 = 找到, 0 = 未找到。
 */
static uint8_t App_VoiceCmdMap(uint8_t voiceCmd, AppVehicleCommand_t *pOut)
{
    uint8_t i;

    if (pOut == NULL)
    {
        return 0U;
    }

    for (i = 0; i < VOICE_CMD_MAP_SIZE; i++)
    {
        LOG_INFO("voiceTable[%d] table_voiceCmd: 0x%02X, table_voiceCmd: 0x%02X", i, s_voiceCmdMap[i].voiceCmd, voiceCmd);
        if (s_voiceCmdMap[i].voiceCmd == voiceCmd)
        {
            *pOut = s_voiceCmdMap[i].vehicleCmd;
            return 1U;
        }
    }

    return 0U;
}

/**
 * @brief  控制结果 → 协议应答码映射。
 * @param  result  VehicleTask 返回的执行结果。
 * @return 协议应答码（VOICE_RESP_OK / FAIL / SAFETY）。
 */
static uint8_t App_VoiceMapRespCode(AppVehicleResult_t result)
{
    if (result == APP_VEHICLE_RESULT_OK)
    {
        return VOICE_RESP_OK;
    }
    else if (result == APP_VEHICLE_RESULT_SAFETY)
    {
        return VOICE_RESP_SAFETY;
    }
    else
    {
        return VOICE_RESP_FAIL;
    }
}

/**
 * @brief  处理一帧语音识别命令。
 * @param  pFrame 语音 UART 层解析出的完整识别帧。
 * @note   流程：
 *         1. 查表：语音命令码 → 内部统一控制命令
 *         2. 委托 VehicleTask 执行（同步等待，最多 500ms）
 *         3. 结果 → 应答码 → 回复语音芯片
 */
void App_VoiceProcessFrame(const VoiceFrame_t *pFrame)
{
    AppVehicleCommand_t vehicleCmd;
    AppVehicleResult_t  result;
    uint8_t             respCode;

    if (pFrame == NULL)
    {
        return;
    }

    LOG_INFO("Voice cmd: 0x%02X", pFrame->cmd);

    /******************** ① 查表：语音命令 → 控制命令 ********************/
    if (App_VoiceCmdMap(pFrame->cmd, &vehicleCmd) == 0U)
    {
        LOG_WARN("Unknown voice cmd: 0x%02X", pFrame->cmd);
        Voice_SendFrame(pFrame->cmd, VOICE_RESP_FAIL, NULL);
        return;
    }

    /******************** ② 委托 VehicleTask 执行（最多等 500ms） ********************/
    result = App_ControlRequest(vehicleCmd, pdMS_TO_TICKS(500U));

    /******************** ③ 结果 → 应答 → 回复语音芯片 ********************/
    respCode = App_VoiceMapRespCode(result);
    Voice_SendFrame(pFrame->cmd, respCode, NULL);
}
