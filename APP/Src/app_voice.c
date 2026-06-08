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
#include "app_can_proto.h"

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
    { APP_VOICE_CMD_WAKEUP,              APP_VEHICLE_CMD_WAKEUP          },
    { APP_VOICE_CMD_WAKE_WORD,           APP_VEHICLE_CMD_WAKE_WORD       },
    { APP_VOICE_CMD_WASHER_ON,           APP_VEHICLE_CMD_WASHER_ON       },
    { APP_VOICE_CMD_LEFT_TURN_ON,        APP_VEHICLE_CMD_LEFT_TURN_ON    },
    { APP_VOICE_CMD_LEFT_TURN_OFF,       APP_VEHICLE_CMD_LEFT_TURN_OFF   },
    { APP_VOICE_CMD_RIGHT_TURN_ON,       APP_VEHICLE_CMD_RIGHT_TURN_ON   },
    { APP_VOICE_CMD_RIGHT_TURN_OFF,      APP_VEHICLE_CMD_RIGHT_TURN_OFF  },
    { APP_VOICE_CMD_LOW_BEAM_ON,         APP_VEHICLE_CMD_LOW_BEAM_ON     },
    { APP_VOICE_CMD_LOW_BEAM_OFF,        APP_VEHICLE_CMD_LOW_BEAM_OFF    },
    { APP_VOICE_CMD_HIGH_BEAM_ON,        APP_VEHICLE_CMD_HIGH_BEAM_ON    },
    { APP_VOICE_CMD_HIGH_BEAM_OFF,       APP_VEHICLE_CMD_HIGH_BEAM_OFF   },
    { APP_VOICE_CMD_WIPER_OFF,           APP_VEHICLE_CMD_WIPER_OFF       },
    { APP_VOICE_CMD_WIPER_INT,           APP_VEHICLE_CMD_WIPER_INTERVAL  },
    { APP_VOICE_CMD_WIPER_ON,            APP_VEHICLE_CMD_WIPER_ON        },
    { APP_VOICE_CMD_WIPER_HIGH,          APP_VEHICLE_CMD_WIPER_HIGH      },
    { APP_VOICE_CMD_PARKING_LIGHT_ON,    APP_VEHICLE_CMD_PARKING_LIGHT_ON    },
    { APP_VOICE_CMD_PARKING_LIGHT_OFF,   APP_VEHICLE_CMD_PARKING_LIGHT_OFF   },
    { APP_VOICE_CMD_REAR_FOG_ON,         APP_VEHICLE_CMD_REAR_FOG_ON         },
    { APP_VOICE_CMD_REAR_FOG_OFF,        APP_VEHICLE_CMD_REAR_FOG_OFF        },
    { APP_VOICE_CMD_HAZARD_ON,           APP_VEHICLE_CMD_HAZARD_ON           },
    { APP_VOICE_CMD_HAZARD_OFF,          APP_VEHICLE_CMD_HAZARD_OFF          },
    { APP_VOICE_CMD_TRUNK_UNLOCK,        APP_VEHICLE_CMD_TRUNK_UNLOCK        },
    { APP_VOICE_CMD_MIRROR_UNFOLD,       APP_VEHICLE_CMD_MIRROR_UNFOLD       },
    { APP_VOICE_CMD_MIRROR_FOLD,         APP_VEHICLE_CMD_MIRROR_FOLD         },
    { APP_VOICE_CMD_READING_LIGHT_ON,    APP_VEHICLE_CMD_READING_LIGHT_ON    },
    { APP_VOICE_CMD_READING_LIGHT_OFF,   APP_VEHICLE_CMD_READING_LIGHT_OFF   },
    { APP_VOICE_CMD_SUNROOF_FAN_ON,      APP_VEHICLE_CMD_SUNROOF_FAN_ON      },
    { APP_VOICE_CMD_SUNROOF_FAN_OFF,     APP_VEHICLE_CMD_SUNROOF_FAN_OFF     },
    { APP_VOICE_CMD_SUNROOF_FAN_L1,      APP_VEHICLE_CMD_SUNROOF_FAN_L1      },
    { APP_VOICE_CMD_SUNROOF_FAN_L2,      APP_VEHICLE_CMD_SUNROOF_FAN_L2      },
    { APP_VOICE_CMD_SUNROOF_FAN_L3,      APP_VEHICLE_CMD_SUNROOF_FAN_L3      },
    { APP_VOICE_CMD_AC_HEAT_ON,          APP_VEHICLE_CMD_AC_HEAT_ON          },
    { APP_VOICE_CMD_AC_COOL_ON,          APP_VEHICLE_CMD_AC_COOL_ON          },
    { APP_VOICE_CMD_AC_OFF,              APP_VEHICLE_CMD_AC_OFF              },
    { APP_VOICE_CMD_AC_LEVEL1,           APP_VEHICLE_CMD_AC_LEVEL1           },
    { APP_VOICE_CMD_AC_LEVEL2,           APP_VEHICLE_CMD_AC_LEVEL2           },
    { APP_VOICE_CMD_AC_LEVEL3,           APP_VEHICLE_CMD_AC_LEVEL3           },
    { APP_VOICE_CMD_WEATHER_QUERY,       APP_VEHICLE_CMD_WEATHER_QUERY       },
    { APP_VOICE_CMD_DATE_QUERY,          APP_VEHICLE_CMD_DATE_QUERY          },
    { APP_VOICE_CMD_TIME_QUERY,          APP_VEHICLE_CMD_TIME_QUERY          },
    { APP_VOICE_CMD_VOLUME_UP,           APP_VEHICLE_CMD_VOLUME_UP           },
    { APP_VOICE_CMD_VOLUME_DOWN,         APP_VEHICLE_CMD_VOLUME_DOWN         },
    { APP_VOICE_CMD_VOLUME_MAX,          APP_VEHICLE_CMD_VOLUME_MAX          },
    { APP_VOICE_CMD_VOLUME_MIN,          APP_VEHICLE_CMD_VOLUME_MIN          },
    { APP_VOICE_CMD_DATA_WAKEUP,         APP_VEHICLE_CMD_DATA_WAKEUP         },
    { APP_VOICE_CMD_DATA_EXIT_WAKEUP,    APP_VEHICLE_CMD_DATA_EXIT_WAKEUP    },
    { APP_VOICE_CMD_15S_EXIT_WAKEUP,     APP_VEHICLE_CMD_15S_EXIT_WAKEUP     },
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
 * @brief  处理需要携带数据返回的语音信息查询命令。
 * @param  voiceCmd 语音模块原始命令码，例如天气/日期/时间查询。
 */
static void App_VoiceReplyInfoQuery(uint8_t voiceCmd)
{
    uint8_t data[5] = {0};
    uint8_t valid = 0U;
    int16_t temp;

    if (voiceCmd == APP_VOICE_CMD_WEATHER_QUERY)
    {
        /*
         * CAN 协议 TBOX_Temp 物理值 = 原始值 - 40；
         * 语音协议负温度编码为 0x80 + 绝对值，例如 -40 度为 0xA8。
         */
        temp = (int16_t)g_vehicleStatus.tbox_temp_raw - 40;

        data[0] = g_vehicleStatus.tbox_weather;
        data[1] = (temp < 0) ? (0x80U + (uint8_t)(-temp)) : (uint8_t)temp;
        data[2] = g_vehicleStatus.tbox_wind_speed;
    }
    else if (voiceCmd == APP_VOICE_CMD_DATE_QUERY)
    {
        /* 语音协议日期回复：年为 2000 年偏移值，月/日直接使用 TBOX_DPLY 缓存。 */
        data[0] = g_vehicleStatus.tbox_year;
        data[1] = g_vehicleStatus.tbox_month;
        data[2] = g_vehicleStatus.tbox_date;
    }
    else
    {
        /*
         * CAN 缓存为 24 小时制；语音协议拆成上午/下午 + 0~11 小时 + 分钟。
         * 12:xx 及以后回复下午，小时字段按协议取 12 小时制余数。
         */
        data[0] = (g_vehicleStatus.tbox_hour >= 12U) ? 1U : 0U;
        data[1] = g_vehicleStatus.tbox_hour % 12U;
        data[2] = g_vehicleStatus.tbox_min;
    }

    /* 三个业务字节全为 0 时按无有效 TBOX 信息处理，避免播报默认空数据。 */
    valid = (uint8_t)((data[0] != 0U) || (data[1] != 0U) || (data[2] != 0U));

    if (valid != 0U)
    {
        Voice_SendFrame(voiceCmd, VOICE_RESP_OK, data);
    }
    else
    {
        Voice_SendFrame(voiceCmd, VOICE_RESP_FAIL, NULL);
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
    if ((pFrame->cmd != APP_VOICE_CMD_WEATHER_QUERY) &&
        (pFrame->cmd != APP_VOICE_CMD_DATE_QUERY) &&
        (pFrame->cmd != APP_VOICE_CMD_TIME_QUERY))
    {
        respCode = App_VoiceMapRespCode(result);
        Voice_SendFrame(pFrame->cmd, respCode, NULL);
    }else{
        App_VoiceReplyInfoQuery(pFrame->cmd);
    }
}
