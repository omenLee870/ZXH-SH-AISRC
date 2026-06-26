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

#define APP_VOICE_TIME_PERIOD_AM        0x00U   /* 上午：06:00~11:59。 */
#define APP_VOICE_TIME_PERIOD_PM        0x01U   /* 下午：12:00~17:59。 */
#define APP_VOICE_TIME_PERIOD_DAWN      0x02U   /* 凌晨：00:00~05:59。 */
#define APP_VOICE_TIME_PERIOD_NIGHT     0x03U   /* 晚上：18:00~23:59。 */
#define APP_VOICE_CTRL_QUEUE_LEN        4U      /* 控制类语音命令串行等待队列长度。 */

typedef struct
{
    uint8_t             voiceCmd;        /* 语音模块原始命令码，用于最终回复对应词条。 */
    AppVehicleCommand_t vehicleCmd;      /* 内部车辆控制命令，由 VehicleTask 执行。 */
} AppVoiceCtrlItem_t;

static uint8_t s_voiceControlPending = 0U;       /* 车辆控制命令等待 VehicleTask 反馈结果标志。 */
static uint8_t s_voicePendingCmd = 0U;            /* 当前等待回复的语音原始命令码。 */
static AppVoiceCtrlItem_t s_voiceCtrlQueue[APP_VOICE_CTRL_QUEUE_LEN]; /* 控制命令待执行队列。 */
static uint8_t s_voiceCtrlQueueHead = 0U;         /* 待执行队列读指针。 */
static uint8_t s_voiceCtrlQueueTail = 0U;         /* 待执行队列写指针。 */
static uint8_t s_voiceCtrlQueueCount = 0U;        /* 待执行队列当前元素个数。 */

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
 * @brief  判断语音命令是否为信息查询类命令。
 * @param  voiceCmd 语音模块原始命令码。
 * @return 1 = 天气/日期/时间查询，0 = 其他命令。
 */
static uint8_t App_VoiceIsInfoQuery(uint8_t voiceCmd)
{
    return (uint8_t)((voiceCmd == APP_VOICE_CMD_WEATHER_QUERY) ||
                     (voiceCmd == APP_VOICE_CMD_DATE_QUERY) ||
                     (voiceCmd == APP_VOICE_CMD_TIME_QUERY));
}

/**
 * @brief  将控制类语音命令加入待执行队列。
 * @param  voiceCmd   语音模块原始命令码。
 * @param  vehicleCmd 内部车辆控制命令。
 * @return 1 = 入队成功，0 = 队列已满。
 */
static uint8_t App_VoiceCtrlQueuePush(uint8_t voiceCmd,
                                      AppVehicleCommand_t vehicleCmd)
{
    if (s_voiceCtrlQueueCount >= APP_VOICE_CTRL_QUEUE_LEN)
    {
        return 0U;
    }

    s_voiceCtrlQueue[s_voiceCtrlQueueTail].voiceCmd = voiceCmd;
    s_voiceCtrlQueue[s_voiceCtrlQueueTail].vehicleCmd = vehicleCmd;
    s_voiceCtrlQueueTail++;
    if (s_voiceCtrlQueueTail >= APP_VOICE_CTRL_QUEUE_LEN)
    {
        s_voiceCtrlQueueTail = 0U;
    }
    s_voiceCtrlQueueCount++;

    return 1U;
}

/**
 * @brief  从待执行队列取出下一条控制命令。
 * @param  pItem 输出队列元素。
 * @return 1 = 取出成功，0 = 队列为空。
 */
static uint8_t App_VoiceCtrlQueuePop(AppVoiceCtrlItem_t *pItem)
{
    if ((pItem == NULL) || (s_voiceCtrlQueueCount == 0U))
    {
        return 0U;
    }

    *pItem = s_voiceCtrlQueue[s_voiceCtrlQueueHead];
    s_voiceCtrlQueueHead++;
    if (s_voiceCtrlQueueHead >= APP_VOICE_CTRL_QUEUE_LEN)
    {
        s_voiceCtrlQueueHead = 0U;
    }
    s_voiceCtrlQueueCount--;

    return 1U;
}

/**
 * @brief  提交一条控制命令给 VehicleTask，并记录当前等待回复的语音命令。
 * @param  pItem 控制命令队列元素。
 * @retval 无。
 */
static void App_VoiceStartControl(const AppVoiceCtrlItem_t *pItem)
{
    AppVehicleResult_t submitResult;
    uint8_t respCode;
    uint32_t notifyValue = 0U;

    if (pItem == NULL)
    {
        return;
    }

    xTaskNotifyWait(0U, 0xFFFFFFFFUL, &notifyValue, 0U);
    s_voicePendingCmd = pItem->voiceCmd;
    s_voiceControlPending = 1U;

    submitResult = App_ControlSubmit(pItem->vehicleCmd, xTaskGetCurrentTaskHandle());
    if (submitResult != APP_VEHICLE_RESULT_OK)
    {
        s_voiceControlPending = 0U;
        s_voicePendingCmd = 0U;
        respCode = App_VoiceMapRespCode(submitResult);
        Voice_SendFrame(pItem->voiceCmd, respCode, NULL);
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
    uint8_t hour;
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
         * CAN 缓存为 24 小时制；语音协议拆成时间段 + 0~11 小时 + 分钟。
         * 时间段编码：00=上午，01=下午，02=凌晨，03=晚上。
         */
        hour = g_vehicleStatus.tbox_hour;
        if (hour < 6U)
        {
            data[0] = APP_VOICE_TIME_PERIOD_DAWN;
        }
        else if (hour < 12U)
        {
            data[0] = APP_VOICE_TIME_PERIOD_AM;
        }
        else if (hour < 18U)
        {
            data[0] = APP_VOICE_TIME_PERIOD_PM;
        }
        else
        {
            data[0] = APP_VOICE_TIME_PERIOD_NIGHT;
        }

        data[1] = hour % 12U;
        data[2] = g_vehicleStatus.tbox_min;
    }

    /* 三个业务字节全为 0 时按无有效 TBOX 信息处理，避免播报默认空数据。 */
    valid = (uint8_t)((data[0] != 0U) || ((data[1] != 0U) && (data[1] != 0xA8)) || (data[2] != 0U));

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
 * @brief  轮询车辆控制异步执行结果，并在完成时回复语音模块。
 * @retval 无。
 * @note   由 App_VoiceTask 周期调用。VehicleTask 完成 3 秒状态确认后通过任务通知
 *         把结果发回语音任务，本函数负责把结果映射为语音协议应答码。
 */
void App_VoicePollControlResult(void)
{
    uint32_t notifyValue = 0U;
    uint8_t respCode;
    AppVoiceCtrlItem_t nextItem;

    if (s_voiceControlPending == 0U)
    {
        return;
    }

    if (xTaskNotifyWait(0U, 0xFFFFFFFFUL, &notifyValue, 0U) == pdPASS)
    {
        respCode = App_VoiceMapRespCode((AppVehicleResult_t)notifyValue);
        Voice_SendFrame(s_voicePendingCmd, respCode, NULL);
        s_voiceControlPending = 0U;
        s_voicePendingCmd = 0U;

        if (App_VoiceCtrlQueuePop(&nextItem) != 0U)
        {
            App_VoiceStartControl(&nextItem);
        }
    }
}

/**
 * @brief  处理一帧语音识别命令。
 * @param  pFrame 语音 UART 层解析出的完整识别帧。
 * @note   流程：
 *         1. 查表：语音命令码 → 内部统一控制命令
 *         2. 查询类命令直接回复缓存信息
 *         3. 控制类命令异步提交给 VehicleTask，后续由 App_VoicePollControlResult() 回复
 */
void App_VoiceProcessFrame(const VoiceFrame_t *pFrame)
{
    AppVehicleCommand_t vehicleCmd;
    AppVoiceCtrlItem_t  item;

    if (pFrame == NULL)
    {
        return;
    }

    App_VoicePollControlResult();

    LOG_INFO("Voice cmd: 0x%02X", pFrame->cmd);

    /******************** ① 查表：语音命令 → 控制命令 ********************/
    if (App_VoiceCmdMap(pFrame->cmd, &vehicleCmd) == 0U)
    {
        LOG_WARN("Unknown voice cmd: 0x%02X", pFrame->cmd);
        Voice_SendFrame(pFrame->cmd, VOICE_RESP_FAIL, NULL);
        return;
    }

    /******************** ② 查询类命令直接回复缓存信息 ********************/
    if (App_VoiceIsInfoQuery(pFrame->cmd) != 0U)
    {
        App_VoiceReplyInfoQuery(pFrame->cmd);
        return;
    }

    /******************** ③ 控制类命令串行排队，保证语音回复顺序与识别顺序一致 ********************/
    if (s_voiceControlPending != 0U)
    {
        if (App_VoiceCtrlQueuePush(pFrame->cmd, vehicleCmd) == 0U)
        {
            LOG_WARN("Voice control queue full, reject cmd: 0x%02X", pFrame->cmd);
            Voice_SendFrame(pFrame->cmd, VOICE_RESP_FAIL, NULL);
        }
        return;
    }

    /******************** ④ 没有命令执行中，立即启动当前控制命令 ********************/
    item.voiceCmd = pFrame->cmd;
    item.vehicleCmd = vehicleCmd;
    App_VoiceStartControl(&item);
}
