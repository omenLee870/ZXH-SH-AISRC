/**
 * @file    app_vehicle.c
 * @brief   车辆控制模块实现
 * @details 统一接收所有控制请求（语音/按键/BLE），
 *          集中执行安全判断、CAN 发送、结果返回。
 *
 * 请求链路：
 *   同步请求：上层 → App_ControlRequest() → 队列 → VehicleTask → 执行 → 任务通知 → 上层
 *   语音请求：语音 → App_ControlSubmit() → 队列 → VehicleTask → pending 表 → 语音回复
 */

#include "app_vehicle.h"
#include "app_debug.h"
#include "queue.h"
#include "app_can_proto.h"
#include "voice_uart.h"
#include <stddef.h>

#define LOG_TAG "vehicle"

/**
 * @brief  CAN 控制目标类型。
 */
typedef enum
{
    APP_VEHICLE_CAN_TARGET_BCM = 0,      /* 发 IVI_BCM，等待 BCM_TBOX1 反馈。 */
    APP_VEHICLE_CAN_TARGET_ACU,          /* 发 IVI_ACU，等待 ACU_IVI 反馈。   */
    APP_VEHICLE_CAN_TARGET_SRCM          /* 发 IVI_BCM，等待 SRCM 反馈。      */
} AppVehicleCanTarget_t;

/**
 * @brief  单条车辆控制命令的表驱动配置。
 */
typedef struct
{
    AppVehicleCommand_t   cmd;           /* 内部车辆控制命令。 */
    AppVehicleCanTarget_t target;        /* 发送目标和反馈来源。 */

    uint8_t txByte;                      /* 要写入的 CAN 数据字节下标。 */
    uint8_t txValue;                     /* 已经左移到目标 bit 位置后的值。 */

    uint16_t statusOffset;               /* VehicleStatus_t 中反馈字段的偏移。 */
    uint8_t expected;                    /* 期望反馈值。 */
} AppVehicleCmdMap_t;

/**
 * @brief  语音异步控制 pending 项。
 */
typedef struct
{
    uint8_t voiceCmd;                         /* 原始语音命令码，用于完成后回复语音模块。 */
    uint8_t used;                             /* 1 = 本槽位正在等待反馈，0 = 空闲。 */
    TickType_t startTick;                     /* 发送请求的系统 tick，用于 3s 超时判断。 */
    uint32_t oldRxTick;                       /* 发送请求前对应反馈帧的时间戳，用于识别新回复。 */
    const AppVehicleCmdMap_t *pMap;           /* 指向命令映射表项，提供反馈字段和期望值。 */
} AppVehiclePending_t;

#define APP_VEHICLE_REQ_QUEUE_LENGTH        32U
#define APP_VEHICLE_REQ_SEND_TIMEOUT_MS     20U

/** RX 反馈等待时间（ms），给 BCM/ACU/SRCM 至少一个周期的反应时间 */
#define APP_VEHICLE_RX_WAIT_MS              150U

#define APP_VEHICLE_PENDING_MAX             32U      /* 3s 窗口内允许等待反馈的语音控制命令数量。 */
#define APP_VEHICLE_PENDING_TIMEOUT_MS      3000U    /* 单条语音控制命令等待车身状态达到目标的最长时间。 */

static QueueHandle_t s_vehicleReqQueue = NULL;
static AppVehiclePending_t s_vehiclePending[APP_VEHICLE_PENDING_MAX];

static const AppVehicleCmdMap_t s_vehicleCmdMap[] =
{
    { APP_VEHICLE_CMD_LEFT_TURN_ON,      APP_VEHICLE_CAN_TARGET_BCM,  5U, CAN_SET_2BIT(CAN_BCM_TURN_ON, CAN_BCM_LH_TURN_POS),              offsetof(VehicleStatus_t, lh_turn_light_out),        1U },
    { APP_VEHICLE_CMD_LEFT_TURN_OFF,     APP_VEHICLE_CAN_TARGET_BCM,  5U, CAN_SET_2BIT(CAN_BCM_TURN_OFF, CAN_BCM_LH_TURN_POS),             offsetof(VehicleStatus_t, lh_turn_light_out),        0U },
    { APP_VEHICLE_CMD_RIGHT_TURN_ON,     APP_VEHICLE_CAN_TARGET_BCM,  5U, CAN_SET_2BIT(CAN_BCM_TURN_ON, CAN_BCM_RH_TURN_POS),              offsetof(VehicleStatus_t, rh_turn_light_out),        1U },
    { APP_VEHICLE_CMD_RIGHT_TURN_OFF,    APP_VEHICLE_CAN_TARGET_BCM,  5U, CAN_SET_2BIT(CAN_BCM_TURN_OFF, CAN_BCM_RH_TURN_POS),             offsetof(VehicleStatus_t, rh_turn_light_out),        0U },

    { APP_VEHICLE_CMD_LOW_BEAM_ON,       APP_VEHICLE_CAN_TARGET_BCM,  3U, CAN_SET_2BIT(CAN_BCM_LIGHT_ON, CAN_BCM_LOW_BEAM_POS),            offsetof(VehicleStatus_t, low_beam_light_out),       1U },
    { APP_VEHICLE_CMD_LOW_BEAM_OFF,      APP_VEHICLE_CAN_TARGET_BCM,  3U, CAN_SET_2BIT(CAN_BCM_LIGHT_OFF, CAN_BCM_LOW_BEAM_POS),           offsetof(VehicleStatus_t, low_beam_light_out),       0U },
    { APP_VEHICLE_CMD_HIGH_BEAM_ON,      APP_VEHICLE_CAN_TARGET_BCM,  3U, CAN_SET_2BIT(CAN_BCM_LIGHT_ON, CAN_BCM_HIGH_BEAM_POS),           offsetof(VehicleStatus_t, high_beam_light_status),   1U },
    { APP_VEHICLE_CMD_HIGH_BEAM_OFF,     APP_VEHICLE_CAN_TARGET_BCM,  3U, CAN_SET_2BIT(CAN_BCM_LIGHT_OFF, CAN_BCM_HIGH_BEAM_POS),          offsetof(VehicleStatus_t, high_beam_light_status),   0U },

    { APP_VEHICLE_CMD_WIPER_OFF,         APP_VEHICLE_CAN_TARGET_BCM,  6U, CAN_SET_3BIT(CAN_BCM_WIPER_OFF, CAN_BCM_FRONT_WIPER_POS),        offsetof(VehicleStatus_t, front_wiper_status),       0U },
    { APP_VEHICLE_CMD_WIPER_INTERVAL,    APP_VEHICLE_CAN_TARGET_BCM,  6U, CAN_SET_3BIT(CAN_BCM_WIPER_INTERVAL, CAN_BCM_FRONT_WIPER_POS),   offsetof(VehicleStatus_t, front_wiper_status),       1U },
    { APP_VEHICLE_CMD_WIPER_ON,          APP_VEHICLE_CAN_TARGET_BCM,  6U, CAN_SET_3BIT(CAN_BCM_WIPER_LOW, CAN_BCM_FRONT_WIPER_POS),        offsetof(VehicleStatus_t, front_wiper_status),       2U },
    { APP_VEHICLE_CMD_WIPER_HIGH,        APP_VEHICLE_CAN_TARGET_BCM,  6U, CAN_SET_3BIT(CAN_BCM_WIPER_HIGH, CAN_BCM_FRONT_WIPER_POS),       offsetof(VehicleStatus_t, front_wiper_status),       3U },
    { APP_VEHICLE_CMD_WASHER_ON,         APP_VEHICLE_CAN_TARGET_BCM,  2U, CAN_SET_2BIT(CAN_BCM_WASHER_ON, CAN_BCM_WASHER_POS),             offsetof(VehicleStatus_t, wiper_wash_out),           1U },

    { APP_VEHICLE_CMD_PARKING_LIGHT_ON,  APP_VEHICLE_CAN_TARGET_BCM,  3U, CAN_SET_2BIT(CAN_BCM_LIGHT_ON, CAN_BCM_PARKING_LIGHT_POS),       offsetof(VehicleStatus_t, parking_light_out),        1U },
    { APP_VEHICLE_CMD_PARKING_LIGHT_OFF, APP_VEHICLE_CAN_TARGET_BCM,  3U, CAN_SET_2BIT(CAN_BCM_LIGHT_OFF, CAN_BCM_PARKING_LIGHT_POS),      offsetof(VehicleStatus_t, parking_light_out),        0U },
    { APP_VEHICLE_CMD_REAR_FOG_ON,       APP_VEHICLE_CAN_TARGET_BCM,  4U, CAN_SET_2BIT(CAN_BCM_LIGHT_ON, CAN_BCM_REAR_FOG_POS),            offsetof(VehicleStatus_t, rear_fog_light_status),    1U },
    { APP_VEHICLE_CMD_REAR_FOG_OFF,      APP_VEHICLE_CAN_TARGET_BCM,  4U, CAN_SET_2BIT(CAN_BCM_LIGHT_OFF, CAN_BCM_REAR_FOG_POS),           offsetof(VehicleStatus_t, rear_fog_light_status),    0U },
    { APP_VEHICLE_CMD_HAZARD_ON,         APP_VEHICLE_CAN_TARGET_BCM,  4U, CAN_SET_2BIT(CAN_BCM_LIGHT_ON, CAN_BCM_HAZARD_POS),              offsetof(VehicleStatus_t, hazard_light_switch),      1U },
    { APP_VEHICLE_CMD_HAZARD_OFF,        APP_VEHICLE_CAN_TARGET_BCM,  4U, CAN_SET_2BIT(CAN_BCM_LIGHT_OFF, CAN_BCM_HAZARD_POS),             offsetof(VehicleStatus_t, hazard_light_switch),      0U },

    { APP_VEHICLE_CMD_TRUNK_UNLOCK,      APP_VEHICLE_CAN_TARGET_BCM,  4U, CAN_SET_2BIT(CAN_BCM_TRUNK_UNLOCK, CAN_BCM_TRUNK_UNLOCK_POS),    offsetof(VehicleStatus_t, trunk_unlock_out),         1U },
    { APP_VEHICLE_CMD_MIRROR_UNFOLD,     APP_VEHICLE_CAN_TARGET_BCM,  2U, CAN_SET_2BIT(CAN_BCM_MIRROR_UNFOLD, CAN_BCM_MIRROR_FOLD_POS),    offsetof(VehicleStatus_t, outer_mirror_fold),        1U },
    { APP_VEHICLE_CMD_MIRROR_FOLD,       APP_VEHICLE_CAN_TARGET_BCM,  2U, CAN_SET_2BIT(CAN_BCM_MIRROR_FOLD, CAN_BCM_MIRROR_FOLD_POS),      offsetof(VehicleStatus_t, outer_mirror_fold),        2U },
    { APP_VEHICLE_CMD_READING_LIGHT_ON,  APP_VEHICLE_CAN_TARGET_BCM,  6U, CAN_SET_2BIT(CAN_BCM_READING_LIGHT_ON, CAN_BCM_READING_LIGHT_POS), offsetof(VehicleStatus_t, reading_light_out),        1U },
    { APP_VEHICLE_CMD_READING_LIGHT_OFF, APP_VEHICLE_CAN_TARGET_BCM,  6U, CAN_SET_2BIT(CAN_BCM_READING_LIGHT_OFF, CAN_BCM_READING_LIGHT_POS), offsetof(VehicleStatus_t, reading_light_out),       0U },

    { APP_VEHICLE_CMD_SUNROOF_FAN_ON,    APP_VEHICLE_CAN_TARGET_SRCM, 5U, CAN_SET_3BIT(CAN_BCM_SUNROOF_FAN_LEVEL1, CAN_BCM_SUNROOF_FAN_POS), offsetof(VehicleStatus_t, sr_fan_level),             1U },
    { APP_VEHICLE_CMD_SUNROOF_FAN_OFF,   APP_VEHICLE_CAN_TARGET_SRCM, 5U, CAN_SET_3BIT(CAN_BCM_SUNROOF_FAN_OFF, CAN_BCM_SUNROOF_FAN_POS),    offsetof(VehicleStatus_t, sr_fan_level),             0U },
    { APP_VEHICLE_CMD_SUNROOF_FAN_L1,    APP_VEHICLE_CAN_TARGET_SRCM, 5U, CAN_SET_3BIT(CAN_BCM_SUNROOF_FAN_LEVEL1, CAN_BCM_SUNROOF_FAN_POS), offsetof(VehicleStatus_t, sr_fan_level),             1U },
    { APP_VEHICLE_CMD_SUNROOF_FAN_L2,    APP_VEHICLE_CAN_TARGET_SRCM, 5U, CAN_SET_3BIT(CAN_BCM_SUNROOF_FAN_LEVEL2, CAN_BCM_SUNROOF_FAN_POS), offsetof(VehicleStatus_t, sr_fan_level),             2U },
    { APP_VEHICLE_CMD_SUNROOF_FAN_L3,    APP_VEHICLE_CAN_TARGET_SRCM, 5U, CAN_SET_3BIT(CAN_BCM_SUNROOF_FAN_LEVEL3, CAN_BCM_SUNROOF_FAN_POS), offsetof(VehicleStatus_t, sr_fan_level),             3U },

    { APP_VEHICLE_CMD_AC_HEAT_ON,        APP_VEHICLE_CAN_TARGET_ACU,  1U, CAN_ACU_MODE_HEAT,                                              offsetof(VehicleStatus_t, ac_mode),                  1U },
    { APP_VEHICLE_CMD_AC_COOL_ON,        APP_VEHICLE_CAN_TARGET_ACU,  1U, CAN_ACU_MODE_COOL,                                              offsetof(VehicleStatus_t, ac_mode),                  2U },
    { APP_VEHICLE_CMD_AC_OFF,            APP_VEHICLE_CAN_TARGET_ACU,  1U, CAN_ACU_MODE_OFF,                                               offsetof(VehicleStatus_t, ac_mode),                  0U },
    { APP_VEHICLE_CMD_AC_LEVEL1,         APP_VEHICLE_CAN_TARGET_ACU,  0U, CAN_ACU_FAN_LEVEL1,                                             offsetof(VehicleStatus_t, ac_fan_gear),              1U },
    { APP_VEHICLE_CMD_AC_LEVEL2,         APP_VEHICLE_CAN_TARGET_ACU,  0U, CAN_ACU_FAN_LEVEL2,                                             offsetof(VehicleStatus_t, ac_fan_gear),              2U },
    { APP_VEHICLE_CMD_AC_LEVEL3,         APP_VEHICLE_CAN_TARGET_ACU,  0U, CAN_ACU_FAN_LEVEL3,                                             offsetof(VehicleStatus_t, ac_fan_gear),              3U }
};

#define APP_VEHICLE_CMD_MAP_SIZE \
    (sizeof(s_vehicleCmdMap) / sizeof(s_vehicleCmdMap[0]))

/**
 * @brief  根据车辆命令查找表驱动配置。
 */
static const AppVehicleCmdMap_t *App_VehicleFindCmdMap(AppVehicleCommand_t cmd)
{
    uint8_t i;

    for (i = 0U; i < APP_VEHICLE_CMD_MAP_SIZE; i++)
    {
        if (s_vehicleCmdMap[i].cmd == cmd)
        {
            return &s_vehicleCmdMap[i];
        }
    }

    return NULL;
}

/**
 * @brief  判断命令是否不需要等待车身 CAN 反馈。
 * @param  cmd 内部统一车辆控制命令。
 * @return 1 = 可立即回复成功, 0 = 需要按映射表发送 CAN 并等待反馈。
 */
static uint8_t App_VehicleIsImmediateOkCmd(AppVehicleCommand_t cmd)
{
    switch (cmd)
    {
        case APP_VEHICLE_CMD_WAKEUP:
        case APP_VEHICLE_CMD_WAKE_WORD:
        case APP_VEHICLE_CMD_WEATHER_QUERY:
        case APP_VEHICLE_CMD_DATE_QUERY:
        case APP_VEHICLE_CMD_TIME_QUERY:
        case APP_VEHICLE_CMD_VOLUME_UP:
        case APP_VEHICLE_CMD_VOLUME_DOWN:
        case APP_VEHICLE_CMD_VOLUME_MAX:
        case APP_VEHICLE_CMD_VOLUME_MIN:
        case APP_VEHICLE_CMD_DATA_WAKEUP:
        case APP_VEHICLE_CMD_DATA_EXIT_WAKEUP:
        case APP_VEHICLE_CMD_15S_EXIT_WAKEUP:
            return 1U;

        default:
            return 0U;
    }
}

/**
 * @brief  从车辆状态结构体中读取一个 uint8_t 状态字段。
 */
static uint8_t App_VehicleReadStatusU8(uint16_t offset)
{
    const uint8_t *pBase = (const uint8_t *)&g_vehicleStatus;

    return *(const uint8_t *)(pBase + offset);
}

/**
 * @brief  将车辆执行结果映射为语音应答码。
 * @param  result 车辆控制结果。
 * @return 语音协议应答码。
 */
static uint8_t App_VehicleVoiceRespCode(AppVehicleResult_t result)
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
 * @brief  获取映射表对应反馈报文的最近更新时间戳。
 * @param  pMap 命令映射表项。
 * @return 反馈报文更新时间戳；映射无效时返回 0。
 */
static uint32_t App_VehicleGetRxTickByMap(const AppVehicleCmdMap_t *pMap)
{
    if (pMap == NULL)
    {
        return 0U;
    }

    if (pMap->target == APP_VEHICLE_CAN_TARGET_BCM)
    {
        return g_vehicleStatus.bcm_tbox1_tick;
    }
    else if (pMap->target == APP_VEHICLE_CAN_TARGET_ACU)
    {
        return g_vehicleStatus.acu_ivi_tick;
    }
    else
    {
        return g_vehicleStatus.srcm_tick;
    }
}

/**
 * @brief  发送映射表描述的 CAN 请求帧。
 * @param  pMap 命令映射表项。
 * @return 0 = 发送成功, 其他 = 发送失败。
 */
static int App_VehicleSendRequestByMap(const AppVehicleCmdMap_t *pMap)
{
    uint8_t data[8] = {0};

    if (pMap == NULL)
    {
        return -1;
    }

    data[pMap->txByte] = pMap->txValue;

    if (pMap->target == APP_VEHICLE_CAN_TARGET_ACU)
    {
        return APP_CAN_SendIVI_ACU(data);
    }
    else
    {
        return APP_CAN_SendIVI_BCM(data);
    }
}

/**
 * @brief  向指定 CAN 控制通道发送全 0x00 无请求帧。
 * @param  target CAN 控制目标。
 * @retval 无。
 * @note   协议中 Signal Value Description = 0x00 表示无请求；
 *         pending 指令完成或超时后发送该帧，释放 IVI 对车身控制器的请求。
 */
static void App_VehicleSendNoRequest(AppVehicleCanTarget_t target)
{
    uint8_t data[8] = {0};

    if (target == APP_VEHICLE_CAN_TARGET_ACU)
    {
        (void)APP_CAN_SendIVI_ACU(data);
    }
    else
    {
        (void)APP_CAN_SendIVI_BCM(data);
    }
}

/**
 * @brief  判断指定 CAN 目标是否还有 pending 指令。
 * @param  target CAN 控制目标。
 * @return 1 = 仍有 pending, 0 = 没有 pending。
 */
static uint8_t App_VehicleHasPendingTarget(AppVehicleCanTarget_t target)
{
    uint8_t i;

    for (i = 0U; i < APP_VEHICLE_PENDING_MAX; i++)
    {
        if ((s_vehiclePending[i].used != 0U) &&
            (s_vehiclePending[i].pMap != NULL) &&
            (s_vehiclePending[i].pMap->target == target))
        {
            return 1U;
        }
    }

    return 0U;
}

/**
 * @brief  查找空闲 pending 槽位。
 * @return 空闲槽位指针；无空闲槽位时返回 NULL。
 */
static AppVehiclePending_t *App_VehicleFindFreePending(void)
{
    uint8_t i;

    for (i = 0U; i < APP_VEHICLE_PENDING_MAX; i++)
    {
        if (s_vehiclePending[i].used == 0U)
        {
            return &s_vehiclePending[i];
        }
    }

    return NULL;
}

/**
 * @brief  回复语音模块并清理 pending 槽位。
 * @param  pPending pending 槽位。
 * @param  result   本条命令的执行结果。
 * @retval 无。
 */
static void App_VehicleFinishPending(AppVehiclePending_t *pPending,
                                     AppVehicleResult_t result)
{
    AppVehicleCanTarget_t target;

    if ((pPending == NULL) || (pPending->used == 0U) || (pPending->pMap == NULL))
    {
        return;
    }

    target = pPending->pMap->target;
    Voice_SendFrame(pPending->voiceCmd, App_VehicleVoiceRespCode(result), NULL);

    pPending->used = 0U;
    pPending->pMap = NULL;

    if (App_VehicleHasPendingTarget(target) == 0U)
    {
        App_VehicleSendNoRequest(target);
    }
}

/**
 * @brief  等待指定类型的 CAN 反馈报文刷新。
 * @param  pRxTick   指向对应反馈报文更新时间戳，例如 bcm_tbox1_tick / acu_ivi_tick / srcm_tick。
 * @param  oldTick   发送控制命令前记录的旧时间戳。
 * @param  timeoutMs 最长等待时间，单位 ms。
 * @return pdTRUE = 等到新的反馈报文, pdFALSE = 超时未等到。
 * @note   该函数用于避免读取旧反馈状态：
 *         发送命令前先保存 oldTick，发送后等待 CAN_RxTask 解析新反馈帧并更新 tick。
 */
static BaseType_t App_VehicleWaitRxUpdate(const uint32_t *pRxTick,
                                          uint32_t oldTick,
                                          uint32_t timeoutMs)
{
    TickType_t startTick;     /* 进入等待时的系统 tick，用于计算超时。 */

    if (pRxTick == NULL)
    {
        return pdFALSE;
    }

    startTick = xTaskGetTickCount();

    while ((xTaskGetTickCount() - startTick) < pdMS_TO_TICKS(timeoutMs))
    {
        if (*pRxTick != oldTick)
        {
            return pdTRUE;
        }

        vTaskDelay(pdMS_TO_TICKS(10U));
    }

    return pdFALSE;
}

/**
 * @brief  按表项发送 CAN 命令，并等待对应反馈确认。
 */
static AppVehicleResult_t App_VehicleExecuteByMap(const AppVehicleCmdMap_t *pMap)
{
    uint32_t oldTick;
    const uint32_t *pRxTick;
    int sendRet;

    if (pMap == NULL)
    {
        return APP_VEHICLE_RESULT_FAIL;
    }

    if (pMap->target == APP_VEHICLE_CAN_TARGET_BCM)
    {
        pRxTick = &g_vehicleStatus.bcm_tbox1_tick;
        oldTick = g_vehicleStatus.bcm_tbox1_tick;
    }
    else if (pMap->target == APP_VEHICLE_CAN_TARGET_ACU)
    {
        pRxTick = &g_vehicleStatus.acu_ivi_tick;
        oldTick = g_vehicleStatus.acu_ivi_tick;
    }
    else
    {
        pRxTick = &g_vehicleStatus.srcm_tick;
        oldTick = g_vehicleStatus.srcm_tick;
    }

    sendRet = App_VehicleSendRequestByMap(pMap);

    if (sendRet != 0)
    {
        return APP_VEHICLE_RESULT_FAIL;
    }

    if (App_VehicleWaitRxUpdate(pRxTick, oldTick, APP_VEHICLE_RX_WAIT_MS) != pdTRUE)
    {
        return APP_VEHICLE_RESULT_FAIL;
    }

    if (App_VehicleReadStatusU8(pMap->statusOffset) == pMap->expected)
    {
        return APP_VEHICLE_RESULT_OK;
    }

    return APP_VEHICLE_RESULT_FAIL;
}

/* ===== 初始化 ===== */

/**
 * @brief  初始化车辆控制模块。
 * @return pdPASS = 成功, pdFAIL = 队列创建失败。
 * @note   必须在 VehicleTask 创建前调用。
 */
BaseType_t App_VehicleInit(void)
{
    s_vehicleReqQueue = xQueueCreate(APP_VEHICLE_REQ_QUEUE_LENGTH,
                                     sizeof(AppVehicleRequest_t));
    if (s_vehicleReqQueue == NULL)
    {
        LOG_ERR("Vehicle queue create failed");
        return pdFAIL;
    }

    LOG_INFO("Vehicle module initialized");
    return pdPASS;
}

/* ===== 执行命令 ===== */

/**
 * @brief  执行车辆控制命令。
 * @param  cmd 内部统一命令编号。
 * @return 执行结果。
 * @note
 */
AppVehicleResult_t App_VehicleExecute(AppVehicleCommand_t cmd)
{
    const AppVehicleCmdMap_t *pMap;      /* 命令映射表项。 */

    if (App_VehicleIsImmediateOkCmd(cmd) != 0U)
    {
        return APP_VEHICLE_RESULT_OK;
    }

    pMap = App_VehicleFindCmdMap(cmd);
    if (pMap == NULL)
    {
        LOG_WARN("Unknown vehicle cmd: %d", cmd);
        return APP_VEHICLE_RESULT_FAIL;
    }

    return App_VehicleExecuteByMap(pMap);
}

/**
 * @brief  获取车辆控制接收队列句柄
 * @return 队列句柄，供语音任务阻塞等待
 */
QueueHandle_t Vehicle_GetRxQueue(void)
{
    return s_vehicleReqQueue;
}

/* ===== 统一控制入口 ===== */

/**
 * @brief  统一控制请求入口（同步等待结果）。
 * @param  cmd          控制命令。
 * @param  timeoutTicks 等待超时 tick 数。
 * @return 执行结果。
 * @note   流程：
 *         1. 清掉旧通知（防脏读）
 *         2. 打包请求（cmd + 当前任务句柄）→ 发到队列
 *         3. 阻塞等通知 ← VehicleTask 执行完发来
 *         4. 超时或拿到结果 → 返回
 */
AppVehicleResult_t App_ControlRequest(AppVehicleCommand_t cmd,
                                      TickType_t timeoutTicks)
{
    AppVehicleRequest_t req;
    uint32_t notifyValue = 0U;

    if (s_vehicleReqQueue == NULL)
    {
        return APP_VEHICLE_RESULT_NOT_READY;
    }

    /** @brief  清掉旧通知，防止读到上一次残留结果。 */
    xTaskNotifyWait(0U, 0xFFFFFFFFUL, &notifyValue, 0U);

    req.cmd       = cmd;
    req.requester = xTaskGetCurrentTaskHandle();
    req.voiceCmd  = 0U;

    /** @brief  发送请求到 VehicleTask。 */
    if (xQueueSend(s_vehicleReqQueue,
                   &req,
                   pdMS_TO_TICKS(APP_VEHICLE_REQ_SEND_TIMEOUT_MS)) != pdPASS)
    {
        LOG_WARN("Vehicle req queue full, cmd: %d", cmd);
        return APP_VEHICLE_RESULT_TIMEOUT;
    }

    /** @brief  阻塞等待 VehicleTask 通知。 */
    if (xTaskNotifyWait(0U,
                        0xFFFFFFFFUL,
                        &notifyValue,
                        timeoutTicks) != pdPASS)
    {
        LOG_WARN("Vehicle req timeout, cmd: %d", cmd);
        return APP_VEHICLE_RESULT_TIMEOUT;
    }

    return (AppVehicleResult_t)notifyValue;
}

/**
 * @brief  提交语音异步控制请求。
 * @param  cmd      内部统一车辆控制命令。
 * @param  voiceCmd 语音模块原始命令码。
 * @return OK = 请求已入队或已直接处理, 其他 = 请求提交失败。
 */
AppVehicleResult_t App_ControlSubmit(AppVehicleCommand_t cmd,
                                     uint8_t voiceCmd)
{
    AppVehicleRequest_t req;

    if (s_vehicleReqQueue == NULL)
    {
        return APP_VEHICLE_RESULT_NOT_READY;
    }

    req.cmd       = cmd;
    req.requester = NULL;
    req.voiceCmd  = voiceCmd;

    if (xQueueSend(s_vehicleReqQueue,
                   &req,
                   pdMS_TO_TICKS(APP_VEHICLE_REQ_SEND_TIMEOUT_MS)) != pdPASS)
    {
        LOG_WARN("Vehicle async req queue full, cmd: %d", cmd);
        return APP_VEHICLE_RESULT_TIMEOUT;
    }

    return APP_VEHICLE_RESULT_OK;
}

/**
 * @brief  VehicleTask 处理一条车辆请求。
 * @param  pReq 队列收到的车辆请求。
 * @retval 无。
 */
void App_VehicleProcessRequest(const AppVehicleRequest_t *pReq)
{
    AppVehicleResult_t result;
    AppVehiclePending_t *pPending;
    const AppVehicleCmdMap_t *pMap;

    if (pReq == NULL)
    {
        return;
    }

    if (pReq->requester != NULL)
    {
        result = App_VehicleExecute(pReq->cmd);
        xTaskNotify(pReq->requester, (uint32_t)result, eSetValueWithOverwrite);
        return;
    }

    if (App_VehicleIsImmediateOkCmd(pReq->cmd) != 0U)
    {
        Voice_SendFrame(pReq->voiceCmd, VOICE_RESP_OK, NULL);
        return;
    }

    pMap = App_VehicleFindCmdMap(pReq->cmd);
    if (pMap == NULL)
    {
        LOG_WARN("Unknown async vehicle cmd: %d", pReq->cmd);
        Voice_SendFrame(pReq->voiceCmd, VOICE_RESP_FAIL, NULL);
        return;
    }

    pPending = App_VehicleFindFreePending();
    if (pPending == NULL)
    {
        LOG_WARN("Vehicle pending table full, cmd: %d", pReq->cmd);
        Voice_SendFrame(pReq->voiceCmd, VOICE_RESP_FAIL, NULL);
        return;
    }

    pPending->oldRxTick = App_VehicleGetRxTickByMap(pMap);

    if (App_VehicleSendRequestByMap(pMap) != 0)
    {
        Voice_SendFrame(pReq->voiceCmd, VOICE_RESP_FAIL, NULL);
        return;
    }

    pPending->voiceCmd   = pReq->voiceCmd;
    pPending->startTick  = xTaskGetTickCount();
    pPending->pMap       = pMap;
    pPending->used       = 1U;
}

/**
 * @brief  VehicleTask 周期轮询语音 pending 表。
 * @retval 无。
 */
void App_VehiclePollPending(void)
{
    uint8_t i;
    TickType_t nowTick;
    const AppVehicleCmdMap_t *pMap;

    nowTick = xTaskGetTickCount();

    for (i = 0U; i < APP_VEHICLE_PENDING_MAX; i++)
    {
        if ((s_vehiclePending[i].used == 0U) || (s_vehiclePending[i].pMap == NULL))
        {
            continue;
        }

        pMap = s_vehiclePending[i].pMap;

        if ((App_VehicleGetRxTickByMap(pMap) != s_vehiclePending[i].oldRxTick) &&
            (App_VehicleReadStatusU8(pMap->statusOffset) == pMap->expected))
        {
            App_VehicleFinishPending(&s_vehiclePending[i], APP_VEHICLE_RESULT_OK);
        }
        else if ((nowTick - s_vehiclePending[i].startTick) >=
                 pdMS_TO_TICKS(APP_VEHICLE_PENDING_TIMEOUT_MS))
        {
            LOG_WARN("Vehicle pending timeout, voice cmd: 0x%02X",
                     s_vehiclePending[i].voiceCmd);
            App_VehicleFinishPending(&s_vehiclePending[i], APP_VEHICLE_RESULT_TIMEOUT);
        }
    }
}
