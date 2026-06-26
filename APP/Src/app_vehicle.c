/**
 * @file    app_vehicle.c
 * @brief   车辆控制模块实现
 * @details 统一接收所有控制请求（语音/按键/BLE），
 *          集中执行安全判断、CAN 发送、结果返回。
 *
 * 请求链路：
 *   同步：上层 → App_ControlRequest() → 队列 → VehicleTask → 执行 → 任务通知 → 上层
 *   异步：上层 → App_ControlSubmit()  → 队列 → VehicleTask → 执行 → 任务通知 → 上层轮询
 */

#include "app_vehicle.h"
#include "app_debug.h"
#include "queue.h"
#include "app_can_proto.h"
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

#define APP_VEHICLE_REQ_QUEUE_LENGTH        4U
#define APP_VEHICLE_REQ_SEND_TIMEOUT_MS     20U

/** RX 反馈确认时间（ms），3 秒内持续等待目标状态位达到期望值。 */
#define APP_VEHICLE_STATUS_CONFIRM_MS       3000U

static QueueHandle_t s_vehicleReqQueue = NULL;

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

    { APP_VEHICLE_CMD_SUNROOF_FAN_ON,    APP_VEHICLE_CAN_TARGET_SRCM, 5U, CAN_SET_3BIT(CAN_BCM_SUNROOF_FAN_LEVEL2, CAN_BCM_SUNROOF_FAN_POS), offsetof(VehicleStatus_t, sr_fan_level),             2U },
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
 * @brief  从车辆状态结构体中读取一个 uint8_t 状态字段。
 */
static uint8_t App_VehicleReadStatusU8(uint16_t offset)
{
    const uint8_t *pBase = (const uint8_t *)&g_vehicleStatus;

    return *(const uint8_t *)(pBase + offset);
}

/**
 * @brief  将控制帧数据域清为“无请求”。
 * @param  data 8 字节 CAN 数据域。
 * @retval 无。
 */
static void App_VehicleClearRequestData(uint8_t *data)
{
    uint8_t i;

    if (data == NULL)
    {
        return;
    }

    for (i = 0U; i < 8U; i++)
    {
        data[i] = 0U;
    }
}

/**
 * @brief  为复合语音命令补充关联控制请求。
 * @param  pMap 命令映射表项。
 * @param  data 8 字节 CAN 数据域，已填入主请求字段。
 * @retval 无。
 * @note   某些语音词条在协议上需要同时下发多个请求位，例如关闭大灯同时关闭远光、
 *         洗涤同时打开低速雨刮、空调模式同时带默认风速。
 */
static void App_VehicleApplyLinkedRequest(const AppVehicleCmdMap_t *pMap,
                                          uint8_t *data)
{
    if ((pMap == NULL) || (data == NULL))
    {
        return;
    }

    switch (pMap->cmd)
    {
        case APP_VEHICLE_CMD_LOW_BEAM_OFF:
            data[3] |= CAN_SET_2BIT(CAN_BCM_LIGHT_OFF, CAN_BCM_HIGH_BEAM_POS);
            break;

        case APP_VEHICLE_CMD_WASHER_ON:
            data[6] |= CAN_SET_3BIT(CAN_BCM_WIPER_LOW, CAN_BCM_FRONT_WIPER_POS);
            break;

        case APP_VEHICLE_CMD_AC_HEAT_ON:
        case APP_VEHICLE_CMD_AC_COOL_ON:
            data[0] = CAN_ACU_FAN_LEVEL2;
            break;

        case APP_VEHICLE_CMD_AC_OFF:
            data[0] = CAN_ACU_FAN_OFF;
            break;

        default:
            break;
    }
}

/**
 * @brief  检查主状态位和关联状态位是否都达到期望。
 * @param  pMap 命令映射表项。
 * @return 1 = 状态满足命令成功条件，0 = 未满足。
 */
static uint8_t App_VehicleIsExpectedStatus(const AppVehicleCmdMap_t *pMap)
{
    if (pMap == NULL)
    {
        return 0U;
    }

    if (App_VehicleReadStatusU8(pMap->statusOffset) != pMap->expected)
    {
        return 0U;
    }

    switch (pMap->cmd)
    {
        case APP_VEHICLE_CMD_LOW_BEAM_OFF:
            return (uint8_t)(g_vehicleStatus.high_beam_light_status == 0U);

        case APP_VEHICLE_CMD_WASHER_ON:
            return (uint8_t)(g_vehicleStatus.front_wiper_status == 2U);

        case APP_VEHICLE_CMD_AC_HEAT_ON:
        case APP_VEHICLE_CMD_AC_COOL_ON:
            return (uint8_t)(g_vehicleStatus.ac_fan_gear == 2U);

        case APP_VEHICLE_CMD_AC_OFF:
            return (uint8_t)(g_vehicleStatus.ac_fan_gear == 0U);

        default:
            return 1U;
    }
}

/**
 * @brief  根据命令目标发送 IVI 控制帧。
 * @param  pMap 命令映射表项，用于选择 IVI_BCM 或 IVI_ACU。
 * @param  data 8 字节 CAN 数据域，调用方负责填入请求值或全 0 清请求值。
 * @return 0 = 发送成功，其他 = 发送失败。
 */
static int App_VehicleSendByMapTarget(const AppVehicleCmdMap_t *pMap,
                                      uint8_t *data)
{
    if ((pMap == NULL) || (data == NULL))
    {
        return -1;
    }

    if (pMap->target == APP_VEHICLE_CAN_TARGET_ACU)
    {
        return APP_CAN_SendIVI_ACU(data);
    }

    return APP_CAN_SendIVI_BCM(data);
}

/**
 * @brief  获取命令对应反馈报文的更新时间戳指针。
 * @param  pMap 命令映射表项。
 * @return 对应反馈 tick 指针；参数异常时返回 NULL。
 */
static const uint32_t *App_VehicleGetRxTickPtr(const AppVehicleCmdMap_t *pMap)
{
    if (pMap == NULL)
    {
        return NULL;
    }

    if (pMap->target == APP_VEHICLE_CAN_TARGET_BCM)
    {
        return &g_vehicleStatus.bcm_tbox1_tick;
    }
    else if (pMap->target == APP_VEHICLE_CAN_TARGET_ACU)
    {
        return &g_vehicleStatus.acu_ivi_tick;
    }
    else
    {
        return &g_vehicleStatus.srcm_tick;
    }
}

/**
 * @brief  3 秒内持续等待目标反馈状态位达到期望值。
 * @param  pMap      命令映射表项，提供目标状态字段和期望值。
 * @param  pRxTick   对应反馈报文更新时间戳指针。
 * @param  oldTick   发送控制命令前记录的旧时间戳，用于忽略旧反馈。
 * @param  timeoutMs 最长等待时间，单位 ms。
 * @return pdTRUE = 目标状态位在超时前达到期望值，pdFALSE = 超时未达到。
 * @note   等待期间 CAN 接收仍由 CAN_RxTask 异步解析；本函数只观察缓存状态，
 *         无关 ID 不影响结果，同类反馈状态暂未到位时继续等待。
 */
static BaseType_t App_VehicleWaitExpectedStatus(const AppVehicleCmdMap_t *pMap,
                                                const uint32_t *pRxTick,
                                                uint32_t oldTick,
                                                uint32_t timeoutMs)
{
    TickType_t startTick;     /* 进入等待时的系统 tick，用于计算 3 秒确认窗口。 */
    uint32_t lastTick;        /* 已处理过的反馈 tick，避免重复判断同一帧。 */

    if ((pMap == NULL) || (pRxTick == NULL))
    {
        return pdFALSE;
    }

    startTick = xTaskGetTickCount();
    lastTick = oldTick;

    while ((xTaskGetTickCount() - startTick) < pdMS_TO_TICKS(timeoutMs))
    {
        if (*pRxTick != lastTick)
        {
            lastTick = *pRxTick;
            if (App_VehicleIsExpectedStatus(pMap) != 0U)
            {
                return pdTRUE;
            }
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
    uint8_t data[8] = {0};
    uint32_t oldTick;
    const uint32_t *pRxTick;
    int sendRet;
    AppVehicleResult_t result;

    if (pMap == NULL)
    {
        return APP_VEHICLE_RESULT_FAIL;
    }

    data[pMap->txByte] = pMap->txValue;
    App_VehicleApplyLinkedRequest(pMap, data);
    pRxTick = App_VehicleGetRxTickPtr(pMap);
    oldTick = (pRxTick != NULL) ? *pRxTick : 0U;
    sendRet = App_VehicleSendByMapTarget(pMap, data);

    if (sendRet != 0)
    {
        return APP_VEHICLE_RESULT_FAIL;
    }

    if (App_VehicleWaitExpectedStatus(pMap,
                                      pRxTick,
                                      oldTick,
                                      APP_VEHICLE_STATUS_CONFIRM_MS) == pdTRUE)
    {
        result = APP_VEHICLE_RESULT_OK;
    }
    else
    {
        result = APP_VEHICLE_RESULT_FAIL;
    }

    App_VehicleClearRequestData(data);
    if (App_VehicleSendByMapTarget(pMap, data) != 0)
    {
        LOG_WARN("Vehicle clear request failed, cmd: %d", pMap->cmd);
        return APP_VEHICLE_RESULT_FAIL;
    }

    return result;
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
            return APP_VEHICLE_RESULT_OK;

        default:
            pMap = App_VehicleFindCmdMap(cmd);
            if (pMap == NULL)
            {
                LOG_WARN("Unknown vehicle cmd: %d", cmd);
                return APP_VEHICLE_RESULT_FAIL;
            }

            return App_VehicleExecuteByMap(pMap);
    }
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
 * @brief  异步提交控制请求，由 VehicleTask 执行完成后通知 requester。
 * @param  cmd       控制命令。
 * @param  requester 接收执行结果通知的任务句柄。
 * @return OK = 已入队，TIMEOUT = 队列满，NOT_READY = 模块未初始化。
 * @note   该接口不阻塞等待执行结果，适合语音任务在 3 秒确认窗口内继续处理新语音帧。
 */
AppVehicleResult_t App_ControlSubmit(AppVehicleCommand_t cmd,
                                     TaskHandle_t requester)
{
    AppVehicleRequest_t req;

    if (s_vehicleReqQueue == NULL)
    {
        return APP_VEHICLE_RESULT_NOT_READY;
    }

    req.cmd       = cmd;
    req.requester = requester;

    if (xQueueSend(s_vehicleReqQueue,
                   &req,
                   pdMS_TO_TICKS(APP_VEHICLE_REQ_SEND_TIMEOUT_MS)) != pdPASS)
    {
        LOG_WARN("Vehicle req queue full, cmd: %d", cmd);
        return APP_VEHICLE_RESULT_TIMEOUT;
    }

    return APP_VEHICLE_RESULT_OK;
}
