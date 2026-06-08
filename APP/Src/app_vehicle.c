/**
 * @file    app_vehicle.c
 * @brief   车辆控制模块实现
 * @details 统一接收所有控制请求（语音/按键/BLE），
 *          集中执行安全判断、CAN 发送、结果返回。
 *
 * 请求链路：
 *   上层 → App_ControlRequest() → 队列 → VehicleTask → 执行 → 任务通知 → 上层
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

/** RX 反馈等待时间（ms），给 BCM/ACU/SRCM 至少一个周期的反应时间 */
#define APP_VEHICLE_RX_WAIT_MS              150U

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
 * @brief  从车辆状态结构体中读取一个 uint8_t 状态字段。
 */
static uint8_t App_VehicleReadStatusU8(uint16_t offset)
{
    const uint8_t *pBase = (const uint8_t *)&g_vehicleStatus;

    return *(const uint8_t *)(pBase + offset);
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
    uint8_t data[8] = {0};
    uint32_t oldTick;
    const uint32_t *pRxTick;
    int sendRet;

    if (pMap == NULL)
    {
        return APP_VEHICLE_RESULT_FAIL;
    }

    data[pMap->txByte] = pMap->txValue;

    if (pMap->target == APP_VEHICLE_CAN_TARGET_BCM)
    {
        pRxTick = &g_vehicleStatus.bcm_tbox1_tick;
        oldTick = g_vehicleStatus.bcm_tbox1_tick;
        sendRet = APP_CAN_SendIVI_BCM(data);
    }
    else if (pMap->target == APP_VEHICLE_CAN_TARGET_ACU)
    {
        pRxTick = &g_vehicleStatus.acu_ivi_tick;
        oldTick = g_vehicleStatus.acu_ivi_tick;
        sendRet = APP_CAN_SendIVI_ACU(data);
    }
    else
    {
        pRxTick = &g_vehicleStatus.srcm_tick;
        oldTick = g_vehicleStatus.srcm_tick;
        sendRet = APP_CAN_SendIVI_BCM(data);
    }

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
