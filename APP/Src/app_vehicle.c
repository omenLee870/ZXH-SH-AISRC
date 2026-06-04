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

#define LOG_TAG "vehicle"

#define APP_VEHICLE_REQ_QUEUE_LENGTH        4U
#define APP_VEHICLE_REQ_SEND_TIMEOUT_MS     20U

static QueueHandle_t s_vehicleReqQueue = NULL;

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
 * @brief  执行车辆控制命令
 * @param  cmd 内部统一命令编号
 * @return 执行结果
 * @note   根据命令类型，组装雷迈协议 IVI_BCM 帧并通过 CAN 总线发出。
 *
 *         映射关系（内部命令 → CAN 信号）：
 *         - 转向灯 → IVI_BCM Data[5] : LH_Turn/RH_Turn (2bit)
 *         - 近光灯 → IVI_BCM Data[3] : Low_Beam (2bit)
 *         - 远光灯 → IVI_BCM Data[3] : High_Beam (2bit)
 *         - 雨刮   → IVI_BCM Data[6] : Front_Wiper (3bit)
 *         - 洗涤   → IVI_BCM Data[2] : Wiper_Wash (2bit)
 *
 *         信号值约定（雷迈协议）：
 *         0x0 = 无请求（默认）, 0x1 = ON/开启, 0x2 = OFF/关闭
 *         雨刮多级: 0x1=关, 0x2=间歇, 0x3=低速, 0x4=高速
 *
 *         TODO: 后续增加安全条件判断（车速 > 0 时禁止某些操作）
 *         TODO: 当前是事件驱动（每次操作发一帧），非 100ms 周期模式
 */
AppVehicleResult_t App_VehicleExecute(AppVehicleCommand_t cmd)
{
uint8_t canData[8] = {0};           /**< 初始化为全 0x00（无请求） */

    switch (cmd)
    {
        case APP_VEHICLE_CMD_WAKEUP:
        case APP_VEHICLE_CMD_WAKE_WORD:
            return APP_VEHICLE_RESULT_OK;

        /* ---------- 转向灯 → Data[5] ---------- */
        case APP_VEHICLE_CMD_LEFT_TURN_ON:
            canData[5] |= CAN_SET_2BIT(CAN_BCM_TURN_ON, CAN_BCM_LH_TURN_POS);
            break;
        case APP_VEHICLE_CMD_LEFT_TURN_OFF:
            canData[5] |= CAN_SET_2BIT(CAN_BCM_TURN_OFF, CAN_BCM_LH_TURN_POS);
            break;
        case APP_VEHICLE_CMD_RIGHT_TURN_ON:
            canData[5] |= CAN_SET_2BIT(CAN_BCM_TURN_ON, CAN_BCM_RH_TURN_POS);
            break;
        case APP_VEHICLE_CMD_RIGHT_TURN_OFF:
            canData[5] |= CAN_SET_2BIT(CAN_BCM_TURN_OFF, CAN_BCM_RH_TURN_POS);
            break;

        /* ---------- 近光灯 → Data[3] ---------- */
        case APP_VEHICLE_CMD_LOW_BEAM_ON:
            canData[3] |= CAN_SET_2BIT(CAN_BCM_LIGHT_ON, CAN_BCM_LOW_BEAM_POS);
            break;
        case APP_VEHICLE_CMD_LOW_BEAM_OFF:
            canData[3] |= CAN_SET_2BIT(CAN_BCM_LIGHT_OFF, CAN_BCM_LOW_BEAM_POS);
            break;

        /* ---------- 远光灯 → Data[3] ---------- */
        case APP_VEHICLE_CMD_HIGH_BEAM_ON:
            canData[3] |= CAN_SET_2BIT(CAN_BCM_LIGHT_ON, CAN_BCM_HIGH_BEAM_POS);
            break;
        case APP_VEHICLE_CMD_HIGH_BEAM_OFF:
            canData[3] |= CAN_SET_2BIT(CAN_BCM_LIGHT_OFF, CAN_BCM_HIGH_BEAM_POS);
            break;

        /* ---------- 前雨刮 → Data[6] ---------- */
        case APP_VEHICLE_CMD_WIPER_OFF:
            canData[6] |= CAN_SET_3BIT(CAN_BCM_WIPER_OFF, CAN_BCM_FRONT_WIPER_POS);
            break;
        case APP_VEHICLE_CMD_WIPER_INTERVAL:
            canData[6] |= CAN_SET_3BIT(CAN_BCM_WIPER_INTERVAL, CAN_BCM_FRONT_WIPER_POS);
            break;
        case APP_VEHICLE_CMD_WIPER_ON:
            canData[6] |= CAN_SET_3BIT(CAN_BCM_WIPER_LOW, CAN_BCM_FRONT_WIPER_POS);
            break;
        case APP_VEHICLE_CMD_WIPER_HIGH:
            canData[6] |= CAN_SET_3BIT(CAN_BCM_WIPER_HIGH, CAN_BCM_FRONT_WIPER_POS);
            break;

        /* ---------- 洗涤 → Data[2] ---------- */
        case APP_VEHICLE_CMD_WASHER_ON:
            canData[2] |= CAN_SET_2BIT(CAN_BCM_WASHER_ON, CAN_BCM_WASHER_POS);
            break;

        default:
            LOG_WARN("Unknown vehicle cmd: %d", cmd);
            return APP_VEHICLE_RESULT_FAIL;
    }

    if (APP_CAN_SendIVI_BCM(canData) != 0)
    {
        LOG_ERR("CAN BCM send failed, cmd=%d", cmd);
        return APP_VEHICLE_RESULT_FAIL;
    }

    vTaskDelay(pdMS_TO_TICKS(10U));
    return APP_VEHICLE_RESULT_OK;
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
