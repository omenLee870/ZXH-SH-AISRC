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
 * @brief  执行车辆控制命令。
 * @param  cmd 内部统一命令编号。
 * @return 执行结果。
 * @note   TODO: CAN 模块完成后，替换为：
 *         1. 安全条件判断（车速、档位等）
 *         2. CAN 报文发送
 *         3. 等待 CAN 应答或超时
 *         当前暂时 10ms 后返回成功（模拟）。
 */
AppVehicleResult_t App_VehicleExecute(AppVehicleCommand_t cmd)
{
    switch (cmd)
    {
        case APP_VEHICLE_CMD_WAKEUP:
            return APP_VEHICLE_RESULT_OK;
        case APP_VEHICLE_CMD_WAKE_WORD:
            return APP_VEHICLE_RESULT_OK;

        case APP_VEHICLE_CMD_LEFT_TURN_ON:
        case APP_VEHICLE_CMD_LEFT_TURN_OFF:
        case APP_VEHICLE_CMD_RIGHT_TURN_ON:
        case APP_VEHICLE_CMD_RIGHT_TURN_OFF:
        case APP_VEHICLE_CMD_LOW_BEAM_ON:
        case APP_VEHICLE_CMD_LOW_BEAM_OFF:
        case APP_VEHICLE_CMD_HIGH_BEAM_ON:
        case APP_VEHICLE_CMD_HIGH_BEAM_OFF:
        case APP_VEHICLE_CMD_WIPER_OFF:
        case APP_VEHICLE_CMD_WIPER_INTERVAL:
        case APP_VEHICLE_CMD_WIPER_ON:
        case APP_VEHICLE_CMD_WIPER_HIGH:
        case APP_VEHICLE_CMD_WASHER_ON:
            /*
             * TODO: 替换为：安全判断 → CAN 发送 → 等反馈
             */
            vTaskDelay(pdMS_TO_TICKS(10U));
            return APP_VEHICLE_RESULT_OK;

        default:
            LOG_WARN("Unknown vehicle cmd: %d", cmd);
            return APP_VEHICLE_RESULT_FAIL;
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
