/**
 * @file    app_vehicle.h
 * @brief   车辆控制模块
 * @details 提供统一的控制请求入口 App_ControlRequest()。
 *          语音、按键、BLE 等任何上层任务都通过此接口发起车辆控制。
 *          内部通过 FreeRTOS 队列 + 任务通知实现同步等待。
 */

#ifndef __APP_VEHICLE_H__
#define __APP_VEHICLE_H__

#include "FreeRTOS.h"
#include "task.h"
#include <Queue.h>
#include <stdint.h>

/* ===== 车辆控制命令（内部统一编号） ===== */
typedef enum
{
    APP_VEHICLE_CMD_WAKE_WORD = 0,      /* 你好盛昊。                 */
    APP_VEHICLE_CMD_WASHER_ON,          /* 打开雨刮喷水。             */
    APP_VEHICLE_CMD_LEFT_TURN_ON,       /* 打开左转灯。               */
    APP_VEHICLE_CMD_LEFT_TURN_OFF,      /* 关闭左转灯。               */
    APP_VEHICLE_CMD_RIGHT_TURN_ON,      /* 打开右转灯。               */
    APP_VEHICLE_CMD_RIGHT_TURN_OFF,     /* 关闭右转灯。               */
    APP_VEHICLE_CMD_LOW_BEAM_ON,        /* 打开大灯/近光灯。          */
    APP_VEHICLE_CMD_LOW_BEAM_OFF,       /* 关闭大灯。                 */
    APP_VEHICLE_CMD_HIGH_BEAM_ON,       /* 打开远光灯。               */
    APP_VEHICLE_CMD_HIGH_BEAM_OFF,      /* 关闭远光灯。               */
    APP_VEHICLE_CMD_WIPER_OFF,          /* 关闭雨刮。                 */
    APP_VEHICLE_CMD_WIPER_INTERVAL,     /* 雨刮间歇。                 */
    APP_VEHICLE_CMD_WIPER_ON,           /* 打开雨刮。                 */
    APP_VEHICLE_CMD_WIPER_HIGH,         /* 雨刮高速。                 */

    APP_VEHICLE_CMD_WAKEUP = 0xF3U      /* 唤醒命令                   */
} AppVehicleCommand_t;

/* ===== 执行结果 ===== */
typedef enum
{
    APP_VEHICLE_RESULT_OK = 0,          /* 执行成功。                 */
    APP_VEHICLE_RESULT_FAIL,            /* 执行失败。                 */
    APP_VEHICLE_RESULT_SAFETY,          /* 安全限制，禁止操作。       */
    APP_VEHICLE_RESULT_TIMEOUT,         /* 等待超时。                 */
    APP_VEHICLE_RESULT_NOT_READY        /* 模块未就绪。               */
} AppVehicleResult_t;

/* ===== 请求结构体（队列元素） ===== */
typedef struct
{
    AppVehicleCommand_t cmd;            /* 要执行的命令。             */
    TaskHandle_t        requester;      /* 发起请求的任务句柄。       */
} AppVehicleRequest_t;

/* ===== 函数声明 ===== */

BaseType_t App_VehicleInit(void);
QueueHandle_t Vehicle_GetRxQueue(void);

/**
 * @brief  统一控制请求入口（同步等待结果）。
 * @param  cmd          车辆控制命令。
 * @param  timeoutTicks 等待超时（FreeRTOS tick 数）。
 * @return 执行结果（OK/FAIL/SAFETY/TIMEOUT）。
 * @note   内部：请求 → 队列 → VehicleTask 执行 → 任务通知 → 返回。
 */
AppVehicleResult_t App_ControlRequest(AppVehicleCommand_t cmd,
                                      TickType_t timeoutTicks);

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
AppVehicleResult_t App_VehicleExecute(AppVehicleCommand_t cmd);

#endif /* __APP_VEHICLE_H__ */
