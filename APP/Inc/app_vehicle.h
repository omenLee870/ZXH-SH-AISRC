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
    APP_VEHICLE_CMD_PARKING_LIGHT_ON,    /* 打开示廓灯。               */
    APP_VEHICLE_CMD_PARKING_LIGHT_OFF,   /* 关闭示廓灯。               */
    APP_VEHICLE_CMD_REAR_FOG_ON,         /* 打开后雾灯。               */
    APP_VEHICLE_CMD_REAR_FOG_OFF,        /* 关闭后雾灯。               */
    APP_VEHICLE_CMD_HAZARD_ON,           /* 打开双闪。                 */
    APP_VEHICLE_CMD_HAZARD_OFF,          /* 关闭双闪。                 */
    APP_VEHICLE_CMD_TRUNK_UNLOCK,        /* 打开后备箱。               */
    APP_VEHICLE_CMD_MIRROR_UNFOLD,       /* 展开后视镜。               */
    APP_VEHICLE_CMD_MIRROR_FOLD,         /* 折叠后视镜。               */
    APP_VEHICLE_CMD_READING_LIGHT_ON,    /* 打开室内灯。               */
    APP_VEHICLE_CMD_READING_LIGHT_OFF,   /* 关闭室内灯。               */
    APP_VEHICLE_CMD_SUNROOF_FAN_ON,      /* 打开风扇。                 */
    APP_VEHICLE_CMD_SUNROOF_FAN_OFF,     /* 关闭风扇。                 */
    APP_VEHICLE_CMD_SUNROOF_FAN_L1,      /* 风扇一档。                 */
    APP_VEHICLE_CMD_SUNROOF_FAN_L2,      /* 风扇二档。                 */
    APP_VEHICLE_CMD_SUNROOF_FAN_L3,      /* 风扇三档。                 */
    APP_VEHICLE_CMD_AC_HEAT_ON,          /* 空调制热。                 */
    APP_VEHICLE_CMD_AC_COOL_ON,          /* 空调制冷。                 */
    APP_VEHICLE_CMD_AC_OFF,              /* 关闭空调。                 */
    APP_VEHICLE_CMD_AC_LEVEL1,           /* 空调一档。                 */
    APP_VEHICLE_CMD_AC_LEVEL2,           /* 空调二档。                 */
    APP_VEHICLE_CMD_AC_LEVEL3,           /* 空调三档。                 */
    APP_VEHICLE_CMD_WEATHER_QUERY,       /* 查询天气。                 */
    APP_VEHICLE_CMD_DATE_QUERY,          /* 查询日期。                 */
    APP_VEHICLE_CMD_TIME_QUERY,          /* 查询时间。                 */
    APP_VEHICLE_CMD_VOLUME_UP,           /* 增大音量。                 */
    APP_VEHICLE_CMD_VOLUME_DOWN,         /* 减小音量。                 */
    APP_VEHICLE_CMD_VOLUME_MAX,          /* 最大音量。                 */
    APP_VEHICLE_CMD_VOLUME_MIN,          /* 最小音量。                 */
    APP_VEHICLE_CMD_DATA_WAKEUP,         /* 数据唤醒。                 */
    APP_VEHICLE_CMD_DATA_EXIT_WAKEUP,    /* 退出唤醒。                 */
    APP_VEHICLE_CMD_15S_EXIT_WAKEUP,     /* 超时退出唤醒。             */

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
 * @note   
 */
AppVehicleResult_t App_VehicleExecute(AppVehicleCommand_t cmd);

#endif /* __APP_VEHICLE_H__ */
