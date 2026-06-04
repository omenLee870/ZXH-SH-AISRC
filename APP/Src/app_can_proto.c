/**
 * @file    app_can_proto.c
 * @brief   雷迈通讯协议层实现
 * @details 负责将内部车辆控制命令转换为符合雷迈协议（IVI A2.1）的 CAN 帧，
 *          并调用驱动层 APP_CAN_Send() 发出。
 *
 *          数据流：
 *          app_vehicle.c (App_VehicleExecute)
 *            → APP_CAN_BuildBCM_Frame()      ← 协议层：组帧
 *            → APP_CAN_SendIVI_BCM()         ← 协议层：封装发送
 *              → APP_CAN_Send(id, data, 8)   ← 驱动层：硬件发送
 *
 *          参考文档：
 *          E:\...\资料\雷迈通讯协议（IVI）A2.1.xlsx
 */

#include "app_can_proto.h"
#include "app_can.h"
#include "app_debug.h"

/**
 * @brief  构建并发送 IVI_BCM 报文
 */
int APP_CAN_SendIVI_BCM(uint8_t *data)
{
    LOG_DBG("BCM TX: %02X %02X %02X %02X %02X %02X %02X %02X",
            data[0], data[1], data[2], data[3],
            data[4], data[5], data[6], data[7]);

    return APP_CAN_Send(CAN_ID_IVI_BCM, data, 8);
}

/**
 * @brief  构建并发送 IVI_MCU 报文（驾驶模式，预留）
 */
int APP_CAN_SendIVI_MCU(uint8_t *data)
{
    LOG_DBG("MCU TX: %02X %02X %02X %02X %02X %02X %02X %02X",
            data[0], data[1], data[2], data[3],
            data[4], data[5], data[6], data[7]);

    return APP_CAN_Send(CAN_ID_IVI_MCU, data, 8);
}
