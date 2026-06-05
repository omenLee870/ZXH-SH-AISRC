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
#include "app_debug.h"
#include "FreeRTOS.h"
#include "task.h"

#define LOG_TAG "can_proto"

/** @brief  全局车辆实时状态，RX 解析函数更新，App_VehicleExecute 读取 */
VehicleStatus_t g_vehicleStatus = {0};

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

/**
 * @brief  构建并发送 IVI_ACU 报文
 */
int APP_CAN_SendIVI_ACU(uint8_t *data)
{
    LOG_DBG("ACU TX: %02X %02X %02X %02X %02X %02X %02X %02X",
            data[0], data[1], data[2], data[3],
            data[4], data[5], data[6], data[7]);

    return APP_CAN_Send(CAN_ID_IVI_ACU, data, 8);
}

/**
 * @brief  解析 BCM_TBOX1 报文 (0x18FF271D)
 * @param  data 8 字节 CAN 帧数据
 * @note   BCM 广播给各节点的车身状态（100ms 周期）。
 *         包含灯光、门锁、车窗、雨刮、后视镜等 30+ 个信号。
 *
 *         信号布局（Motorola LSB，Start Bit → 字节和偏移）：
 *         Data[0]: start[0] 门锁[1] ... 寻车[5] SOS[6]
 *         Data[1]: 车窗禁止[1] 后备箱[2] 双闪[3] 一键启动[4] 碰撞[5]
 *         Data[2]: 门锁检测[0] 四门[1-4] 阅读灯[5] 右转[6] 左转[7]
 *         Data[3]: 近光[0] 远光[1] 后备箱解锁[2] 小灯[3] 洗涤[4] 中控锁[6-7]
 *         Data[4]: 后除霜[0] 后雾灯[2] 倒车灯[3] 喇叭[4] 前雨刮[5-6]
 *         Data[5]: 右前窗[0-1] 右后窗[2-3] 左前窗[4-5] 左后窗[6-7]
 *         Data[6]: 后视镜[0-1] 一键启动状态[2-3]
 */
void CAN_ParseBCM_TBOX1(const uint8_t *data)
{
    /* ===== Data[0] ===== */
    g_vehicleStatus.start_status         = (data[0] >> 0) & 0x01;
    g_vehicleStatus.door_lock_status     = (data[0] >> 1) & 0x01;
    g_vehicleStatus.vehicle_locate_state = (data[0] >> 5) & 0x01;
    g_vehicleStatus.sos_state            = (data[0] >> 6) & 0x01;

    /* ===== Data[1] ===== */
    g_vehicleStatus.window_inhibit_state = (data[1] >> 1) & 0x01;
    g_vehicleStatus.trunk_status_monitor = (data[1] >> 2) & 0x01;
    g_vehicleStatus.hazard_light_switch  = (data[1] >> 3) & 0x01;
    g_vehicleStatus.one_click_start_in   = (data[1] >> 4) & 0x01;
    g_vehicleStatus.collision_detection  = (data[1] >> 5) & 0x01;

    /* ===== Data[2] ===== */
    g_vehicleStatus.door_lock_status_mon = (data[2] >> 0) & 0x01;
    g_vehicleStatus.fl_door_status       = (data[2] >> 1) & 0x01;
    g_vehicleStatus.rf_door_status       = (data[2] >> 2) & 0x01;
    g_vehicleStatus.lr_door_status       = (data[2] >> 3) & 0x01;
    g_vehicleStatus.rr_door_status       = (data[2] >> 4) & 0x01;
    g_vehicleStatus.reading_light_out    = (data[2] >> 5) & 0x01;
    g_vehicleStatus.rh_turn_light_out    = (data[2] >> 6) & 0x01;
    g_vehicleStatus.lh_turn_light_out    = (data[2] >> 7) & 0x01;

    /* ===== Data[3] ===== */
    g_vehicleStatus.low_beam_light_out    = (data[3] >> 0) & 0x01;
    g_vehicleStatus.high_beam_light_status = (data[3] >> 1) & 0x01;
    g_vehicleStatus.trunk_unlock_out      = (data[3] >> 2) & 0x01;
    g_vehicleStatus.parking_light_out     = (data[3] >> 3) & 0x01;
    g_vehicleStatus.wiper_wash_out        = (data[3] >> 4) & 0x01;
    g_vehicleStatus.central_lock_state    = (data[3] >> 6) & 0x03;   /* 2-bit */

    /* ===== Data[4] ===== */
    g_vehicleStatus.rear_defrost_out      = (data[4] >> 0) & 0x01;
    g_vehicleStatus.rear_fog_light_status = (data[4] >> 2) & 0x01;
    g_vehicleStatus.reverse_light_status  = (data[4] >> 3) & 0x01;
    g_vehicleStatus.horn_out              = (data[4] >> 4) & 0x01;
    g_vehicleStatus.front_wiper_status    = (data[4] >> 5) & 0x03;   /* 2-bit */

    /* ===== Data[5] ===== */
    g_vehicleStatus.rf_window_pos = (data[5] >> 0) & 0x03;  /* 2-bit */
    g_vehicleStatus.rr_window_pos = (data[5] >> 2) & 0x03;
    g_vehicleStatus.lf_window_pos = (data[5] >> 4) & 0x03;
    g_vehicleStatus.lr_window_pos = (data[5] >> 6) & 0x03;

    /* ===== Data[6] ===== */
    g_vehicleStatus.outer_mirror_fold      = (data[6] >> 0) & 0x03; /* 2-bit */
    g_vehicleStatus.one_click_start_status = (data[6] >> 2) & 0x03; /* 2-bit */

    /* 更新时间戳 */
    g_vehicleStatus.bcm_tbox1_tick  = xTaskGetTickCount();

    LOG_DBG("BCM_TBOX1 RX: LH=%d RH=%d LB=%d HB=%d Park=%d Fog=%d Haz=%d Wsh=%d Wip=%d RL=%d",
            g_vehicleStatus.lh_turn_light_out,
            g_vehicleStatus.rh_turn_light_out,
            g_vehicleStatus.low_beam_light_out,
            g_vehicleStatus.high_beam_light_status,
            g_vehicleStatus.parking_light_out,
            g_vehicleStatus.rear_fog_light_status,
            g_vehicleStatus.hazard_light_switch,
            g_vehicleStatus.wiper_wash_out,
            g_vehicleStatus.front_wiper_status,
            g_vehicleStatus.reading_light_out);
}

/**
 * @brief  解析 ACU_IVI 报文 (0x18FF181C)
 * @param  data 8 字节 CAN 帧数据
 * @note   ACU 发给 IVI 的空调状态反馈。
 *         Data[0] = 风机挡位 (0=关, 1=1档, 2=2档, 3=3档)
 *         Data[1] = 模式 (0=关, 1=制热, 2=制冷)
 *         Data[2] = 故障码
 *
 *         注意 TX/RX 编码偏移：
 *         TX 侧 CAN_ACU_FAN_LEVEL1=0x02，RX 侧 AC_Gear=1 表示 1 档。
 */
void CAN_ParseACU_IVI(const uint8_t *data)
{
    g_vehicleStatus.ac_fan_gear   = data[0] & 0xFF;
    g_vehicleStatus.ac_mode       = data[1] & 0xFF;
    g_vehicleStatus.ac_fault_code = data[2] & 0xFF;

    g_vehicleStatus.acu_ivi_tick  = xTaskGetTickCount();

    LOG_DBG("ACU_IVI RX: Fan=%d Mode=%d Fault=%d",
            g_vehicleStatus.ac_fan_gear,
            g_vehicleStatus.ac_mode,
            g_vehicleStatus.ac_fault_code);
}

/**
 * @brief  解析 SRCM 报文 (0x18FF271F)
 * @param  data 8 字节 CAN 帧数据
 * @note   天窗/阅读灯控制器的状态反馈。
 *         Data[0].bit0   = 阅读灯状态 (0=关, 1=开)
 *         Data[0].bit1-2 = 风扇挡位 (0=关, 1=一档, 2=二档, 3=三档)
 */
void CAN_ParseSRCM(const uint8_t *data)
{
    g_vehicleStatus.sr_reading_lamp = (data[0] >> 0) & 0x01;
    g_vehicleStatus.sr_fan_level    = (data[0] >> 1) & 0x03;

    g_vehicleStatus.srcm_tick  = xTaskGetTickCount();

    LOG_DBG("SRCM RX: ReadingLamp=%d FanLevel=%d",
            g_vehicleStatus.sr_reading_lamp,
            g_vehicleStatus.sr_fan_level);
}

