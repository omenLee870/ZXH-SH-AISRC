/**
 * @file    app_can_proto.h
 * @brief   雷迈通讯协议（IVI A2.1）—— CAN 报文定义
 * @details 基于雷迈 IVI A2.1 协议文档，定义 IVI（车机）与各 ECU 之间的 CAN 通信规范。
 *
 *          协议概要：
 *          - 物理层：CAN 2.0B, 250Kbps
 *          - 帧格式：J1939 扩展帧（29-bit Identifier）
 *          - 字节序：Motorola LSB（高字节在低地址，信号从高位开始）
 *          - 信号编码：0x0 = 默认/无请求, 0x1 = 开启/ON, 0x2 = 关闭/OFF
 *            部分信号用 3-bit 编码多级状态（如雨刮 0x1=关/0x2=间歇/0x3=低速/0x4=高速）
 *
 *          IVI 发送的报文（本模块负责）：
 *          - IVI_BCM (0x18FF1D18) : 车身控制, DLC=8, 100ms 周期
 *          - IVI_MCU (0x18FFEF18) : 驾驶模式控制, DLC=8, 100ms 周期（预留）
 *
 *          其他节点发给 IVI 的报文（本模块负责接收）：
 *          - MCU_DPLY1 (0x18FF17EF) : 档位/车速/SOC, DLC=8, 20ms 周期
 *          - BCM_TBOX1 (0x18FF271D) : 门锁/灯光状态, DLC=8, 100ms 周期
 *          - BCM_TBOX2 (0x18FE271D) : 故障码/传感器, DLC=8, 100ms 周期
 *
 *          参考文档：
 *          E:\...\资料\雷迈通讯协议（IVI）A2.1.xlsx
 */

#ifndef __APP_CAN_PROTO_H__
#define __APP_CAN_PROTO_H__

#include <stdint.h>

/* ================================================================== */
/* 雷迈协议 — 报文 ID（J1939 29-bit 扩展帧）                            */
/* ================================================================== */

/* ---------- IVI 发送 (TX) ---------- */
#define CAN_ID_IVI_BCM              0x18FF1D18U     /**< IVI → BCM: 车身控制命令。 */
#define CAN_ID_IVI_MCU              0x18FFEF18U     /**< IVI → MCU: 驾驶模式控制。 */

/* ---------- IVI 接收 (RX) ---------- */
#define CAN_ID_MCU_DPLY1            0x18FF17EFU     /**< MCU → 仪表: 档位/车速/SOC   */
#define CAN_ID_BCM_TBOX1            0x18FF271DU     /**< BCM → 各节点: 门锁/灯光     */
#define CAN_ID_BCM_TBOX2            0x18FE271DU     /**< BCM → 各节点: 故障码/传感器  */

/* ================================================================== */
/* 雷迈协议 — IVI_BCM 信号定义 (ID=0x18FF1D18, DLC=8, 100ms 周期)      */
/*                                                                     */
/* 编码规则：                                                           */
/*   2-bit 信号: 0x0=无请求, 0x1=ON/开启, 0x2=OFF/关闭                  */
/*   3-bit 信号: 0x0=无请求, 0x1=状态0, 0x2=状态1, ...                  */
/*                                                                     */
/* 字节布局 (Motorola LSB):                                            */
/*   Byte0 [bit7-0]: 座椅加热[3:2] 车窗禁止[5:4] 中控锁[7:6]           */
/*   Byte1 [bit7-0]: 四窗升降（RF/RR/LF/LR 各 2bit）                    */
/*   Byte2 [bit7-0]: 防狼[1:0] 后视镜[3:2] 预留[5:4] 洗涤[7:6]         */
/*   Byte3 [bit7-0]: 小灯[1:0] 近光[3:2] 远光[5:4] 超车灯[7:6]         */
/*   Byte4 [bit7-0]: 后雾灯[1:0] 双闪[3:2] 后备箱[5:4] 预留[bit6]      */
/*   Byte5 [bit7-0]: 天窗风扇[2:0] 预留[bit3] 左转向[5:4] 右转向[7:6]  */
/*   Byte6 [bit7-0]: 阅读灯[1:0] 前雨刮[4:2] 一键启动[7:5]             */
/*   Byte7 : 预留                                                       */
/* ================================================================== */

/* ---------- Data[0]: 座椅加热 / 车窗禁止 / 中控锁 ---------- */
#define CAN_BCM_SEAT_HEATING_POS        2       /**< bit2-3, 座椅加热            */
#define CAN_BCM_SEAT_HEATING_ON         0x1
#define CAN_BCM_SEAT_HEATING_OFF        0x2

#define CAN_BCM_WINDOW_INHIBIT_POS      4       /**< bit4-5, 乘员侧车窗禁止       */
#define CAN_BCM_WINDOW_INHIBIT_ON       0x1
#define CAN_BCM_WINDOW_INHIBIT_OFF      0x2

#define CAN_BCM_CENTRAL_LOCK_POS        6       /**< bit6-7, 中控锁              */
#define CAN_BCM_CENTRAL_LOCK_UNLOCK     0x1     /**< 解锁                        */
#define CAN_BCM_CENTRAL_LOCK_LOCK       0x2     /**< 关锁                        */

/* ---------- Data[1]: 四车窗一键升降 ---------- */
#define CAN_BCM_RF_WINDOW_POS           0       /**< bit0-1, 右前车窗             */
#define CAN_BCM_RR_WINDOW_POS           2       /**< bit2-3, 右后车窗             */
#define CAN_BCM_LF_WINDOW_POS           4       /**< bit4-5, 左前车窗             */
#define CAN_BCM_LR_WINDOW_POS           6       /**< bit6-7, 左后车窗             */
#define CAN_BCM_WINDOW_UP               0x1     /**< 上升                        */
#define CAN_BCM_WINDOW_DOWN             0x2     /**< 下降                        */

/* ---------- Data[2]: 一键防狼 / 外后视镜 / 洗涤 ---------- */
#define CAN_BCM_MIRROR_FOLD_POS         2       /**< bit2-3, 外后视镜折叠         */
#define CAN_BCM_MIRROR_UNFOLD           0x1     /**< 展开                        */
#define CAN_BCM_MIRROR_FOLD             0x2     /**< 折叠                        */

#define CAN_BCM_WASHER_POS              6       /**< bit6-7, 雨刮喷水洗涤         */
#define CAN_BCM_WASHER_ON               0x1
#define CAN_BCM_WASHER_OFF              0x2

/* ---------- Data[3]: 车灯控制 ---------- */
#define CAN_BCM_PARKING_LIGHT_POS       0       /**< bit0-1, 小灯/示宽灯          */
#define CAN_BCM_LOW_BEAM_POS            2       /**< bit2-3, 近光灯              */
#define CAN_BCM_HIGH_BEAM_POS           4       /**< bit4-5, 远光灯              */
#define CAN_BCM_PASSING_LIGHT_POS       6       /**< bit6-7, 超车灯(闪大灯)      */
#define CAN_BCM_LIGHT_ON                0x1
#define CAN_BCM_LIGHT_OFF               0x2

/* ---------- Data[4]: 后雾灯 / 双闪 / 后备箱 ---------- */
#define CAN_BCM_REAR_FOG_POS            0       /**< bit0-1, 后雾灯              */
#define CAN_BCM_HAZARD_POS              2       /**< bit2-3, 危险报警灯(双闪)     */
#define CAN_BCM_TRUNK_UNLOCK_POS        4       /**< bit4-5, 后备箱解锁           */
#define CAN_BCM_TRUNK_UNLOCK            0x1     /**< 解锁指令                    */

/* ---------- Data[5]: 天窗风扇 / 转向灯 ---------- */
#define CAN_BCM_SUNROOF_FAN_POS         0       /**< bit0-2 (3bit), 天窗风扇      */
#define CAN_BCM_SUNROOF_FAN_OFF         0x1
#define CAN_BCM_SUNROOF_FAN_LEVEL1      0x2
#define CAN_BCM_SUNROOF_FAN_LEVEL2      0x3
#define CAN_BCM_SUNROOF_FAN_LEVEL3      0x4

#define CAN_BCM_LH_TURN_POS             4       /**< bit4-5, 左转向灯             */
#define CAN_BCM_RH_TURN_POS             6       /**< bit6-7, 右转向灯             */
#define CAN_BCM_TURN_ON                 0x1
#define CAN_BCM_TURN_OFF                0x2

/* ---------- Data[6]: 阅读灯 / 前雨刮 / 一键启动 ---------- */
#define CAN_BCM_READING_LIGHT_POS       0       /**< bit0-1, 阅读灯              */
#define CAN_BCM_READING_LIGHT_ON        0x1
#define CAN_BCM_READING_LIGHT_OFF       0x2

#define CAN_BCM_FRONT_WIPER_POS         2       /**< bit2-4 (3bit), 前雨刮        */
#define CAN_BCM_WIPER_OFF               0x1
#define CAN_BCM_WIPER_INTERVAL          0x2     /**< 间歇模式                    */
#define CAN_BCM_WIPER_LOW               0x3     /**< 低速连续                    */
#define CAN_BCM_WIPER_HIGH              0x4     /**< 高速连续                    */

#define CAN_BCM_START_REQ_POS           5       /**< bit5-7 (3bit), 一键启动      */
#define CAN_BCM_START_OFF               0x1
#define CAN_BCM_START_ACC               0x2
#define CAN_BCM_START_ON                0x3
#define CAN_BCM_START_CRANK             0x4     /**< START 点火                   */

/* ================================================================== */
/* 雷迈协议 — IVI_MCU 信号定义 (ID=0x18FFEF18, DLC=8, 100ms 周期)      */
/* 驾驶模式控制，当前预留，后续扩展                                     */
/* ================================================================== */
#define CAN_MCU_BRAKE_POS               0       /**< Data[0] bit0-1, 刹车         */
#define CAN_MCU_FWD_REV_POS             24      /**< Data[3] bit0-1, 前进/后退     */

/* ================================================================== */
/* 辅助宏: 写 2-bit / 3-bit 信号到目标字节                              */
/* ================================================================== */

/**
 * @brief  将 2-bit 信号值写入目标字节的特定位
 * @param  val  信号值 (0x0 ~ 0x3)
 * @param  pos  信号起始 bit 位置 (0~6)
 * @return 可直接 OR 到目标字节的值
 * @note   使用示例:
 *         byte3 = CAN_SET_2BIT(CAN_BCM_LOW_BEAM_ON,  CAN_BCM_LOW_BEAM_POS)
 *               | CAN_SET_2BIT(CAN_BCM_HIGH_BEAM_OFF, CAN_BCM_HIGH_BEAM_POS);
 */
#define CAN_SET_2BIT(val, pos)          (((val) & 0x03) << (pos))

/**
 * @brief  将 3-bit 信号值写入目标字节的特定位
 * @param  val  信号值 (0x0 ~ 0x7)
 * @param  pos  信号起始 bit 位置 (0~5)
 * @return 可直接 OR 到目标字节的值
 */
#define CAN_SET_3BIT(val, pos)          (((val) & 0x07) << (pos))

/* ================================================================== */
/* 协议层 API 声明                                                     */
/* ================================================================== */

/**
 * @brief  构建并发送 IVI_BCM 报文
 * @param  data 8 字节帧数据（由 APP_CAN_BuildBCM_Frame 生成或手动填充）
 * @return 0 = 成功, -1 = 发送失败
 */
int APP_CAN_SendIVI_BCM(uint8_t *data);

/**
 * @brief  构建并发送 IVI_MCU 报文（驾驶模式，预留）
 * @param  data 8 字节帧数据
 * @return 0 = 成功, -1 = 发送失败
 */
int APP_CAN_SendIVI_MCU(uint8_t *data);

#endif /* __APP_CAN_PROTO_H__ */
