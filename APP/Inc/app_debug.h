/**
 * @file    app_debug.h
 * @brief   调试日志模块
 * @details 提供带等级、可选模块标签的日志宏，支持编译期开关和运行时等级过滤
 *
 * 使用方法：
 *   - LOG_DBG("state=%d", var)                 开发调试
 *   - LOG_INFO("UART started")                 运行信息
 *   - LOG_WARN("retry %d", n)                  警告
 *   - LOG_ERR("timeout!")                      错误
 *   - LOG_INFO_TAG("UART", "baud=%lu", baud)   指定模块标签的信息日志
 *
 * 开关控制：
 *   - DBG_ENABLE = 0          编译时完全剔除所有日志代码（零开销）
 *   - LOG_ACTIVE_LEVEL        运行时过滤，只输出低于等于该等级的日志
 */

#ifndef __APP_DEBUG_H__
#define __APP_DEBUG_H__

#include <stdio.h>

/* ------------------------------------------------------------------ */
/* 调试总开关                                                         */
/* 1 = 开启，0 = 关闭（发布固件时改为 0，编译后零字节占用）            */
/* ------------------------------------------------------------------ */
#define DBG_ENABLE              1

/* ------------------------------------------------------------------ */
/* 日志等级枚举                                                       */
/* 数值越大越详细，LOG_ACTIVE_LEVEL 只输出 <= 自己的等级              */
/* ------------------------------------------------------------------ */
#define LOG_LEVEL_NONE          0   /**< 关闭所有输出                    */
#define LOG_LEVEL_ERR           1   /**< 仅致命错误                     */
#define LOG_LEVEL_WARN          2   /**< 错误 + 警告                    */
#define LOG_LEVEL_INFO          3   /**< 错误 + 警告 + 运行信息         */
#define LOG_LEVEL_DBG           4   /**< 全部输出（含调试）              */

/** @brief 当前生效的日志等级：开发期用 LOG_LEVEL_DBG，发布后改为 LOG_LEVEL_ERR */
#define LOG_ACTIVE_LEVEL        LOG_LEVEL_DBG

/* ================================================================== */
/* 核心日志宏                                                        */
/* ================================================================== */

#if DBG_ENABLE

/**
 * @brief 错误日志 —— 出现时必须立刻处理的异常
 */
#define LOG_ERR(fmt, ...) \
    do { \
        if (LOG_ACTIVE_LEVEL >= LOG_LEVEL_ERR) \
            printf("[ERR] " fmt "\r\n", ##__VA_ARGS__); \
    } while(0)

/**
 * @brief 带模块标签的错误日志 —— 用于区分 UART、CAN、KEY 等不同模块
 */
#define LOG_ERR_TAG(tag, fmt, ...) \
    do { \
        if (LOG_ACTIVE_LEVEL >= LOG_LEVEL_ERR) \
            printf("[ERR][%s] " fmt "\r\n", tag, ##__VA_ARGS__); \
    } while(0)

/**
 * @brief 警告日志 —— 不影响当前流程但需关注的异常状态
 */
#define LOG_WARN(fmt, ...) \
    do { \
        if (LOG_ACTIVE_LEVEL >= LOG_LEVEL_WARN) \
            printf("[WARN] " fmt "\r\n", ##__VA_ARGS__); \
    } while(0)

/**
 * @brief 带模块标签的警告日志 —— 用于快速定位是哪一类外设或业务状态异常
 */
#define LOG_WARN_TAG(tag, fmt, ...) \
    do { \
        if (LOG_ACTIVE_LEVEL >= LOG_LEVEL_WARN) \
            printf("[WARN][%s] " fmt "\r\n", tag, ##__VA_ARGS__); \
    } while(0)

/**
 * @brief 信息日志 —— 关键节点运行状态（启动、配置成功等）
 */
#define LOG_INFO(fmt, ...) \
    do { \
        if (LOG_ACTIVE_LEVEL >= LOG_LEVEL_INFO) \
            printf("[INFO] " fmt "\r\n", ##__VA_ARGS__); \
    } while(0)

/**
 * @brief 带模块标签的信息日志 —— 推荐用于初始化结果和关键业务节点
 */
#define LOG_INFO_TAG(tag, fmt, ...) \
    do { \
        if (LOG_ACTIVE_LEVEL >= LOG_LEVEL_INFO) \
            printf("[INFO][%s] " fmt "\r\n", tag, ##__VA_ARGS__); \
    } while(0)

/**
 * @brief 调试日志 —— 变量值、流程分支等临时调试信息
 */
#define LOG_DBG(fmt, ...) \
    do { \
        if (LOG_ACTIVE_LEVEL >= LOG_LEVEL_DBG) \
            printf("[DBG] " fmt "\r\n", ##__VA_ARGS__); \
    } while(0)

/**
 * @brief 带模块标签的调试日志 —— 推荐用于变量值、状态机分支等临时调试信息
 */
#define LOG_DBG_TAG(tag, fmt, ...) \
    do { \
        if (LOG_ACTIVE_LEVEL >= LOG_LEVEL_DBG) \
            printf("[DBG][%s] " fmt "\r\n", tag, ##__VA_ARGS__); \
    } while(0)

/**
 * @brief 裸 printf 替代，不受日志等级限制
 */
#define DBG_PRINT(fmt, ...) \
    printf(fmt, ##__VA_ARGS__)

#else
/* 调试关闭：所有宏展开为空，编译器直接在预处理阶段抹掉 */
#define LOG_ERR(fmt, ...)
#define LOG_ERR_TAG(tag, fmt, ...)
#define LOG_WARN(fmt, ...)
#define LOG_WARN_TAG(tag, fmt, ...)
#define LOG_INFO(fmt, ...)
#define LOG_INFO_TAG(tag, fmt, ...)
#define LOG_DBG(fmt, ...)
#define LOG_DBG_TAG(tag, fmt, ...)
#define DBG_PRINT(fmt, ...)
#endif /* DBG_ENABLE */

/* ------------------------------------------------------------------ */
/* 轻量 assert                                                       */
/* 标准库 assert 在嵌入式不太合适，这里提供一个不依赖 NDEBUG 的版本   */
/* ------------------------------------------------------------------ */
#define LOG_ASSERT(cond) \
    do { \
        if (!(cond)) { \
            LOG_ERR("ASSERT FAILED: %s", #cond); \
            while (1) {} \
        } \
    } while(0)

#endif /* __APP_DEBUG_H__ */
