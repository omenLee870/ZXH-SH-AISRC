#ifndef __APP_TASK_H__
#define __APP_TASK_H__

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  创建应用层 FreeRTOS 启动任务。
 * @retval 无。
 * @note   main() 只调用该入口，不直接创建各业务任务。
 *         后续 LED、CAN、语音识别、按键等任务统一由启动任务创建。
 */
void App_TaskCreate(void);

#ifdef __cplusplus
}
#endif

#endif /* __APP_TASK_H__ */
