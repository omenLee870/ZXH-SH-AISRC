#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "app.h"
#include "queue.h"

#define APP_START_TASK_STACK_WORDS      192U    /* 启动任务栈，单位 word，Cortex-M0+ 上 1 word = 4 字节。 */
#define APP_LED_TASK_STACK_WORDS        96U     /* LED 任务栈，单位 word，仅翻转 GPIO 和延时，保持较小即可。 */
#define APP_VOICE_TASK_STACK_WORDS      256U    /* 语音任务栈。                 */
#define APP_VEHICLE_TASK_STACK_WORDS    256U    /* 车辆控制栈。 */
#define APP_CAN_MON_TASK_STACK_WORDS    256U    /* CAN 监控任务栈*/
#define APP_WDG_TASK_STACK_WORDS        64U     /* 看门狗喂狗任务，极小栈即可 */

#define APP_START_TASK_PRIORITY         2U      /* 启动任务优先级，创建完业务任务后会删除自己。 */
#define APP_LED_TASK_PRIORITY           1U      /* LED 心跳任务优先级，低于启动任务。 */
#define APP_VOICE_TASK_PRIORITY         2U      /* 语音任务优先级。*/
#define APP_VEHICLE_TASK_PRIORITY       3U      /* 车辆控制（最高）。*/
#define APP_CAN_MON_TASK_PRIORITY       2U      /* CAN 监控任务优先级 */
#define APP_WDG_TASK_PRIORITY           0U      /* 看门狗任务：最低优先级 */

#define APP_LED_PERIOD_MS               500U    /* PB0 心跳灯翻转周期，单位 ms。 */
#define APP_CAN_MON_PERIOD_MS           500U    /* CAN 状态打印周期，单位 ms，用于观察接收错误计数变化。 */

static TaskHandle_t s_appStartTaskHandle = NULL;    /* 启动任务句柄，当前仅用于创建阶段记录。 */

static void App_StartTask(void *pvParameters);
static void App_LedTask(void *pvParameters);
static void App_VoiceTask(void *pvParameters);
static void App_VehicleTask(void *pvParameters);
static void CAN_RxTask(void *pvParameters);
static void App_WatchdogTask(void *pvParameters);

/**                                                                                                                                         
 * @brief  创建应用启动任务。
 * @retval 无。
 * @note   该函数在 main() 中、vTaskStartScheduler() 之前调用。
 *         如果启动任务创建失败，输出 EEEEEEEE 并进入 APP_ErrorHandler。
 */
void App_TaskCreate(void)
{
    BaseType_t ret;

    ret = xTaskCreate(App_StartTask,
                      "Start",
                      APP_START_TASK_STACK_WORDS,
                      NULL,
                      APP_START_TASK_PRIORITY,
                      &s_appStartTaskHandle);

    if (ret != pdPASS)
    {
        APP_ErrorHandler();
    }
}

/**
 * @brief  应用启动任务。
 * @param  pvParameters FreeRTOS 任务参数，当前未使用。
 * @retval 无。
 * @note   该任务负责创建所有业务任务。创建完成后主动删除自己，
 *         避免长期占用任务栈和调度资源。
 */
static void App_StartTask(void *pvParameters)
{
    BaseType_t ret;

    (void)pvParameters;

    LOG_INFO("FreeRTOS started, creating application tasks...");

    /******************** 创建心跳灯任务 ********************/
    // ret = xTaskCreate(App_LedTask,
    //                   "LED",
    //                   APP_LED_TASK_STACK_WORDS,
    //                   NULL,
    //                   APP_LED_TASK_PRIORITY,
    //                   NULL);

    // if (ret != pdPASS)
    // {
    //     APP_ErrorHandler();
    // }

    /******************** 创建语音处理任务 ********************/
    ret = xTaskCreate(App_VoiceTask, 
                     "Voice",
                      APP_VOICE_TASK_STACK_WORDS,
                      NULL,
                      APP_VOICE_TASK_PRIORITY,
                      NULL);
    if(ret != pdPASS)
    {
        APP_ErrorHandler();
    }

    /******************** 创建车辆控制任务 ********************/
    ret = xTaskCreate(App_VehicleTask,
                      "Vehicle",
                      APP_VEHICLE_TASK_STACK_WORDS,
                      NULL,
                      APP_VEHICLE_TASK_PRIORITY,
                      NULL);
    if (ret != pdPASS)
    {
        LOG_ERR("Failed to create Vehicle task!");
        APP_ErrorHandler();
    }

    /******************** 创建 CAN 接收任务 ********************/
    ret = xTaskCreate(CAN_RxTask,
                      "CAN_Rx",
                      APP_CAN_MON_TASK_STACK_WORDS,
                      NULL,
                      APP_CAN_MON_TASK_PRIORITY,
                      NULL);
    if (ret != pdPASS)
    {
        LOG_ERR("Failed to create CAN_Rx task!");
        APP_ErrorHandler();
    }

    /******************** 创建看门狗喂狗任务 ********************/
    ret = xTaskCreate(App_WatchdogTask,
                      "IWDG",
                      APP_WDG_TASK_STACK_WORDS,
                      NULL,
                      APP_WDG_TASK_PRIORITY,
                      NULL);
    if (ret != pdPASS)
    {
        APP_ErrorHandler();
    }

    LOG_DBG("Start task self-deleting");
    vTaskDelete(NULL);
}

/**
 * @brief  PB0 心跳灯任务。
 * @param  pvParameters FreeRTOS 任务参数，当前未使用。
 * @retval 无。
 * @note   该任务用于验证 FreeRTOS 调度、SysTick 和 vTaskDelay 是否正常。
 */
static void App_LedTask(void *pvParameters)
{
    (void)pvParameters;

    while (1)
    {
        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);
        vTaskDelay(pdMS_TO_TICKS(APP_LED_PERIOD_MS));
    }
}

/* ===== 语音处理任务 ===== */
/**
 * @brief  语音处理任务。
* @param  pvParameters FreeRTOS 任务参数，当前未使用。
* @retval 无。
* @note   该任务负责语音帧的接收和处理。
*         语音帧通过队列从 UART 接收，处理完成后回复成功应答。
*/
static void App_VoiceTask(void *pvParameters)
{
    (void)pvParameters;
    VoiceFrame_t frame;

    LOG_INFO("Voice task started");

    while (1)
    {
        if (xQueueReceive(Voice_GetRxQueue(), &frame, pdMS_TO_TICKS(100U)) == pdPASS)
        {
            App_VoiceProcessFrame(&frame);
        }
        else if (Voice_WakePending())
        {
            /* 三次检测消抖 */
            uint8_t highCount = 0;
            uint8_t i;

            for (i = 0; i < 3; i++)
            {
                vTaskDelay(pdMS_TO_TICKS(5U));           /* 间隔 5ms */
                if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4) == GPIO_PIN_SET)
                {
                    highCount++;
                }
            }

            if (highCount >= 3)                           /* 三次全是高 → 确认 */
            {
                Voice_SendFrame(VOICE_CMD_WAKEUP, 0x00, NULL);
                LOG_INFO("Wake frame sent (PA4 confirmed)");
            }
            else
            {
                LOG_DBG("PA4 bounce ignored (highCount=%d)", highCount);
            }
        }
    }
}

/* ===== VehicleTask ===== */

/**
 * @brief  车辆控制任务。
 * @param  pvParameters 未使用。
 * @note   周期接收车辆请求并轮询语音 pending 表：
 *         1. 收到同步请求时立即执行并通知请求任务；
 *         2. 收到语音异步请求时交给车辆模块入 pending 表；
 *         3. 每 10ms 检查 pending 是否收到车身反馈或超过 3s。
 */
void App_VehicleTask(void *pvParameters)
{
    AppVehicleRequest_t req;

    (void)pvParameters;

    LOG_INFO("Vehicle task started");

    while (1)
    {
        /** @brief  短等待收新请求，避免没有新命令时 pending 超时检查被永久阻塞。 */
        if (xQueueReceive(Vehicle_GetRxQueue(), &req, pdMS_TO_TICKS(10U)) == pdPASS)
        {
            App_VehicleProcessRequest(&req);
        }

        App_VehiclePollPending();
    }
}

/**
 * @brief  CAN 接收任务。
 * @param  pvParameters FreeRTOS 任务参数，当前未使用。
 * @note
 */
static void CAN_RxTask(void *pvParameters)
{
    CAN_RxFrame_t frame;

    (void)pvParameters;

    LOG_INFO("CAN Rx task started");

    while (1)
    {
        if (xQueueReceive(CAN_GetRxQueue(), &frame, portMAX_DELAY) == pdPASS)
        {
            CAN_ProcessRxFrame(&frame);
        }
    }
}

/* ===== 看门狗任务 ===== */

/**
 * @brief  独立看门狗喂狗任务。
 * @note
 */
static void App_WatchdogTask(void *pvParameters)
{
    (void)pvParameters;

    while (1)
    {
        APP_IWDG_Feed();
        vTaskDelay(pdMS_TO_TICKS(500U));
    }
}
