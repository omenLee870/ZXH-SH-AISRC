#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "app.h"
#include "queue.h"

#define APP_START_TASK_STACK_WORDS      192U    /* 启动任务栈，单位 word，Cortex-M0+ 上 1 word = 4 字节。 */
#define APP_LED_TASK_STACK_WORDS        96U     /* LED 任务栈，单位 word，仅翻转 GPIO 和延时，保持较小即可。 */
#define APP_VOICE_TASK_STACK_WORDS      256U    /* 语音任务栈。                 */
#define APP_VEHICLE_TASK_STACK_WORDS    256U    /* 车辆控制栈。 */
#define APP_CAN_MON_TASK_STACK_WORDS    192U    /* CAN 监控任务栈，用于周期读取状态并打印诊断日志。 */

#define APP_START_TASK_PRIORITY         2U      /* 启动任务优先级，创建完业务任务后会删除自己。 */
#define APP_LED_TASK_PRIORITY           1U      /* LED 心跳任务优先级，低于启动任务。 */
#define APP_VOICE_TASK_PRIORITY         2U      /* 语音任务优先级。*/
#define APP_VEHICLE_TASK_PRIORITY       3U      /* 车辆控制（最高）。← 新增    */
#define APP_CAN_MON_TASK_PRIORITY       1U      /* CAN 监控任务优先级，仅输出调试信息，保持较低。 */

#define APP_LED_PERIOD_MS               500U    /* PB0 心跳灯翻转周期，单位 ms。 */
#define APP_CAN_MON_PERIOD_MS           500U    /* CAN 状态打印周期，单位 ms，用于观察接收错误计数变化。 */

static TaskHandle_t s_appStartTaskHandle = NULL;    /* 启动任务句柄，当前仅用于创建阶段记录。 */

static void App_StartTask(void *pvParameters);
static void App_LedTask(void *pvParameters);
static void App_VoiceTask(void *pvParameters);
static void App_VehicleTask(void *pvParameters);

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
        /* 阻塞等待语音帧（ISR 通过队列扔进来的） */
        if (xQueueReceive(Voice_GetRxQueue(), &frame, portMAX_DELAY) == pdPASS)
        {
            /* 应答 */
            App_VoiceProcessFrame(&frame);
        }
    }
}

/* ===== VehicleTask ===== */

/**
 * @brief  车辆控制任务。
 * @param  pvParameters 未使用。
 * @note   阻塞等待请求 → 执行 → 任务通知回结果 → 循环。
 */
void App_VehicleTask(void *pvParameters)
{
    AppVehicleRequest_t req;
    AppVehicleResult_t  result;

    (void)pvParameters;

    LOG_INFO("Vehicle task started");

    while (1)
    {
        /** @brief  阻塞等请求，无请求时 CPU 让给其他任务。 */
        if (xQueueReceive(Vehicle_GetRxQueue(), &req, portMAX_DELAY) == pdPASS)
        {
            result = App_VehicleExecute(req.cmd);

            /** @brief  通过任务通知，把结果定向发给请求方。 */
            if (req.requester != NULL)
            {
                xTaskNotify(req.requester,
                            (uint32_t)result,
                            eSetValueWithOverwrite);
            }
        }
    }
}
