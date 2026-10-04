#include <stddef.h>
#include "stm32f1xx_hal.h"
#include "common/mavlink.h"

#include "uORB.h"
#include "Imu.h"
#include "FreeRTOS.h"
#include "task.h"

static uORB_Topic *uORB_TopicHandler = NULL;
static mavlink_raw_imu_t ImuData = {0};
static TaskHandle_t ImuPublishTaskHandle = NULL;

static void ImuPublishData(void *args)
{
    while (1)
    {
        // Simulate reading IMU data
        ImuData.time_usec = HAL_GetTick() * 1000; // Convert milliseconds to microseconds
        ImuData.xacc = 100;                       // Simulated accelerometer data
        ImuData.yacc = 200;                       // Simulated accelerometer data
        ImuData.zacc = 300;                       // Simulated accelerometer data
        ImuData.xgyro = 10;                       // Simulated gyroscope data
        ImuData.ygyro = 20;                       // Simulated gyroscope data
        ImuData.zgyro = 30;                       // Simulated gyroscope data

        uORB_Publish(uORB_TopicHandler, &ImuData);
        vTaskDelay(pdMS_TO_TICKS(20)); // Delay for 50 Hz update rate
    }
}

int8_t Imu_Init(void)
{
    // Initialization code for IMU
    uORB_TopicHandler = uORB_RegisterTopic("sensor/imu_data", (uint8_t *)&ImuData, sizeof(mavlink_raw_imu_t));

    if (uORB_TopicHandler == NULL)
    {
        return -1; // Error: Failed to register topic
    }

    int ret = xTaskCreate(ImuPublishData, "ImuStreamer", configMINIMAL_STACK_SIZE, NULL, 1, &ImuPublishTaskHandle);
    if (ret != pdPASS)
    {
        return -1; // Error: Failed to create task
    }

    return 0; // Success
}
