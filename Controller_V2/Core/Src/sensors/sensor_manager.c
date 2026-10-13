/*
 * sensor_manager.c
 *
 *  Created on: 18-Sept-2026
 *      Author: GeetikSharma
 */

#include "sensors/sensor_manager.h"
#include "sensors/sensor_processor.h"
#include "drivers/imu/mpu6500.h"
#include "system/data_bus.h"

#include "FreeRTOS.h"
#include "task.h"

#define SENSOR_TASK_FREQUENCY_HZ    1000U
#define SENSOR_TASK_PERIOD_MS       (1000U / SENSOR_TASK_FREQUENCY_HZ)

#define SENSOR_TASK_STACK_SIZE      256U
#define SENSOR_TASK_PRIORITY        6U  // High priority driver task

extern TaskHandle_t xEstimatorTaskHandle;

static void SensorTask(void *argument);

void SensorManager_Start(void)
{
    BaseType_t status;

    status = xTaskCreate(
        SensorTask,
        "Sensor",
        SENSOR_TASK_STACK_SIZE,
        NULL,
        SENSOR_TASK_PRIORITY,
        NULL
    );

    if (status != pdPASS)
    {
        taskDISABLE_INTERRUPTS();
        while (1)
        {
            // Task creation failure
        }
    }
}

static void SensorTask(void *argument)
{
    MPU6500_RawData_t raw_data;
    SensorPhysicalData_t physical_data;
    uint8_t estimator_divider = 0;
    TickType_t last_wake_time;

    (void)argument;

    if (MPU6500_Init() == 0)
    {
        taskDISABLE_INTERRUPTS();
        while (1)
        {
            // Hardware driver init error
        }
    }

    last_wake_time = xTaskGetTickCount();

    for (;;)
    {
        /* 1. Read raw SPI data from MPU6500 */
        if (MPU6500_ReadRaw(&raw_data) == HAL_OK)
        {
            /* 2. Process raw LSBs into physical units (g, dps, °C) */
            SensorProcessor_ProcessRawToPhysical(&raw_data, &physical_data);

            /* 3. Thread-safe publish directly to DataBus */
            DataBus_PublishSensorData(&physical_data);

            /* 4. Downsample 1 kHz -> 500 Hz for EstimatorTask */
            estimator_divider++;
            if (estimator_divider >= 2)
            {
                estimator_divider = 0;

                /* Instantly wake EstimatorTask */
                if (xEstimatorTaskHandle != NULL)
                {
                    xTaskNotifyGive(xEstimatorTaskHandle);
                }
            }
        }

        /* Wait for next 1 ms tick */
        vTaskDelayUntil(
            &last_wake_time,
            pdMS_TO_TICKS(SENSOR_TASK_PERIOD_MS)
        );
    }
}
