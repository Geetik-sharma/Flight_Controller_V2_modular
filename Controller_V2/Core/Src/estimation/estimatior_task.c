///*
// * estimatior_task.h
// *
// *  Created on: 24-Sept-2026
// *      Author: GeetikSharma
// */

#include "estimation/estimator_task.h"
#include "estimation/ahrs.h"
#include "estimation/estimator_state.h"
#include "system/data_bus.h"
#include "system/system_event.h"

TaskHandle_t xEstimatorTaskHandle = NULL;
TaskHandle_t xControllerManagerTaskHandle = NULL;
static TaskHandle_t estimator_task_handle_internal = NULL;

#define ESTIMATOR_TASK_FREQUENCY_HZ    500U
#define ESTIMATOR_DT_SEC               (1.0f / (float)ESTIMATOR_TASK_FREQUENCY_HZ)

static void EstimatorTask(void *argument);

void EstimatorTask_Start(void)
{
    BaseType_t status;

    status = xTaskCreate(
        EstimatorTask,
        "Estimator",
        512,
        NULL,
        5,
        &estimator_task_handle_internal
    );

    if (status != pdPASS)
    {
        taskDISABLE_INTERRUPTS();

        while (1)
        {
            /* Estimator task creation failure */
        }
    }

    xEstimatorTaskHandle = estimator_task_handle_internal;
}


static void EstimatorTask(void *argument)
{
    (void)argument;

    // Wait until calibration completes
    SystemEvents_Wait(SYSTEM_EVENT_CALIBRATION_DONE);

    AHRS_Init();
    EstimatorState_Init();

    SensorPhysicalData_t sensor_data;
    AHRS_Quaternion_t q;
    AHRS_Attitude_t euler;

    for (;;)
    {
        // Block until notified by SensorTask (10 ms timeout as safety fallback)
        uint32_t ulNotificationValue = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(10));

        if (ulNotificationValue > 0)
        {
            if (DataBus_GetSensorData(&sensor_data))
            {
                AHRS_Update(&sensor_data, ESTIMATOR_DT_SEC);

                AHRS_GetQuaternion(&q);
                AHRS_GetAttitude(&euler);

                EstimatorState_Publish(&q, euler.roll, euler.pitch, euler.yaw,
                                       sensor_data.gyro_x_dps,
                                       sensor_data.gyro_y_dps,
                                       sensor_data.gyro_z_dps);
                if (xControllerManagerTaskHandle != NULL)
                {
                    xTaskNotifyGive(xControllerManagerTaskHandle);
                }
            }
        }
    }
}
