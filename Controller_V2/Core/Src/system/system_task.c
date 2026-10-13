#include "system/system_task.h"
#include "main.h"
#include "sensors/calibration.h"
#include "sensors/sensor_manager.h"
#include "sensors/sensor_data.h"
#include "FreeRTOS.h"
#include "task.h"
#include "sensors/sensor_processor.h"
#include "system/system_event.h"
#include "estimation/estimator_task.h"
#include "controller/mixer.h"
#include "actuator/actuator_output.h"
#include "controller/controller_manager.h"
#include "rc/rc_input.h"

static void SystemTask(void *argument)
{
    (void)argument;

    vTaskDelay(pdMS_TO_TICKS(1000));

    Calibration_Init();

    if (Calibration_Gyro(1000U) != 0U &&
        Calibration_AccelFlat(1000U) != 0U)
    {
        SystemEvents_Set(SYSTEM_EVENT_CALIBRATION_DONE);
    }
    RCInput_Init();
    ControllerManager_Init();
    EstimatorTask_Start();
    ControllerManager_Start();
    for (;;)
    {

        vTaskDelay(pdMS_TO_TICKS(1000));
        HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);
    }
}

void SystemTask_Start(void)
{
    BaseType_t status;

    status = xTaskCreate(
        SystemTask,
        "System",
        128,
        NULL,
        1,
        NULL
    );

    if (status != pdPASS)
    {
        taskDISABLE_INTERRUPTS();

        while (1)
        {
        }
    }
}
