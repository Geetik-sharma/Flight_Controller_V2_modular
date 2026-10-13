/*
 * data_bus.c
 *
 *  Created on: 24-Sept-2026
 *      Author: GeetikSharma
 */

#include "system/Data_bus.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"

/* =========================
 * Sensor Data Storage
 * ========================= */

static SensorPhysicalData_t sensor_data;


/* =========================
 * Publish Sensor Data
 * ========================= */

void DataBus_PublishSensorData(
    const SensorPhysicalData_t *data
)
{
    if (data == NULL)
    {
        return;
    }

    // Protect the shared structure copy from concurrent access
    taskENTER_CRITICAL();
    sensor_data = *data;
    taskEXIT_CRITICAL();
}


/* =========================
 * Get Sensor Data
 * ========================= */

uint8_t DataBus_GetSensorData(
    SensorPhysicalData_t *data
)
{
    if (data == NULL)
    {
        return 0U;
    }

    // Protect the shared structure copy from concurrent access
    taskENTER_CRITICAL();
    *data = sensor_data;
    taskEXIT_CRITICAL();

    return 1U;
}
