/*
 * sensor_processor.c
 *
 *  Created on: 23-Sept-2026
 *      Author: GeetikSharma
 */

#include "sensors/sensor_processor.h"
#include "sensors/calibration.h"
#include <stddef.h>

void SensorProcessor_ProcessRawToPhysical(
    const MPU6500_RawData_t *raw,
    SensorPhysicalData_t *physical
)
{
    if (raw == NULL || physical == NULL)
    {
        return;
    }

    /* Apply calibration offsets and LSB to physical units conversion */
    Calibration_ApplyAccel(raw, physical);
    Calibration_ApplyGyro(raw, physical);

    /* MPU6500 Temperature LSB to Celsius conversion */
    physical->temperature_c = ((float)raw->temperature / 333.87f) + 21.0f;
}
