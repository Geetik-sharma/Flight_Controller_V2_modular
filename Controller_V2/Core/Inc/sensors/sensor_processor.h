/*
 * sensor_processor.h
 *
 *  Created on: 23-Sept-2026
 *      Author: GeetikSharma
 */

#ifndef SENSORS_SENSOR_PROCESSOR_H_
#define SENSORS_SENSOR_PROCESSOR_H_

#include "drivers/imu/mpu6500.h"
#include "sensors/sensor_data.h"

#ifdef __cplusplus
extern "C" {
#endif

void SensorProcessor_ProcessRawToPhysical(
    const MPU6500_RawData_t *raw,
    SensorPhysicalData_t *physical
);

#ifdef __cplusplus
}
#endif

#endif /* SENSORS_SENSOR_PROCESSOR_H_ */
