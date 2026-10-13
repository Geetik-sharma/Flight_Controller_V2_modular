/*
 * calibration.h
 *
 *  Created on: 23-Sept-2026
 *      Author: GeetikSharma
 */

#ifndef SENSORS_CALIBRATION_H_
#define SENSORS_CALIBRATION_H_

#include <stdint.h>
#include "drivers/imu/mpu6500.h"
#include "sensors/sensor_data.h"

typedef enum
{
    CALIBRATION_STATUS_IDLE = 0,
    CALIBRATION_STATUS_IN_PROGRESS,
    CALIBRATION_STATUS_COMPLETE,
    CALIBRATION_STATUS_ERROR
} CalibrationStatus_t;

typedef struct
{
    float bias_x;
    float bias_y;
    float bias_z;
} GyroCalibration_t;

typedef struct
{
    float bias_x;
    float bias_y;
    float bias_z;
    float scale_x;
    float scale_y;
    float scale_z;
} AccelCalibration_t;

void Calibration_Init(void);

uint8_t Calibration_Gyro(uint32_t samples);
uint8_t Calibration_AccelFlat(uint32_t samples);

void Calibration_GetGyro(GyroCalibration_t *calibration);
void Calibration_GetAccel(AccelCalibration_t *calibration);

void Calibration_ApplyAccel(const MPU6500_RawData_t *raw, SensorPhysicalData_t *calibrated);
void Calibration_ApplyGyro(const MPU6500_RawData_t *raw, SensorPhysicalData_t *calibrated);

CalibrationStatus_t Calibration_GetStatus(void);

#endif /* SENSORS_CALIBRATION_H_ */
