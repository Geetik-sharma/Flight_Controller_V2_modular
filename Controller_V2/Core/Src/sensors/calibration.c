/*
 * calibration.c
 *
 *  Created on: 23-Sept-2026
 *      Author: GeetikSharma
 */

#include "sensors/calibration.h"
#include "FreeRTOS.h"
#include "task.h"

#define ACCEL_1G_COUNTS          4096.0f // MPU6500 ±8 g sensitivity (4096 LSB/g)
#define GYRO_SENSITIVITY_2000DPS 16.4f   // MPU6500 ±2000 dps sensitivity (16.4 LSB/dps)

static GyroCalibration_t gyro_calibration;
static AccelCalibration_t accel_calibration;
static CalibrationStatus_t calibration_status = CALIBRATION_STATUS_IDLE;

void Calibration_Init(void)
{
    gyro_calibration.bias_x = 0.0f;
    gyro_calibration.bias_y = 0.0f;
    gyro_calibration.bias_z = 0.0f;

    accel_calibration.bias_x = 0.0f;
    accel_calibration.bias_y = 0.0f;
    accel_calibration.bias_z = 0.0f;

    accel_calibration.scale_x = 1.0f;
    accel_calibration.scale_y = 1.0f;
    accel_calibration.scale_z = 1.0f;

    calibration_status = CALIBRATION_STATUS_IDLE;
}

uint8_t Calibration_Gyro(uint32_t samples)
{
    MPU6500_RawData_t raw;
    int64_t sum_x = 0, sum_y = 0, sum_z = 0;
    uint32_t collected = 0;

    if (samples < 10U)
    {
        calibration_status = CALIBRATION_STATUS_ERROR;
        return 0U;
    }

    calibration_status = CALIBRATION_STATUS_IN_PROGRESS;

    while (collected < samples)
    {
        if (MPU6500_ReadRaw(&raw) == HAL_OK)
        {
            sum_x += raw.gyro_x;
            sum_y += raw.gyro_y;
            sum_z += raw.gyro_z;
            collected++;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    gyro_calibration.bias_x = (float)sum_x / (float)samples;
    gyro_calibration.bias_y = (float)sum_y / (float)samples;
    gyro_calibration.bias_z = (float)sum_z / (float)samples;

    calibration_status = CALIBRATION_STATUS_COMPLETE;
    return 1U;
}

uint8_t Calibration_AccelFlat(uint32_t samples)
{
    MPU6500_RawData_t raw;
    int64_t sum_x = 0, sum_y = 0, sum_z = 0;
    uint32_t collected = 0;

    if (samples < 10U)
    {
        calibration_status = CALIBRATION_STATUS_ERROR;
        return 0U;
    }

    calibration_status = CALIBRATION_STATUS_IN_PROGRESS;

    while (collected < samples)
    {
        if (MPU6500_ReadRaw(&raw) == HAL_OK)
        {
            sum_x += raw.accel_x;
            sum_y += raw.accel_y;
            sum_z += raw.accel_z;
            collected++;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    accel_calibration.bias_x = (float)sum_x / (float)samples;
    accel_calibration.bias_y = (float)sum_y / (float)samples;

    /* Board flat +Z UP: +1g (4096 LSB) is expected on Z axis */
    accel_calibration.bias_z = ((float)sum_z / (float)samples) - ACCEL_1G_COUNTS;

    accel_calibration.scale_x = 1.0f;
    accel_calibration.scale_y = 1.0f;
    accel_calibration.scale_z = 1.0f;

    calibration_status = CALIBRATION_STATUS_COMPLETE;
    return 1U;
}

void Calibration_GetGyro(GyroCalibration_t *calibration)
{
    if (calibration != NULL)
    {
        *calibration = gyro_calibration;
    }
}

void Calibration_GetAccel(AccelCalibration_t *calibration)
{
    if (calibration != NULL)
    {
        *calibration = accel_calibration;
    }
}

void Calibration_ApplyAccel(const MPU6500_RawData_t *raw, SensorPhysicalData_t *calibrated)
{
    if (raw == NULL || calibrated == NULL) return;

    calibrated->accel_x_g = ((float)raw->accel_x - accel_calibration.bias_x) * accel_calibration.scale_x / ACCEL_1G_COUNTS;
    calibrated->accel_y_g = ((float)raw->accel_y - accel_calibration.bias_y) * accel_calibration.scale_y / ACCEL_1G_COUNTS;
    calibrated->accel_z_g = ((float)raw->accel_z - accel_calibration.bias_z) * accel_calibration.scale_z / ACCEL_1G_COUNTS;
}

void Calibration_ApplyGyro(const MPU6500_RawData_t *raw, SensorPhysicalData_t *calibrated)
{
    if (raw == NULL || calibrated == NULL) return;

    calibrated->gyro_x_dps = ((float)raw->gyro_x - gyro_calibration.bias_x) / GYRO_SENSITIVITY_2000DPS;
    calibrated->gyro_y_dps = ((float)raw->gyro_y - gyro_calibration.bias_y) / GYRO_SENSITIVITY_2000DPS;
    calibrated->gyro_z_dps = ((float)raw->gyro_z - gyro_calibration.bias_z) / GYRO_SENSITIVITY_2000DPS;
}

CalibrationStatus_t Calibration_GetStatus(void)
{
    return calibration_status;
}
