/*
 * mpu6500.h
 *
 *  Created on: 18-Sept-2026
 *      Author: GeetikSharma
 */

#ifndef INC_DRIVERS_IMU_MPU6500_H_
#define INC_DRIVERS_IMU_MPU6500_H_

#include "main.h"
#include <stdint.h>

/* MPU6500 Identification */
#define MPU6500_WHO_AM_I             0x75
#define MPU6500_WHO_AM_I_VALUE       0x70

/* Power Management */
#define MPU6500_PWR_MGMT_1           0x6B
#define MPU6500_PWR_MGMT_2           0x6C

/* Configuration */
#define MPU6500_SMPLRT_DIV           0x19
#define MPU6500_CONFIG                0x1A
#define MPU6500_GYRO_CONFIG           0x1B
#define MPU6500_ACCEL_CONFIG          0x1C
#define MPU6500_ACCEL_CONFIG2         0x1D

/* Raw Sensor Data */
#define MPU6500_ACCEL_XOUT_H          0x3B

/* MPU6500 Raw Sensor Data */
typedef struct
{
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;

    int16_t temperature;

    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;

} MPU6500_RawData_t;


/* Register Access */
uint8_t MPU6500_ReadReg(uint8_t reg);

void MPU6500_WriteReg(uint8_t reg, uint8_t value);


/* Device Functions */
uint8_t MPU6500_GetWhoAmI(void);

uint8_t MPU6500_Init(void);


/* Raw Sensor Data */
HAL_StatusTypeDef MPU6500_ReadRaw(MPU6500_RawData_t *data);

#endif /* INC_DRIVERS_IMU_MPU6500_H_ */
