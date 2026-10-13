/*
 * mpu6500.c
 *
 *  Created on: 18-Sept-2026
 *      Author: GeetikSharma
 */


/*
 * mpu6500.c
 *
 * MPU6500 IMU driver
 */

#include "drivers/imu/mpu6500.h"
#include "hal/hal_spi.h"
#include "main.h"


uint8_t MPU6500_ReadReg(uint8_t reg)
{
    uint8_t tx_data[2];
    uint8_t rx_data[2];

    tx_data[0] = reg | 0x80;
    tx_data[1] = 0x00;

    rx_data[0] = 0x00;
    rx_data[1] = 0x00;

    FC_SPI_Select();

    FC_SPI_WriteRead(
        tx_data,
        rx_data,
        2
    );

    FC_SPI_Deselect();

    return rx_data[1];
}


void MPU6500_WriteReg(uint8_t reg, uint8_t value)
{
    uint8_t tx_data[2];

    tx_data[0] = reg & 0x7F;
    tx_data[1] = value;

    FC_SPI_Select();

    FC_SPI_Write(
        tx_data,
        2
    );

    FC_SPI_Deselect();
}



uint8_t MPU6500_GetWhoAmI(void)
{
    return MPU6500_ReadReg(MPU6500_WHO_AM_I);
}

uint8_t MPU6500_Init(void)
{
    uint8_t who_am_i;

    who_am_i = MPU6500_GetWhoAmI();

    if (who_am_i != MPU6500_WHO_AM_I_VALUE)
    {
        return 0;
    }


    MPU6500_WriteReg(
        MPU6500_PWR_MGMT_1,
        0x80
    );

    HAL_Delay(100);


    MPU6500_WriteReg(
        MPU6500_PWR_MGMT_1,
        0x01
    );


    MPU6500_WriteReg(
        MPU6500_PWR_MGMT_2,
        0x00
    );

    MPU6500_WriteReg(
        MPU6500_GYRO_CONFIG,
        0x18
    );


    MPU6500_WriteReg(
        MPU6500_CONFIG,
        0x01
    );


    MPU6500_WriteReg(
        MPU6500_SMPLRT_DIV,
        0x00
    );


    MPU6500_WriteReg(
        MPU6500_ACCEL_CONFIG,
        0x10
    );



    MPU6500_WriteReg(
        MPU6500_ACCEL_CONFIG2,
        0x02
    );


    return 1;
}

HAL_StatusTypeDef MPU6500_ReadRaw(MPU6500_RawData_t *data)
{
    uint8_t tx_data[15];
    uint8_t rx_data[15];

    tx_data[0] = MPU6500_ACCEL_XOUT_H | 0x80;


    for (uint8_t i = 1; i < 15; i++)
    {
        tx_data[i] = 0x00;
    }
    HAL_StatusTypeDef status;
    FC_SPI_Select();

    status=FC_SPI_WriteRead(
        tx_data,
        rx_data,
        15
    );

    FC_SPI_Deselect();
    if (status != HAL_OK)
    {
        return status;
    }
    data->accel_x =
        (int16_t)((rx_data[1] << 8) | rx_data[2]);

    data->accel_y =
        (int16_t)((rx_data[3] << 8) | rx_data[4]);

    data->accel_z =
        (int16_t)((rx_data[5] << 8) | rx_data[6]);


    data->temperature =
        (int16_t)((rx_data[7] << 8) | rx_data[8]);


    data->gyro_x =
        (int16_t)((rx_data[9] << 8) | rx_data[10]);

    data->gyro_y =
        (int16_t)((rx_data[11] << 8) | rx_data[12]);

    data->gyro_z =
        (int16_t)((rx_data[13] << 8) | rx_data[14]);
    return HAL_OK;
}
