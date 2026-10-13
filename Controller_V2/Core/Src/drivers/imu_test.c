/*
 * imu_test.c
 *
 *  Created on: 17-Sept-2026
 *      Author: GeetikSharma
 */

#include"drivers/imu_test.h"
#include "hal/hal_spi.h"

uint8_t IMU_Test_WhoAmI(void)
{
    uint8_t tx_data[2];
    uint8_t rx_data[2];

    tx_data[0] = 0x75 | 0x80;   // WHO_AM_I + SPI read bit
    tx_data[1] = 0x00;          // Dummy byte

    rx_data[0] = 0x00;
    rx_data[1] = 0x00;

    FC_SPI_Select();

    FC_SPI_WriteRead(tx_data, rx_data, 2);

    FC_SPI_Deselect();

    return rx_data[1];
}
