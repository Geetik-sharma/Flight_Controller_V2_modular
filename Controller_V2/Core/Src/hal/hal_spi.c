
/*
 * hal_spi.c
 *
 * Created on: 17-Sept-2026
 * Author: GeetikSharma
 */

#include "hal/hal_spi.h"
#include "spi.h"

void FC_SPI_Select(void)
{
    HAL_GPIO_WritePin(NCS_GPIO_Port, NCS_Pin, GPIO_PIN_RESET);
}

void FC_SPI_Deselect(void)
{
    HAL_GPIO_WritePin(NCS_GPIO_Port, NCS_Pin, GPIO_PIN_SET);
}

HAL_StatusTypeDef FC_SPI_WriteRead(
    const uint8_t *tx_data,
    uint8_t *rx_data,
    uint16_t length)
{
    return HAL_SPI_TransmitReceive(
        &hspi1,
        (uint8_t *)tx_data,
        rx_data,
        length,
        100
    );
}

HAL_StatusTypeDef FC_SPI_Write(
    const uint8_t *data,
    uint16_t length)
{
    return HAL_SPI_Transmit(
        &hspi1,
        (uint8_t *)data,
        length,
        100
    );
}

HAL_StatusTypeDef FC_SPI_Read(
    uint8_t *data,
    uint16_t length)
{
    return HAL_SPI_Receive(
        &hspi1,
        data,
        length,
        100
    );
}
