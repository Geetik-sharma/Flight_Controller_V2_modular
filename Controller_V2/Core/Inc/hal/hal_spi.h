#ifndef HAL_SPI_H
#define HAL_SPI_H

#include "main.h"
#include <stdint.h>

void FC_SPI_Select(void);
void FC_SPI_Deselect(void);

HAL_StatusTypeDef FC_SPI_WriteRead(
    const uint8_t *tx_data,
    uint8_t *rx_data,
    uint16_t length
);

HAL_StatusTypeDef FC_SPI_Write(
    const uint8_t *data,
    uint16_t length
);

HAL_StatusTypeDef FC_SPI_Read(
    uint8_t *data,
    uint16_t length
);


// const for only reading the tx buffer and do not modify it

#endif /* HAL_SPI_H */
