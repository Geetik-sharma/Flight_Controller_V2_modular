/*
 * system_bus.h
 *
 *  Created on: 24-Sept-2026
 *      Author: GeetikSharma
 */

#ifndef INC_SYSTEM_DATA_BUS_H_
#define INC_SYSTEM_DATA_BUS_H_

#include "sensors/sensor_data.h"
#include <stdint.h>


/* =========================
 * Sensor Data Bus
 * ========================= */

void DataBus_PublishSensorData(
    const SensorPhysicalData_t *data
);

uint8_t DataBus_GetSensorData(
    SensorPhysicalData_t *data
);

#endif /* INC_SYSTEM_DATA_BUS_H_ */
