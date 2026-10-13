/*
 * sensor_data.h
 *
 *  Created on: 19-Sept-2026
 *      Author: GeetikSharma
 */

#ifndef INC_SENSORS_SENSOR_DATA_H_
#define INC_SENSORS_SENSOR_DATA_H_


typedef struct
{
    float accel_x_g;
    float accel_y_g;
    float accel_z_g;

    float gyro_x_dps;
    float gyro_y_dps;
    float gyro_z_dps;

    float temperature_c;

} SensorPhysicalData_t;


#endif /* INC_SENSORS_SENSOR_DATA_H_ */
