#ifndef SENSORS_SENSOR_MANAGER_H_
#define SENSORS_SENSOR_MANAGER_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    int16_t raw_gx;
    int16_t raw_gy;
    int16_t raw_gz;
    float bias_gx;
    float bias_gy;
    float bias_gz;
    float rate_gx_dps;
    float rate_gy_dps;
    float rate_gz_dps;

    float peak_rate_gx_dps;
    float peak_rate_gy_dps;
    float peak_rate_gz_dps;

    int16_t peak_raw_gx;
    int16_t peak_raw_gy;
    int16_t peak_raw_gz;

} GyroDebugSnapshot_t;

extern volatile GyroDebugSnapshot_t gyro_debug;

void SensorManager_Start(void);

#ifdef __cplusplus
}
#endif

#endif /* SENSORS_SENSOR_MANAGER_H_ */
