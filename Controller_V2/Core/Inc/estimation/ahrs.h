/*
 * ahrs.h
 *
 *  Created on: 24-Sept-2026
 *      Author: GeetikSharma
 */

#ifndef INC_ESTIMATION_AHRS_H_
#define INC_ESTIMATION_AHRS_H_

#include <stdint.h>
#include "sensors/sensor_data.h"


/* =========================
 * AHRS State
 * ========================= */

typedef struct
{
    float roll;
    float pitch;
    float yaw;

} AHRS_Attitude_t;

typedef struct
{
    float q0;
    float q1;
    float q2;
    float q3;

} AHRS_Quaternion_t;

/* =========================
 * Initialization
 * ========================= */

void AHRS_Init(void);


/* =========================
 * AHRS Update
 * ========================= */

void AHRS_Update(
    const SensorPhysicalData_t *sensor_data,
    float dt
);


/* =========================
 * Get Attitude
 * ========================= */

void AHRS_GetAttitude(
    AHRS_Attitude_t *attitude
);

void AHRS_GetQuaternion(
    AHRS_Quaternion_t *quaternion
);

#endif /* INC_ESTIMATION_AHRS_H_ */
