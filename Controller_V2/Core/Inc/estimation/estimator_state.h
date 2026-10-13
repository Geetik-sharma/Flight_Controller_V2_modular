/*
 * estimator_state.h
 *
 *  Created on: 25-Sept-2026
 *      Author: GeetikSharma
 */

#ifndef INC_ESTIMATION_ESTIMATOR_STATE_H_
#define INC_ESTIMATION_ESTIMATOR_STATE_H_

#include <stdint.h>

#include "estimation/ahrs.h"


/* =========================
 * Estimated State
 * ========================= */

typedef struct
{
    AHRS_Quaternion_t quaternion;

    float roll;
    float pitch;
    float yaw;
    float gyro_x_dps;
    float gyro_y_dps;
    float gyro_z_dps;
} EstimatorState_t;


/* =========================
 * Initialization
 * ========================= */

void EstimatorState_Init(void);


/* =========================
 * Publish State
 * ========================= */

void EstimatorState_Publish(
    const AHRS_Quaternion_t *quaternion,
    float roll,
    float pitch,
    float yaw,
    float gyro_x_dps,
    float gyro_y_dps,
    float gyro_z_dps
);

/* =========================
 * Get State
 * ========================= */

uint8_t EstimatorState_Get(
    EstimatorState_t *state
);

#endif /* INC_ESTIMATION_ESTIMATOR_STATE_H_ */
