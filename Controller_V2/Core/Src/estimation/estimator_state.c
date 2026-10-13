/*
 * estimator_state.c
 *
 *  Created on: 25-Sept-2026
 *      Author: GeetikSharma
 */


#include "estimation/estimator_state.h"
#include "main.h"


/* =========================
 * Estimated State Storage
 * ========================= */

static EstimatorState_t estimator_state;


/* =========================
 * Initialization
 * ========================= */

void EstimatorState_Init(void)
{
    estimator_state.quaternion.q0 = 1.0f;
    estimator_state.quaternion.q1 = 0.0f;
    estimator_state.quaternion.q2 = 0.0f;
    estimator_state.quaternion.q3 = 0.0f;

    estimator_state.roll  = 0.0f;
    estimator_state.pitch = 0.0f;
    estimator_state.yaw   = 0.0f;

    estimator_state.gyro_x_dps = 0.0f;
    estimator_state.gyro_y_dps = 0.0f;
    estimator_state.gyro_z_dps = 0.0f;
}

void EstimatorState_Publish(
    const AHRS_Quaternion_t *quaternion,
    float roll,
    float pitch,
    float yaw,
    float gyro_x_dps,
    float gyro_y_dps,
    float gyro_z_dps
)
{
    if (quaternion == NULL)
    {
        return;
    }

    estimator_state.quaternion = *quaternion;

    estimator_state.roll  = roll;
    estimator_state.pitch = pitch;
    estimator_state.yaw   = yaw;

    estimator_state.gyro_x_dps = gyro_x_dps;
    estimator_state.gyro_y_dps = gyro_y_dps;
    estimator_state.gyro_z_dps = gyro_z_dps;
}
uint8_t EstimatorState_Get(
    EstimatorState_t *state
)
{
    if (state == NULL)
    {
        return 0U;
    }

    *state = estimator_state;

    return 1U;
}
