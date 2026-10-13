/*
 * attitude_controller.h
 *
 *  Created on: 04-Oct-2026
 *      Author: GeetikSharma
 */

#ifndef INC_CONTROLLER_ATTITUDE_CONTROLLER_H_
#define INC_CONTROLLER_ATTITUDE_CONTROLLER_H_

#include <stdint.h>

/* =========================
 * Attitude Controller State
 * ========================= */

typedef struct
{
    float kp_roll;
    float kp_pitch;
    float kp_yaw;

    float max_roll_rate;
    float max_pitch_rate;
    float max_yaw_rate;

} AttitudeController_t;


/* =========================
 * Attitude Controller Input
 * ========================= */

typedef struct
{
    float desired_roll;
    float desired_pitch;

    float current_roll;
    float current_pitch;

} AttitudeControllerInput_t;


/* =========================
 * Attitude Controller Output
 * ========================= */

typedef struct
{
    float desired_roll_rate;
    float desired_pitch_rate;
    float desired_yaw_rate;

} AttitudeControllerOutput_t;


/* =========================
 * Public Interface
 * ========================= */

extern AttitudeController_t attitude_controller;

void AttitudeController_Init(void);

AttitudeControllerOutput_t AttitudeController_Update(
    const AttitudeControllerInput_t *input
//    float dt
);
#endif /* INC_CONTROLLER_ATTITUDE_CONTROLLER_H_ */
