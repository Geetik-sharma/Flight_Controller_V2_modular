/*
 * attitude_setpoint.h
 *
 *  Created on: 04-Oct-2026
 *      Author: GeetikSharma
 */

#ifndef INC_CONTROLLER_ATTITUDE_SETPOINT_H_
#define INC_CONTROLLER_ATTITUDE_SETPOINT_H_

typedef struct
{
    float roll;       /* Desired roll angle, degrees */
    float pitch;      /* Desired pitch angle, degrees */
    float yaw;        /* Legacy yaw angle, degrees */
    float yaw_rate;   /* Desired yaw rate, degrees/second */
} AttitudeSetpoint_t;
void AttitudeSetpoint_Init(void);

void AttitudeSetpoint_Get(
    AttitudeSetpoint_t *setpoint
);

void AttitudeSetpoint_Set(
    float roll,
    float pitch,
    float yaw,
    float yaw_rate
);
#endif /* INC_CONTROLLER_ATTITUDE_SETPOINT_H_ */
