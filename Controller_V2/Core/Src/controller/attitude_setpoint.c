
/*
 * attitude_setpoint.c
 *
 *  Created on: 04-Oct-2026
 *      Author: GeetikSharma
 */




#include "controller/attitude_setpoint.h"
#include "main.h"
static AttitudeSetpoint_t attitude_setpoint;

void AttitudeSetpoint_Init(void)
{
    attitude_setpoint.roll = 0.0f;
    attitude_setpoint.pitch = 0.0f;
    attitude_setpoint.yaw = 0.0f;
    attitude_setpoint.yaw_rate=0.0f;
}

void AttitudeSetpoint_Get(
    AttitudeSetpoint_t *setpoint
)
{
    if (setpoint == NULL)
    {
        return;
    }

    *setpoint = attitude_setpoint;
}

void AttitudeSetpoint_Set(
    float roll,
    float pitch,
    float yaw,
    float yaw_rate
)
{
    attitude_setpoint.roll = roll;
    attitude_setpoint.pitch = pitch;
    attitude_setpoint.yaw = yaw;
    attitude_setpoint.yaw_rate = yaw_rate;
}
