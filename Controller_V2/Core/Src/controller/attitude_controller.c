/*
 * attitude_controller.c
 *
 * Created on: 04-Oct-2026
 *     Author: GeetikSharma
 */

#include "controller/attitude_controller.h"
#include "main.h"




#define ATTITUDE_RATE_MIN    (-200.0f)
#define ATTITUDE_RATE_MAX    ( 200.0f)



AttitudeController_t attitude_controller;




static float AttitudeController_Clamp(
    float value,
    float min,
    float max
)
{
    if (value > max)
    {
        return max;
    }

    if (value < min)
    {
        return min;
    }

    return value;
}



void AttitudeController_Init(void)
{
    attitude_controller.kp_roll  = 4.0f;
    attitude_controller.kp_pitch = 4.0f;
    attitude_controller.kp_yaw   = 4.0f;

    attitude_controller.max_roll_rate  = ATTITUDE_RATE_MAX;
    attitude_controller.max_pitch_rate = ATTITUDE_RATE_MAX;
    attitude_controller.max_yaw_rate   = ATTITUDE_RATE_MAX;
}



AttitudeControllerOutput_t AttitudeController_Update(
    const AttitudeControllerInput_t *input
)
{
    AttitudeControllerOutput_t output;

    float roll_error;
    float pitch_error;

    if (input == NULL)
    {
        output.desired_roll_rate  = 0.0f;
        output.desired_pitch_rate = 0.0f;
        output.desired_yaw_rate   = 0.0f;

        return output;
    }

    /* =========================
     * Attitude Errors
     * ========================= */

    roll_error =
        input->desired_roll -
        input->current_roll;

    pitch_error =
        input->desired_pitch -
        input->current_pitch;



    /* P Controller */

    output.desired_roll_rate =
        attitude_controller.kp_roll *
        roll_error;

    output.desired_pitch_rate =
        attitude_controller.kp_pitch *
        pitch_error;


    /* =========================
     * Rate Limiting
     * ========================= */

    output.desired_roll_rate =
        AttitudeController_Clamp(
            output.desired_roll_rate,
            -attitude_controller.max_roll_rate,
             attitude_controller.max_roll_rate
        );

    output.desired_pitch_rate =
        AttitudeController_Clamp(
            output.desired_pitch_rate,
            -attitude_controller.max_pitch_rate,
             attitude_controller.max_pitch_rate
        );


    return output;
}/*
 * attitude_controller.c
 *
 *  Created on: 04-Oct-2026
 *      Author: GeetikSharma
 */


