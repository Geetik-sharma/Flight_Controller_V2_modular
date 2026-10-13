/*
 * rate_controller.c
 *
 *  Created on: 26-Sept-2026
 *      Author: GeetikSharma
 */


#include "controller/rate_controller.h"
#include "main.h"
#define RATE_PID_OUTPUT_MIN   (-1.0f)
#define RATE_PID_OUTPUT_MAX   ( 1.0f)
#define RATE_PID_OUTPUT_SCALE    200.0f


RateController_t rate_controller;


void RateController_Init(void)
{

	rate_controller.roll.kp = 1.0f;
	rate_controller.roll.ki = 0.0f;
	rate_controller.roll.kd = 0.0f;
	rate_controller.roll.initialized = 0U;

	rate_controller.pitch.kp = 1.0f;
	rate_controller.pitch.ki = 0.0f;
	rate_controller.pitch.kd = 0.0f;
	rate_controller.pitch.initialized = 0U;

	rate_controller.yaw.kp = 1.0f;
	rate_controller.yaw.ki = 0.0f;
	rate_controller.yaw.kd = 0.0f;
	rate_controller.yaw.initialized = 0U;


    /* Roll */
    rate_controller.roll.integral = 0.0f;
    rate_controller.roll.previous_error = 0.0f;

    /* Pitch */
    rate_controller.pitch.integral = 0.0f;
    rate_controller.pitch.previous_error = 0.0f;

    /* Yaw */
    rate_controller.yaw.integral = 0.0f;
    rate_controller.yaw.previous_error = 0.0f;

    rate_controller.roll.integral_limit = 20.0f;
    rate_controller.pitch.integral_limit = 20.0f;
    rate_controller.yaw.integral_limit = 20.0f;

    rate_controller.roll.derivative_limit = 30.0f;
    rate_controller.pitch.derivative_limit = 30.0f;
    rate_controller.yaw.derivative_limit = 30.0f;

    rate_controller.roll.previous_measurement = 0.0f;
    rate_controller.pitch.previous_measurement = 0.0f;
    rate_controller.yaw.previous_measurement = 0.0f;

    rate_controller.roll.previous_output = 0.0f;
    rate_controller.pitch.previous_output = 0.0f;
    rate_controller.yaw.previous_output = 0.0f;
}




float RatePID_Update(
    RatePID_t *pid,
    float setpoint,
    float measurement,
    float achieved_output,
    float dt
)
{
    float error;
    float proportional;
    float derivative;
    float output;

    error = setpoint - measurement;

    float control_error;
    uint8_t allow_integral;

    if (pid->initialized == 0U)
    {
        pid->previous_error = error;
        pid->previous_measurement = measurement;
        pid->initialized = 1U;
    }

    /* Determine whether actuator limitation opposes integral growth */
    control_error = pid->previous_output - achieved_output;
    allow_integral = 1U;

    if ((control_error > 0.0f) && (error > 0.0f))
    {
        allow_integral = 0U;
    }
    else if ((control_error < 0.0f) && (error < 0.0f))
    {
        allow_integral = 0U;
    }
    /* Proportional term */
    proportional = pid->kp * error;

    /* Integral term */
    if (allow_integral != 0U)
    {
        pid->integral +=
            pid->ki *
            (pid->previous_error + error) *
            0.5f *
            dt;
    }

    /* Integral limit */
    if (pid->integral > pid->integral_limit)
    {
        pid->integral = pid->integral_limit;
    }
    else if (pid->integral < -pid->integral_limit)
    {
        pid->integral = -pid->integral_limit;
    }
    /* Derivative term */
    derivative = 0.0f;

    if (dt > 0.0f)
    {
    	derivative = -pid->kd * (measurement - pid->previous_measurement) /dt;
    	if (derivative > pid->derivative_limit)
    	{
    	    derivative = pid->derivative_limit;
    	}
    	else if (derivative < -pid->derivative_limit)
    	{
    	    derivative = -pid->derivative_limit;
    	}
    }

    /* PID output */
    output = proportional
           + pid->integral
           + derivative;
    /* Store current error */
    pid->previous_error = error;
    pid->previous_measurement = measurement;
    return output;
}

float roll_achieved_error;
float pitch_achieved_error;
float yaw_achieved_error;

RateControllerOutput_t RateController_Update(
    const RateControllerInput_t *input,
    const RateControllerFeedback_t *feedback,
    float dt
)
{
    RateControllerOutput_t output;

    if (input == NULL || feedback == NULL)
    {
        output.roll = 0.0f;
        output.pitch = 0.0f;
        output.yaw = 0.0f;

        return output;
    }

    output.roll = RatePID_Update(
        &rate_controller.roll,
        input->desired_roll_rate,
        input->current_roll_rate,
        feedback->achieved_roll,
        dt
    );

    output.pitch = RatePID_Update(
        &rate_controller.pitch,
        input->desired_pitch_rate,
        input->current_pitch_rate,
        feedback->achieved_pitch,
        dt
    );

    output.yaw = RatePID_Update(
        &rate_controller.yaw,
        input->desired_yaw_rate,
        input->current_yaw_rate,
        feedback->achieved_yaw,
        dt
    );

    //normalizing PID output

    output.roll /= RATE_PID_OUTPUT_SCALE;
    output.pitch /= RATE_PID_OUTPUT_SCALE;
    output.yaw /= RATE_PID_OUTPUT_SCALE;

    //clamping the normalized output
    if (output.roll > RATE_PID_OUTPUT_MAX)
        output.roll = RATE_PID_OUTPUT_MAX;
    else if (output.roll < RATE_PID_OUTPUT_MIN)
        output.roll = RATE_PID_OUTPUT_MIN;

    if (output.pitch > RATE_PID_OUTPUT_MAX)
        output.pitch = RATE_PID_OUTPUT_MAX;
    else if (output.pitch < RATE_PID_OUTPUT_MIN)
        output.pitch = RATE_PID_OUTPUT_MIN;

    if (output.yaw > RATE_PID_OUTPUT_MAX)
        output.yaw = RATE_PID_OUTPUT_MAX;
    else if (output.yaw < RATE_PID_OUTPUT_MIN)
        output.yaw = RATE_PID_OUTPUT_MIN;


    rate_controller.roll.previous_output = output.roll;
    rate_controller.pitch.previous_output = output.pitch;
    rate_controller.yaw.previous_output = output.yaw;


    roll_achieved_error =
        output.roll - feedback->achieved_roll;

    pitch_achieved_error =
        output.pitch - feedback->achieved_pitch;

    yaw_achieved_error =
        output.yaw - feedback->achieved_yaw;


    return output;
}
