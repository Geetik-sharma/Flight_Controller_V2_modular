/*
 * rate_controller.h
 *
 *  Created on: 26-Sept-2026
 *      Author: GeetikSharma
 */

#ifndef INC_CONTROLLER_RATE_CONTROLLER_H_
#define INC_CONTROLLER_RATE_CONTROLLER_H_


#include <stdint.h>
typedef struct
{
    float kp;
    float ki;
    float kd;

    float integral;
    float previous_error;
    float previous_output;

    float integral_limit;
    float derivative_limit;
    float previous_measurement;
    uint8_t initialized;


} RatePID_t;

typedef struct
{
    float achieved_roll;
    float achieved_pitch;
    float achieved_yaw;
} RateControllerFeedback_t;

typedef struct
{
    RatePID_t roll;
    RatePID_t pitch;
    RatePID_t yaw;

} RateController_t;

typedef struct
{
    float desired_roll_rate;
    float desired_pitch_rate;
    float desired_yaw_rate;

    float current_roll_rate;
    float current_pitch_rate;
    float current_yaw_rate;

} RateControllerInput_t;


typedef struct
{
    float roll;
    float pitch;
    float yaw;

} RateControllerOutput_t;


extern RateController_t rate_controller;
void RateController_Init(void);

float RatePID_Update(
    RatePID_t *pid,
    float setpoint,
    float measurement,
    float achieved_output,
    float dt
);
RateControllerOutput_t RateController_Update(
    const RateControllerInput_t *input,
    const RateControllerFeedback_t *feedback,
    float dt
);

#endif /* INC_CONTROLLER_RATE_CONTROLLER_H_ */
