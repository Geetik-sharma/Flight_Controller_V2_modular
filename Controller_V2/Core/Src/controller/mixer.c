/*
 * mixer.c
 *
 *  Created on: 27-Sept-2026
 *      Author: GeetikSharma
 */


#include "controller/mixer.h"
#include "main.h"

#define MIXER_OUTPUT_MIN    0.0f
#define MIXER_OUTPUT_MAX    1.0f
#define MIXER_SATURATION_EPSILON    0.001f


void Mixer_Init(void)
{
    /* No persistent mixer state yet */
}


MixerOutput_t Mixer_Update(const MixerInput_t *input)
{
    MixerOutput_t output;

    if (input == NULL)
    {
        output.motor1 = 0.0f;
        output.motor2 = 0.0f;
        output.motor3 = 0.0f;
        output.motor4 = 0.0f;

        return output;
    }

    /*
     * Motor configuration:
     *
     * M1: Front Right - CCW
     * M2: Back Left   - CCW
     * M3: Front Left  - CW
     * M4: Back Right  - CW
     *
     * Positive pitch = nose up
     * Positive yaw   = CCW
     */

    output.motor1 =
        input->throttle
        - input->roll
        - input->pitch
        - input->yaw;

    output.motor2 =
        input->throttle
        + input->roll
        + input->pitch
        - input->yaw;

    output.motor3 =
        input->throttle
        + input->roll
        - input->pitch
        + input->yaw;

    output.motor4 =
        input->throttle
        - input->roll
        + input->pitch
        + input->yaw;


    /* Motor output limits */

    if (output.motor1 > MIXER_OUTPUT_MAX)
        output.motor1 = MIXER_OUTPUT_MAX;
    else if (output.motor1 < MIXER_OUTPUT_MIN)
        output.motor1 = MIXER_OUTPUT_MIN;

    if (output.motor2 > MIXER_OUTPUT_MAX)
        output.motor2 = MIXER_OUTPUT_MAX;
    else if (output.motor2 < MIXER_OUTPUT_MIN)
        output.motor2 = MIXER_OUTPUT_MIN;

    if (output.motor3 > MIXER_OUTPUT_MAX)
        output.motor3 = MIXER_OUTPUT_MAX;
    else if (output.motor3 < MIXER_OUTPUT_MIN)
        output.motor3 = MIXER_OUTPUT_MIN;

    if (output.motor4 > MIXER_OUTPUT_MAX)
        output.motor4 = MIXER_OUTPUT_MAX;
    else if (output.motor4 < MIXER_OUTPUT_MIN)
        output.motor4 = MIXER_OUTPUT_MIN;
    /* Reconstruct achieved control from constrained motor outputs */

    output.achieved.roll =
          (-output.motor1
           + output.motor2
           + output.motor3
           - output.motor4) * 0.25f;

    output.achieved.pitch =
          (-output.motor1
           + output.motor2
           - output.motor3
           + output.motor4) * 0.25f;

    output.achieved.yaw =
          (-output.motor1
           - output.motor2
           + output.motor3
           + output.motor4) * 0.25f;

    return output;
}
