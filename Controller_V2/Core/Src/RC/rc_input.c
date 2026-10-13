/*
 * rc_input.h
 *
 *  Created on: 07-Oct-2026
 *      Author: GeetikSharma
 */


#include "rc/rc_input.h"
#include "rc/ppm_decoder.h"
#include <stddef.h>

static RCInput_t rc_input;

void RCInput_Init(void)
{
    rc_input.roll = 1500U;
    rc_input.pitch = 1500U;
    rc_input.throttle = 1000U;
    rc_input.yaw = 1500U;
    rc_input.aux1 = 1000U;
    rc_input.aux2 = 1000U;
    rc_input.valid = 0U;
}

void RCInput_Update(void)
{
    PPMFrame_t frame;

    PPMDecoder_GetFrame(&frame);

    if ((frame.frame_valid == 0U) ||
        (frame.channel_count != PPM_MAX_CHANNELS))
    {
        rc_input.valid = 0U;
        return;
    }

    rc_input.roll     = frame.channel[0];
    rc_input.pitch    = frame.channel[1];
    rc_input.throttle = frame.channel[2];
    rc_input.yaw      = frame.channel[3];
    rc_input.aux1     = frame.channel[4];
    rc_input.aux2     = frame.channel[5];

    rc_input.valid = 1U;
}

void RCInput_Get(RCInput_t *input)
{
    if (input == NULL)
    {
        return;
    }

    *input = rc_input;
}
