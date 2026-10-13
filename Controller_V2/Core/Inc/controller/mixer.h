/*
 * mixer.h
 *
 *  Created on: 27-Sept-2026
 *      Author: GeetikSharma
 */

#ifndef INC_CONTROLLER_MIXER_H_
#define INC_CONTROLLER_MIXER_H_
#include <stdint.h>
typedef struct
{
    float throttle;
    float roll;
    float pitch;
    float yaw;
} MixerInput_t;

typedef struct
{
    float roll;
    float pitch;
    float yaw;
} MixerAchievedControl_t;

typedef struct
{
    float motor1;
    float motor2;
    float motor3;
    float motor4;
    MixerAchievedControl_t achieved;

} MixerOutput_t;



void Mixer_Init(void);

MixerOutput_t Mixer_Update(const MixerInput_t *input);

#endif /* INC_CONTROLLER_MIXER_H_ */
