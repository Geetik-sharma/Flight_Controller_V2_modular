/*
 * rc_input.h
 *
 *  Created on: 07-Oct-2026
 *      Author: GeetikSharma
 */

#ifndef INC_RC_RC_INPUT_H_
#define INC_RC_RC_INPUT_H_

#include <stdint.h>

typedef struct
{
    uint16_t roll;
    uint16_t pitch;
    uint16_t throttle;
    uint16_t yaw;
    uint16_t aux1;
    uint16_t aux2;

    uint8_t valid;
} RCInput_t;

void RCInput_Init(void);

void RCInput_Update(void);

void RCInput_Get(RCInput_t *input);


#endif /* INC_RC_RC_INPUT_H_ */
