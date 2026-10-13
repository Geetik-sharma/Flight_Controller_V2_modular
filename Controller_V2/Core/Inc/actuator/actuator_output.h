/*
 * actuator_output.h
 *
 *  Created on: 27-Sept-2026
 *      Author: GeetikSharma
 */

#ifndef INC_ACTUATOR_ACTUATOR_OUTPUT_H_
#define INC_ACTUATOR_ACTUATOR_OUTPUT_H_

typedef struct
{
    float motor1;
    float motor2;
    float motor3;
    float motor4;
} ActuatorMotorCommand_t;

void ActuatorOutput_Init(void);

void ActuatorOutput_SetMotors(
    const ActuatorMotorCommand_t *command
);


#endif /* INC_ACTUATOR_ACTUATOR_OUTPUT_H_ */
