/*
 * actuator_output.c
 *
 *  Created on: 27-Sept-2026
 *      Author: GeetikSharma
 */

#include "main.h"
#include "actuator/actuator_output.h"
#include "tim.h"
#include <stdbool.h>
#include "FreeRTOS.h"
#include "task.h"

static bool actuator_armed = false;
#define ACTUATOR_PWM_MIN_US    1000U
#define ACTUATOR_PWM_MAX_US    2000U



static uint16_t ActuatorOutput_ConvertToPWM(float command)
{
    uint16_t pwm_us;

    /* Final actuator safety boundary */
    if (command < 0.0f)
    {
        command = 0.0f;
    }
    else if (command > 1.0f)
    {
        command = 1.0f;
    }

    pwm_us = (uint16_t)(
        ACTUATOR_PWM_MIN_US +
        (command * (float)(ACTUATOR_PWM_MAX_US -
                           ACTUATOR_PWM_MIN_US))
    );

    return pwm_us;
}


static void ActuatorOutput_WriteMinimumPWM(void)
{
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, ACTUATOR_PWM_MIN_US);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, ACTUATOR_PWM_MIN_US);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, ACTUATOR_PWM_MIN_US);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, ACTUATOR_PWM_MIN_US);
}

void ActuatorOutput_Init(void)
{
    /* TIM3 PWM initialization is handled by CubeMX/HAL */
	    actuator_armed = false;
	    ActuatorOutput_WriteMinimumPWM();
	    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
	    //vTaskDelay(pdMS_TO_TICKS(3000));

}

void ActuatorOutput_SetArmed(bool armed)
{
    actuator_armed = armed;

    if (!actuator_armed)
    {
        ActuatorOutput_WriteMinimumPWM();
    }
}

uint16_t motor1_pwm;
uint16_t motor2_pwm;
uint16_t motor3_pwm;
uint16_t motor4_pwm;
void ActuatorOutput_SetMotors(
    const ActuatorMotorCommand_t *command
)
{

    if ((command == NULL) || (!actuator_armed))
    {
        ActuatorOutput_WriteMinimumPWM();
        return;
    }

    motor1_pwm = ActuatorOutput_ConvertToPWM(command->motor1);
    motor2_pwm = ActuatorOutput_ConvertToPWM(command->motor2);
    motor3_pwm = ActuatorOutput_ConvertToPWM(command->motor3);
    motor4_pwm = ActuatorOutput_ConvertToPWM(command->motor4);

    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, motor1_pwm);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, motor2_pwm);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, motor3_pwm);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, motor4_pwm);
}
