/*
 * controller_manager.c
 *
 *  Created on: 29-Sept-2026
 *      Author: GeetikSharma
 */


/*
 * controller_manager.c
 *
 * Created on: 29-Sept-2026
 * Author: GeetikSharma
 */

#include "controller/controller_manager.h"
#include "estimation/estimator_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "controller/mixer.h"
#include "controller/attitude_controller.h"
#include "controller/attitude_setpoint.h"
#include "rc/rc_input.h"

#define CONTROLLER_TASK_FREQUENCY_HZ    500U
#define CONTROLLER_DT_SEC               (1.0f / (float)CONTROLLER_TASK_FREQUENCY_HZ)
#define RC_MIN_US                  1000.0f
#define RC_CENTER_US               1500.0f
#define RC_MAX_US                  2000.0f

#define RC_ROLL_MAX_ANGLE_DEG      30.0f
#define RC_PITCH_MAX_ANGLE_DEG     30.0f
#define RC_YAW_MAX_RATE_DPS        200.0f

static void ControllerManagerTask(void *argument);



static RateControllerInput_t rate_controller_input;
static RateControllerOutput_t rate_controller_output;
//temporary
static MixerInput_t mixer_input;
static MixerOutput_t mixer_output;
static RateControllerFeedback_t rate_controller_feedback;

static AttitudeControllerInput_t attitude_controller_input;
static AttitudeControllerOutput_t attitude_controller_output;

static AttitudeSetpoint_t attitude_setpoint;

static RCInput_t rc_input;

volatile float debug_roll_rc_setpoint = 0.0f;
volatile float debug_pitch_rc_setpoint = 0.0f;
volatile float debug_yaw_rc_setpoint = 0.0f;

void ControllerManager_Init(void)
{
    RateController_Init();
    Mixer_Init();
    AttitudeController_Init();
    AttitudeSetpoint_Init();

    rate_controller_input.desired_roll_rate = 0.0f;
    rate_controller_input.desired_pitch_rate = 0.0f;
    rate_controller_input.desired_yaw_rate = 0.0f;

    rate_controller_input.current_roll_rate = 0.0f;
    rate_controller_input.current_pitch_rate = 0.0f;
    rate_controller_input.current_yaw_rate = 0.0f;

    rate_controller_output.roll = 0.0f;
    rate_controller_output.pitch = 0.0f;
    rate_controller_output.yaw = 0.0f;

    rate_controller_feedback.achieved_roll = 0.0f;
    rate_controller_feedback.achieved_pitch = 0.0f;
    rate_controller_feedback.achieved_yaw = 0.0f;



}



static float ControllerManager_MapRC(
    uint16_t pulse_us,
    float minimum,
    float center,
    float maximum,
    float negative_limit,
    float positive_limit)
{
    float pulse = (float)pulse_us;

    if (pulse < minimum)
    {
        pulse = minimum;
    }
    else if (pulse > maximum)
    {
        pulse = maximum;
    }

    if (pulse >= center)
    {
        return ((pulse - center) / (maximum - center))
               * positive_limit;
    }

    return ((pulse - center) / (center - minimum))
           * (negative_limit);
}



void ControllerManager_Update(float dt)
{

    EstimatorState_t estimator_state;
    AttitudeSetpoint_Get(&attitude_setpoint);

    RCInput_Get(&rc_input);

    if (rc_input.valid == 0U)
    {
        attitude_setpoint.roll = 0.0f;
        attitude_setpoint.pitch = 0.0f;
        attitude_setpoint.yaw_rate = 0.0f;
    }
    else
    {
        attitude_setpoint.roll = ControllerManager_MapRC(
            rc_input.roll,
            RC_MIN_US,
            RC_CENTER_US,
            RC_MAX_US,
            RC_ROLL_MAX_ANGLE_DEG,
            RC_ROLL_MAX_ANGLE_DEG);

        attitude_setpoint.pitch = ControllerManager_MapRC(
            rc_input.pitch,
            RC_MIN_US,
            RC_CENTER_US,
            RC_MAX_US,
            RC_PITCH_MAX_ANGLE_DEG,
            RC_PITCH_MAX_ANGLE_DEG);

        attitude_setpoint.yaw_rate = ControllerManager_MapRC(
            rc_input.yaw,
            RC_MIN_US,
            RC_CENTER_US,
            RC_MAX_US,
            RC_YAW_MAX_RATE_DPS,
            RC_YAW_MAX_RATE_DPS);
    }
    debug_roll_rc_setpoint = attitude_setpoint.roll;
    debug_pitch_rc_setpoint = attitude_setpoint.pitch;
    debug_yaw_rc_setpoint = attitude_setpoint.yaw_rate;
    if (!EstimatorState_Get(&estimator_state))
    {
        return;
    }


    attitude_controller_input.desired_roll = attitude_setpoint.roll;
    attitude_controller_input.desired_pitch = attitude_setpoint.pitch;

    attitude_controller_input.current_roll  = estimator_state.roll;
    attitude_controller_input.current_pitch = estimator_state.pitch;

    attitude_controller_output =
        AttitudeController_Update(&attitude_controller_input);



    rate_controller_input.desired_roll_rate =
        attitude_controller_output.desired_roll_rate;

    rate_controller_input.desired_pitch_rate =
        attitude_controller_output.desired_pitch_rate;

    rate_controller_input.desired_yaw_rate =
        attitude_setpoint.yaw_rate;


    rate_controller_input.current_roll_rate =
        estimator_state.gyro_x_dps;

    rate_controller_input.current_pitch_rate =
        estimator_state.gyro_y_dps;

    rate_controller_input.current_yaw_rate =
        estimator_state.gyro_z_dps;

    rate_controller_output = RateController_Update(
        &rate_controller_input,
        &rate_controller_feedback,
        dt
    );
    mixer_input.throttle = 0.50f;
    mixer_input.roll = rate_controller_output.roll;
    mixer_input.pitch = rate_controller_output.pitch;
    mixer_input.yaw = rate_controller_output.yaw;
    mixer_output = Mixer_Update(&mixer_input);

    rate_controller_feedback.achieved_roll =
        mixer_output.achieved.roll;

    rate_controller_feedback.achieved_pitch =
        mixer_output.achieved.pitch;

    rate_controller_feedback.achieved_yaw =
        mixer_output.achieved.yaw;
}

void ControllerManager_Start(void)
{
    BaseType_t status;

    status = xTaskCreate(
        ControllerManagerTask,
        "Controller",
        512,
        NULL,
        5,
        &xControllerManagerTaskHandle
    );

    if (status != pdPASS)
    {
        taskDISABLE_INTERRUPTS();

        while (1)
        {
            /* Controller Manager task creation failure */
        }
    }
}


static void ControllerManagerTask(void *argument)
{
    (void)argument;

    for (;;)
    {
        ulTaskNotifyTake(pdTRUE,portMAX_DELAY);
        RCInput_Update();
        ControllerManager_Update(CONTROLLER_DT_SEC);
    }
}
