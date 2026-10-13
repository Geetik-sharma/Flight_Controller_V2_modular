


#include "estimation/ahrs.h"
#include <math.h>

#define AHRS_KP    2.0f
#define AHRS_KI    0.005f
static float integral_error_x = 0.0f;
static float integral_error_y = 0.0f;
static float integral_error_z = 0.0f;

static void AHRS_UpdateEulerAngles(void);

static AHRS_Quaternion_t quaternion;


/* =========================
 * AHRS State
 * ========================= */

static AHRS_Attitude_t attitude;


/* =========================
 * Initialization
 * ========================= */

void AHRS_Init(void)
{
    quaternion.q0 = 1.0f;
    quaternion.q1 = 0.0f;
    quaternion.q2 = 0.0f;
    quaternion.q3 = 0.0f;

    attitude.roll  = 0.0f;
    attitude.pitch = 0.0f;
    attitude.yaw   = 0.0f;
}

/* =========================
 * Get Attitude
 * ========================= */

void AHRS_GetAttitude(
    AHRS_Attitude_t *output
)
{
    if (output == NULL)
    {
        return;
    }

    *output = attitude;
}

static void AHRS_NormalizeQuaternion(void)
{
    float norm;

    norm = sqrtf(
        quaternion.q0 * quaternion.q0 +
        quaternion.q1 * quaternion.q1 +
        quaternion.q2 * quaternion.q2 +
        quaternion.q3 * quaternion.q3
    );

    if (norm > 0.0f)
    {
        quaternion.q0 /= norm;
        quaternion.q1 /= norm;
        quaternion.q2 /= norm;
        quaternion.q3 /= norm;
    }
}

static uint8_t AHRS_NormalizeAccelerometer(
    float *ax,
    float *ay,
    float *az
)
{
    float norm;

    norm = sqrtf(
        (*ax * *ax) +
        (*ay * *ay) +
        (*az * *az)
    );

    if (norm <= 0.0f)
    {
        return 0U;
    }

    *ax /= norm;
    *ay /= norm;
    *az /= norm;

    return 1U;
}

static void AHRS_IntegrateGyro(
    float gx,
    float gy,
    float gz,
    float dt
)
{
    float q0_dot;
    float q1_dot;
    float q2_dot;
    float q3_dot;

    q0_dot = -0.5f * (
        quaternion.q1 * gx +
        quaternion.q2 * gy +
        quaternion.q3 * gz
    );

    q1_dot =  0.5f * (
        quaternion.q0 * gx +
        quaternion.q2 * gz -
        quaternion.q3 * gy
    );

    q2_dot =  0.5f * (
        quaternion.q0 * gy -
        quaternion.q1 * gz +
        quaternion.q3 * gx
    );

    q3_dot =  0.5f * (
        quaternion.q0 * gz +
        quaternion.q1 * gy -
        quaternion.q2 * gx
    );

    quaternion.q0 += q0_dot * dt;
    quaternion.q1 += q1_dot * dt;
    quaternion.q2 += q2_dot * dt;
    quaternion.q3 += q3_dot * dt;

    AHRS_NormalizeQuaternion();
}


static void AHRS_UpdateEulerAngles(void)
{
    float q0 = quaternion.q0;
    float q1 = quaternion.q1;
    float q2 = quaternion.q2;
    float q3 = quaternion.q3;

    float pitch_argument;


    /* =========================
     * Roll
     * ========================= */

    attitude.roll =
        atan2f(
            2.0f * (q0 * q1 + q2 * q3),
            1.0f - 2.0f * (q1 * q1 + q2 * q2)
        );


    /* =========================
     * Pitch
     * ========================= */

    pitch_argument =
        2.0f * (q0 * q2 - q3 * q1);

    if (pitch_argument > 1.0f)
    {
        pitch_argument = 1.0f;
    }
    else if (pitch_argument < -1.0f)
    {
        pitch_argument = -1.0f;
    }

    attitude.pitch =
        asinf(pitch_argument);


    /* =========================
     * Yaw
     * ========================= */

    attitude.yaw =
        atan2f(
            2.0f * (q0 * q3 + q1 * q2),
            1.0f - 2.0f * (q2 * q2 + q3 * q3)
        );


    /* =========================
     * Convert to Degrees
     * ========================= */

    attitude.roll  *= 180.0f / 3.14159265f;
    attitude.pitch *= 180.0f / 3.14159265f;
    attitude.yaw   *= 180.0f / 3.14159265f;
}

static void AHRS_ComputeError(
    float ax,
    float ay,
    float az,
    float *error_x,
    float *error_y,
    float *error_z
)
{
    float q0 = quaternion.q0;
    float q1 = quaternion.q1;
    float q2 = quaternion.q2;
    float q3 = quaternion.q3;

    float estimated_x;
    float estimated_y;
    float estimated_z;

    /*
     * Estimated gravity direction
     * from the current quaternion.
     */

    estimated_x =
        2.0f * (q1 * q3 - q0 * q2);

    estimated_y =
        2.0f * (q0 * q1 + q2 * q3);

    estimated_z =
        q0 * q0 -
        q1 * q1 -
        q2 * q2 +
        q3 * q3;

    /*
     * Cross product:
     *
     * measured gravity ×
     * estimated gravity
     */

    *error_x =
        (ay * estimated_z) -
        (az * estimated_y);

    *error_y =
        (az * estimated_x) -
        (ax * estimated_z);

    *error_z =
        (ax * estimated_y) -
        (ay * estimated_x);
}

static void AHRS_ApplyProportionalCorrection(
    float *gx,
    float *gy,
    float *gz,
    float error_x,
    float error_y,
    float error_z
)
{
    *gx += AHRS_KP * error_x;
    *gy += AHRS_KP * error_y;
    *gz += AHRS_KP * error_z;
}

static void AHRS_ApplyIntegralCorrection(
    float *gx,
    float *gy,
    float *gz,
    float error_x,
    float error_y,
    float error_z,
    float dt
)
{
    integral_error_x += error_x * dt;
    integral_error_y += error_y * dt;
    integral_error_z += error_z * dt;

    *gx += AHRS_KI * integral_error_x;
    *gy += AHRS_KI * integral_error_y;
    *gz += AHRS_KI * integral_error_z;
}

void AHRS_Update(
    const SensorPhysicalData_t *sensor_data,
    float dt
)
{
    float gx;
    float gy;
    float gz;

    float ax;
    float ay;
    float az;

    float error_x;
    float error_y;
    float error_z;

    if (sensor_data == NULL)
    {
        return;
    }

    if (dt <= 0.0f)
    {
        return;
    }

    /* =========================
     * Gyroscope
     * ========================= */

    gx = sensor_data->gyro_x_dps *
         (3.14159265f / 180.0f);

    gy = sensor_data->gyro_y_dps *
         (3.14159265f / 180.0f);

    gz = sensor_data->gyro_z_dps *
         (3.14159265f / 180.0f);


    /* =========================
     * Accelerometer
     * ========================= */

    ax = sensor_data->accel_x_g;
    ay = sensor_data->accel_y_g;
    az = sensor_data->accel_z_g;

    if (AHRS_NormalizeAccelerometer(
            &ax,
            &ay,
            &az) == 0U)
    {
        return;
    }


    /* =========================
     * Compute Gravity Error
     * ========================= */

    AHRS_ComputeError(
        ax,
        ay,
        az,
        &error_x,
        &error_y,
        &error_z
    );


    /* =========================
     * Mahony Correction
     * ========================= */

    AHRS_ApplyIntegralCorrection(
        &gx,
        &gy,
        &gz,
        error_x,
        error_y,
        error_z,
        dt
    );

    AHRS_ApplyProportionalCorrection(
        &gx,
        &gy,
        &gz,
        error_x,
        error_y,
        error_z
    );


    /* =========================
     * Quaternion Integration
     * ========================= */

    AHRS_IntegrateGyro(
        gx,
        gy,
        gz,
        dt
    );


    /* =========================
     * Euler Angles
     * ========================= */

    AHRS_UpdateEulerAngles();
}


void AHRS_GetQuaternion(
    AHRS_Quaternion_t *output
)
{
    if (output == NULL)
    {
        return;
    }

    output->q0 = quaternion.q0;
    output->q1 = quaternion.q1;
    output->q2 = quaternion.q2;
    output->q3 = quaternion.q3;
}
