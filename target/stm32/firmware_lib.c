/*
 * C Firmware Library Implementation
 * SITL (Software-in-the-Loop) wrapper for firmware
 */

#include "firmware_lib.h"
#include "estimate.h"
#include "control.h"
#include "fc_math.h"
#include "types.h"
#include <string.h>

/* ===== Global State ===== */
static MahonyEstimator estimator;
static StabilityController controller;
static StateEstimate last_state;

/* ===== Implementation ===== */

void firmware_lib_init(void) {
    /* Initialize estimator (Mahony AHRS) */
    mahony_init(&estimator, 0.5f, 0.001f);
    
    /* Initialize controller */
    stability_init(&controller);
    
    /* Set initial setpoints */
    stability_set_altitude(&controller, 1.0f);
    stability_set_attitude(&controller, 0.0f, 0.0f, 0.0f);
    
    /* Clear last state */
    memset(&last_state, 0, sizeof(StateEstimate));
    last_state.orientation = quat_identity();
}

void firmware_lib_reset(void) {
    mahony_reset(&estimator);
    stability_reset(&controller);
    memset(&last_state, 0, sizeof(StateEstimate));
    last_state.orientation = quat_identity();
}

void firmware_lib_update(
    float accel_x, float accel_y, float accel_z,
    float gyro_x, float gyro_y, float gyro_z,
    float altitude, float dt,
    float *output)
{
    /* Build sensor readings */
    SensorReadings readings;
    readings.imu.accel[0] = accel_x;
    readings.imu.accel[1] = accel_y;
    readings.imu.accel[2] = accel_z;
    readings.imu.gyro[0] = gyro_x;
    readings.imu.gyro[1] = gyro_y;
    readings.imu.gyro[2] = gyro_z;
    readings.imu.timestamp = 0.0f;  /* Not used in SITL */
    readings.altitude = altitude;
    readings.timestamp = 0.0f;
    
    /* Update estimator */
    last_state = mahony_update(&estimator, &readings, dt);
    
    /* Update controller */
    stability_update(&controller, &last_state, dt, output);
}

void firmware_lib_get_state(
    float *position,
    float *velocity,
    float *orientation,
    float *angular_velocity)
{
    position[0] = last_state.position.x;
    position[1] = last_state.position.y;
    position[2] = last_state.position.z;
    
    velocity[0] = last_state.velocity.x;
    velocity[1] = last_state.velocity.y;
    velocity[2] = last_state.velocity.z;
    
    orientation[0] = last_state.orientation.w;
    orientation[1] = last_state.orientation.x;
    orientation[2] = last_state.orientation.y;
    orientation[3] = last_state.orientation.z;
    
    angular_velocity[0] = last_state.angular_velocity.x;
    angular_velocity[1] = last_state.angular_velocity.y;
    angular_velocity[2] = last_state.angular_velocity.z;
}

void firmware_lib_set_altitude(float z) {
    stability_set_altitude(&controller, z);
}

void firmware_lib_set_attitude(float roll, float pitch, float yaw) {
    stability_set_attitude(&controller, roll, pitch, yaw);
}

int firmware_lib_is_initialized(void) {
    return mahony_is_initialized(&estimator) ? 1 : 0;
}

float firmware_lib_init_progress(void) {
    return mahony_init_progress(&estimator);
}

