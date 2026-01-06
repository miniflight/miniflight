/*
 * C Firmware Library Interface
 * Exposes firmware for Software-in-the-Loop (SITL) testing
 */
#ifndef FIRMWARE_LIB_H
#define FIRMWARE_LIB_H

#include <stdint.h>

/* ===== External API for Python/SITL ===== */

/* Initialize firmware */
void firmware_lib_init(void);

/* Reset firmware state */
void firmware_lib_reset(void);

/* Single control loop iteration */
/* Input: 6 floats (accel xyz, gyro xyz), 1 float (altitude), 1 float (dt) */
/* Output: 4 floats (thrust, roll_cmd, pitch_cmd, yaw_cmd) */
void firmware_lib_update(
    float accel_x, float accel_y, float accel_z,
    float gyro_x, float gyro_y, float gyro_z,
    float altitude, float dt,
    float *output  /* output[4] = [thrust, roll, pitch, yaw] */
);

/* Get current state estimate */
void firmware_lib_get_state(
    float *position,        /* output[3] = [x, y, z] */
    float *velocity,        /* output[3] = [vx, vy, vz] */
    float *orientation,     /* output[4] = [qw, qx, qy, qz] */
    float *angular_velocity /* output[3] = [wx, wy, wz] */
);

/* Set control setpoints */
void firmware_lib_set_altitude(float z);
void firmware_lib_set_attitude(float roll, float pitch, float yaw);

/* Check if estimator is initialized */
int firmware_lib_is_initialized(void);

/* Get initialization progress [0.0, 1.0] */
float firmware_lib_init_progress(void);

#endif /* FIRMWARE_LIB_H */

