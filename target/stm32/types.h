/*
 * Type definitions for flight controller
 * Port of common/types.py
 */
#ifndef TYPES_H
#define TYPES_H

#include <stdint.h>
#include "fc_math.h"

/* ===== Sensor Readings ===== */

typedef struct {
    float accel[3];      /* Accelerometer: specific force (m/s^2) */
    float gyro[3];       /* Gyroscope: angular velocity (deg/s) */
    float timestamp;     /* Timestamp (seconds) */
} ImuSample;

typedef struct {
    ImuSample imu;
    float altitude;      /* Altitude (meters) - optional */
    float timestamp;
} SensorReadings;

/* ===== State Estimate ===== */

typedef struct {
    Vector3D position;          /* Position in world frame (m) */
    Vector3D velocity;          /* Velocity in world frame (m/s) */
    Quaternion orientation;     /* Orientation (body -> world) */
    Vector3D angular_velocity;  /* Angular velocity in body frame (deg/s) */
} StateEstimate;

/* ===== Pilot Command ===== */

typedef struct {
    float roll_target;        /* Roll angle target (rad) */
    float pitch_target;       /* Pitch angle target (rad) */
    float yaw_rate;          /* Yaw rate command (rad/s) */
    float z_setpoint_delta;  /* Altitude change (m) */
} PilotCommand;

#endif /* TYPES_H */

