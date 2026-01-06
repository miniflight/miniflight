/*
 * Control module: PID and stability controllers
 * Port of miniflight/control.py
 */
#ifndef CONTROL_H
#define CONTROL_H

#include "types.h"
#include "fc_math.h"

/* ===== PID Controller ===== */

typedef struct {
    /* Gains */
    float kp;
    float ki;
    float kd;
    float integral_limit;
    
    /* State */
    float integral;
    float prev_error;
} PIDController;

/* Initialize PID controller */
void pid_init(PIDController *pid, float kp, float ki, float kd, float integral_limit);

/* Reset PID controller */
void pid_reset(PIDController *pid);

/* Update PID controller */
float pid_update(PIDController *pid, float error, float dt);

/* ===== Stability Controller ===== */

typedef struct {
    /* PID controllers */
    PIDController z_pid;      /* Altitude control */
    PIDController roll_pid;   /* Roll control */
    PIDController pitch_pid;  /* Pitch control */
    PIDController yaw_pid;    /* Yaw control */
    
    /* Setpoints */
    float z_setpoint;
    float roll_setpoint;
    float pitch_setpoint;
    float yaw_setpoint;
} StabilityController;

/* Initialize stability controller */
void stability_init(StabilityController *ctrl);

/* Reset stability controller */
void stability_reset(StabilityController *ctrl);

/* Update stability controller - returns [thrust, roll_cmd, pitch_cmd, yaw_cmd] */
void stability_update(StabilityController *ctrl, StateEstimate *state, float dt, float *output);

/* Set altitude target */
void stability_set_altitude(StabilityController *ctrl, float z);

/* Set attitude target */
void stability_set_attitude(StabilityController *ctrl, float roll, float pitch, float yaw);

#endif /* CONTROL_H */

