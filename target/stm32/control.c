/*
 * Control module: PID and stability controllers implementation
 * Port of miniflight/control.py
 */
#include "control.h"
#include <math.h>

/* ===== PID Controller ===== */

void pid_init(PIDController *pid, float kp, float ki, float kd, float integral_limit) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral_limit = fabsf(integral_limit);
    pid_reset(pid);
}

void pid_reset(PIDController *pid) {
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
}

float pid_update(PIDController *pid, float error, float dt) {
    /* Integral term */
    pid->integral += error * dt;
    pid->integral = clamp(pid->integral, -pid->integral_limit, pid->integral_limit);
    
    /* Derivative term */
    float derivative = 0.0f;
    if (dt > 0.0f) {
        derivative = (error - pid->prev_error) / dt;
    }
    
    /* PID output */
    float output = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;
    
    pid->prev_error = error;
    return output;
}

/* ===== Stability Controller ===== */

void stability_init(StabilityController *ctrl) {
    /* Initialize PID controllers with same gains as Python version */
    pid_init(&ctrl->z_pid, 1.2f, 0.1f, 2.5f, 1.0f);
    pid_init(&ctrl->roll_pid, 4.5f, 0.7f, 0.65f, 1.5f);
    pid_init(&ctrl->pitch_pid, 4.5f, 0.7f, 0.65f, 1.5f);
    pid_init(&ctrl->yaw_pid, 1.5f, 0.0f, 0.2f, 1.0f);
    
    /* Initialize setpoints */
    ctrl->z_setpoint = 1.0f;
    ctrl->roll_setpoint = 0.0f;
    ctrl->pitch_setpoint = 0.0f;
    ctrl->yaw_setpoint = 0.0f;
}

void stability_reset(StabilityController *ctrl) {
    pid_reset(&ctrl->z_pid);
    pid_reset(&ctrl->roll_pid);
    pid_reset(&ctrl->pitch_pid);
    pid_reset(&ctrl->yaw_pid);
}

void stability_update(StabilityController *ctrl, StateEstimate *state, float dt, float *output) {
    /* Altitude control (collective thrust) */
    float z_error = ctrl->z_setpoint - state->position.z;
    float thrust = GRAVITY + pid_update(&ctrl->z_pid, z_error, dt);
    thrust = clamp(thrust, 0.0f, 20.0f);
    
    /* Attitude error via quaternion */
    Quaternion q_current = state->orientation;
    Quaternion q_desired = quat_from_euler(ctrl->roll_setpoint, ctrl->pitch_setpoint, ctrl->yaw_setpoint);
    
    /* Error quaternion: q_error = q_desired * q_current^(-1) */
    Quaternion q_error = quat_mul(q_desired, quat_conjugate(q_current));
    quat_normalize(&q_error);
    
    /* Convert error quaternion to rotation vector */
    Vector3D rot_vec = quat_to_rotation_vector(q_error);
    
    /* Transform to body frame */
    Quaternion q_current_conj = quat_conjugate(q_current);
    Vector3D err_body = quat_rotate(q_current_conj, rot_vec);
    
    float roll_error = err_body.x;
    float pitch_error = err_body.y;
    float yaw_error = err_body.z;
    
    /* PID updates */
    float roll_cmd = pid_update(&ctrl->roll_pid, roll_error, dt);
    float pitch_cmd = pid_update(&ctrl->pitch_pid, pitch_error, dt);
    float yaw_cmd = pid_update(&ctrl->yaw_pid, yaw_error, dt);
    
    /* Clamp commands */
    roll_cmd = clamp(roll_cmd, -0.6f, 0.6f);
    pitch_cmd = clamp(pitch_cmd, -0.6f, 0.6f);
    yaw_cmd = clamp(yaw_cmd, -0.4f, 0.4f);
    
    /* Output: [thrust, roll, pitch, yaw] */
    output[0] = thrust;
    output[1] = roll_cmd;
    output[2] = pitch_cmd;
    output[3] = yaw_cmd;
}

void stability_set_altitude(StabilityController *ctrl, float z) {
    ctrl->z_setpoint = z;
}

void stability_set_attitude(StabilityController *ctrl, float roll, float pitch, float yaw) {
    ctrl->roll_setpoint = roll;
    ctrl->pitch_setpoint = pitch;
    ctrl->yaw_setpoint = wrap_angle(yaw);
}

