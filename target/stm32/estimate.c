/*
 * State estimation: Mahony AHRS implementation
 * Port of miniflight/estimate.py
 */
#include "estimate.h"
#include <math.h>
#include <string.h>

#define DEG_TO_RAD (PI / 180.0f)
#define RAD_TO_DEG (180.0f / PI)

/* Initialize estimator */
void mahony_init(MahonyEstimator *est, float kp, float ki) {
    est->kp = kp;
    est->ki = ki;
    mahony_reset(est);
}

/* Reset estimator */
void mahony_reset(MahonyEstimator *est) {
    est->bias = vec3_new(0, 0, 0);
    est->q = quat_identity();
    est->initialized = false;
    est->init_sample_idx = 0;
    est->has_last_alt = false;
    est->has_last_state = false;
    memset(est->init_samples, 0, sizeof(est->init_samples));
}

/* Fallback state when we can't estimate */
static StateEstimate mahony_fallback(MahonyEstimator *est) {
    if (est->has_last_state) {
        return est->last_state;
    }
    
    StateEstimate state;
    state.position = vec3_new(0, 0, 0);
    state.velocity = vec3_new(0, 0, 0);
    state.orientation = quat_identity();
    state.angular_velocity = vec3_new(0, 0, 0);
    
    est->last_state = state;
    est->has_last_state = true;
    est->has_last_alt = false;
    
    return state;
}

/* Update estimator with new sensor readings */
StateEstimate mahony_update(MahonyEstimator *est, SensorReadings *readings, float dt) {
    if (dt <= 0.0f) {
        return mahony_fallback(est);
    }
    
    ImuSample *sample = &readings->imu;
    
    /* Accelerometer: specific force (points up at rest) */
    float ax = sample->accel[0];
    float ay = sample->accel[1];
    float az = sample->accel[2];
    
    /* Gyroscope: convert from deg/s to rad/s */
    float gx = sample->gyro[0] * DEG_TO_RAD;
    float gy = sample->gyro[1] * DEG_TO_RAD;
    float gz = sample->gyro[2] * DEG_TO_RAD;
    
    /* Normalize accelerometer measurement */
    float norm = sqrtf(ax*ax + ay*ay + az*az);
    if (norm < 1e-6f) {
        return mahony_fallback(est);
    }
    
    /* Measured gravity direction (opposite of specific force) */
    float grav_x = -ax / norm;
    float grav_y = -ay / norm;
    float grav_z = -az / norm;
    
    /* === INITIALIZATION === */
    if (!est->initialized) {
        /* Collect samples for stable initialization */
        est->init_samples[est->init_sample_idx][0] = grav_x;
        est->init_samples[est->init_sample_idx][1] = grav_y;
        est->init_samples[est->init_sample_idx][2] = grav_z;
        est->init_sample_idx++;
        
        if (est->init_sample_idx < MAHONY_INIT_SAMPLE_COUNT) {
            return mahony_fallback(est);
        }
        
        /* Average gravity samples */
        float avg_gx = 0, avg_gy = 0, avg_gz = 0;
        for (int i = 0; i < MAHONY_INIT_SAMPLE_COUNT; i++) {
            avg_gx += est->init_samples[i][0];
            avg_gy += est->init_samples[i][1];
            avg_gz += est->init_samples[i][2];
        }
        avg_gx /= MAHONY_INIT_SAMPLE_COUNT;
        avg_gy /= MAHONY_INIT_SAMPLE_COUNT;
        avg_gz /= MAHONY_INIT_SAMPLE_COUNT;
        
        /* Initialize attitude from averaged gravity */
        float roll0 = atan2f(-avg_gy, -avg_gz);
        float pitch0 = atan2f(avg_gx, sqrtf(avg_gy*avg_gy + avg_gz*avg_gz));
        est->q = quat_from_euler(roll0, pitch0, 0.0f);
        quat_normalize(&est->q);
        
        est->initialized = true;
        est->init_sample_idx = 0;
    }
    
    /* === MAHONY FILTER === */
    
    /* Predicted gravity in body frame */
    Vector3D world_gravity = vec3_new(0, 0, -1);
    Quaternion q_conj = quat_conjugate(est->q);
    Vector3D v = quat_rotate(q_conj, world_gravity);
    float vx = v.x, vy = v.y, vz = v.z;
    
    /* Error between measured and predicted gravity */
    float ex, ey, ez;
    
    /* Skip accel correction if magnitude deviates too far from 1g */
    if (fabsf(norm - 1.0f) > 0.05f) {
        ex = ey = ez = 0.0f;
    } else {
        /* Cross product: error = measured_gravity × predicted_gravity */
        ex = grav_y * vz - grav_z * vy;
        ey = grav_z * vx - grav_x * vz;
        ez = grav_x * vy - grav_y * vx;
    }
    
    /* For IMU-only (no magnetometer), do not correct yaw */
    ez = 0.0f;
    
    /* Integral feedback (gyro bias correction) */
    /* Only adapt bias when nearly still */
    float ang_mag = sqrtf(gx*gx + gy*gy + gz*gz);
    if (est->ki > 0.0f && fabsf(norm - 1.0f) <= 0.02f && ang_mag < (20.0f * DEG_TO_RAD)) {
        /* Integrate bias for roll/pitch only */
        est->bias.x += ex * est->ki * dt;
        est->bias.y += ey * est->ki * dt;
        /* No yaw correction without magnetometer */
    }
    
    /* Corrected gyro */
    float gx_c = gx + est->kp * ex + est->bias.x;
    float gy_c = gy + est->kp * ey + est->bias.y;
    float gz_c = gz + est->bias.z;  /* No accel-based yaw correction */
    
    /* Integrate quaternion: q̇ = 0.5 * q ⊗ ω */
    Quaternion omega = quat_new(0.0f, gx_c, gy_c, gz_c);
    Quaternion q_dot = quat_mul(est->q, omega);
    q_dot = quat_scale(q_dot, 0.5f);
    
    est->q = quat_add(est->q, quat_scale(q_dot, dt));
    quat_normalize(&est->q);
    
    /* === STATE ESTIMATE === */
    
    StateEstimate state;
    
    /* Position / velocity placeholders */
    if (est->has_last_state) {
        state.position = est->last_state.position;
        state.velocity = est->last_state.velocity;
    } else {
        state.position = vec3_new(0, 0, 0);
        state.velocity = vec3_new(0, 0, 0);
    }
    
    /* Use altitude sensor if available */
    if (readings->altitude > -1e6f) {  /* Check for valid altitude */
        float z = readings->altitude;
        float vz = 0.0f;
        
        if (est->has_last_alt && dt > 0.0f) {
            vz = (z - est->last_alt) / dt;
        }
        
        est->last_alt = z;
        est->has_last_alt = true;
        
        state.position = vec3_new(0, 0, z);
        state.velocity = vec3_new(0, 0, vz);
    }
    
    state.orientation = est->q;
    state.angular_velocity = vec3_new(sample->gyro[0], sample->gyro[1], sample->gyro[2]);
    
    est->last_state = state;
    est->has_last_state = true;
    
    return state;
}

