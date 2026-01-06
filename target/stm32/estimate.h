/*
 * State estimation: Mahony AHRS
 * Port of miniflight/estimate.py
 */
#ifndef ESTIMATE_H
#define ESTIMATE_H

#include "types.h"
#include "fc_math.h"
#include <stdbool.h>

/* ===== Mahony Estimator ===== */

#define MAHONY_INIT_SAMPLE_COUNT 20

typedef struct {
    /* Gains */
    float kp;  /* Proportional gain for accelerometer correction */
    float ki;  /* Integral gain for gyro bias correction */
    
    /* State */
    Vector3D bias;           /* Gyro bias estimate (rad/s) */
    Quaternion q;            /* Orientation quaternion (body -> world) */
    bool initialized;
    
    /* Initialization */
    float init_samples[MAHONY_INIT_SAMPLE_COUNT][3];  /* Gravity samples */
    int init_sample_idx;
    
    /* History for velocity estimation */
    float last_alt;
    bool has_last_alt;
    
    /* Last state (for fallback) */
    StateEstimate last_state;
    bool has_last_state;
} MahonyEstimator;

/* Initialize estimator */
void mahony_init(MahonyEstimator *est, float kp, float ki);

/* Reset estimator */
void mahony_reset(MahonyEstimator *est);

/* Update estimator with new sensor readings */
StateEstimate mahony_update(MahonyEstimator *est, SensorReadings *readings, float dt);

/* Check if initialized */
static inline bool mahony_is_initialized(MahonyEstimator *est) {
    return est->initialized;
}

/* Get initialization progress [0.0, 1.0] */
static inline float mahony_init_progress(MahonyEstimator *est) {
    if (est->initialized) return 1.0f;
    if (est->init_sample_idx == 0) return 0.0f;
    return (float)est->init_sample_idx / MAHONY_INIT_SAMPLE_COUNT;
}

#endif /* ESTIMATE_H */

