/*
 * MiniFlight Firmware - Main Control Loop
 * Port of miniflight/main.py to C
 */

#include "board.h"
#include "estimate.h"
#include "control.h"
#include "fc_math.h"
#include <stdint.h>

/* === Configuration === */
#define LOOP_RATE_HZ 100.0f
#define DT (1.0f / LOOP_RATE_HZ)

/* === Global State === */
static MahonyEstimator estimator;
static StabilityController controller;

/* === Main Control Loop === */
void firmware_loop(void) {
    /* Read sensors */
    SensorReadings readings = board_read_sensors();
    
    /* Update estimator */
    StateEstimate state = mahony_update(&estimator, &readings, DT);
    
    /* Update controller */
    float control_output[4];  /* [thrust, roll, pitch, yaw] */
    stability_update(&controller, &state, DT, control_output);
    
    /* Write motor commands */
    /* For now, directly pass control output */
    /* In a real system, you'd run this through a mixer */
    board_write_actuators(control_output, 4);
}

/* === Initialization === */
void firmware_init(void) {
    /* Initialize hardware */
    board_init();
    
    /* Initialize estimator (Mahony AHRS) */
    mahony_init(&estimator, 0.5f, 0.001f);
    
    /* Initialize controller */
    stability_init(&controller);
    
    /* Set initial setpoints */
    stability_set_altitude(&controller, 1.0f);
    stability_set_attitude(&controller, 0.0f, 0.0f, 0.0f);
}

/* === Rate Keeper === */
static inline void rate_keeper_wait(float *last_time) {
    float target_time = *last_time + DT;
    float current_time = board_time_seconds();
    
    /* Busy wait if we're early (simple approach) */
    while (current_time < target_time) {
        current_time = board_time_seconds();
    }
    
    *last_time = target_time;
}

/* === Main Entry Point === */
int main(void) {
    /* Initialize firmware */
    firmware_init();
    
    /* Wait for estimator to initialize */
    while (!mahony_is_initialized(&estimator)) {
        SensorReadings readings = board_read_sensors();
        mahony_update(&estimator, &readings, DT);
        board_delay_ms(10);
    }
    
    /* Main control loop */
    float last_time = board_time_seconds();
    while (1) {
        firmware_loop();
        rate_keeper_wait(&last_time);
    }
    
    return 0;
}

