/*
 * Board HAL interface
 * Port of miniflight/board.py
 */
#ifndef BOARD_H
#define BOARD_H

#include "types.h"
#include <stdbool.h>

/* ===== Board Interface ===== */

/* Initialize board hardware */
void board_init(void);

/* Read sensors */
SensorReadings board_read_sensors(void);

/* Write actuator commands (4 motor values: 0.0 = off, positive = thrust) */
void board_write_actuators(float *commands, int count);

/* Get current time in seconds */
float board_time_seconds(void);

/* Delay in milliseconds */
void board_delay_ms(uint32_t ms);

/* Motor geometry (optional - for mixer setup) */
/* Returns number of motors, fills positions and spins arrays */
int board_motor_geometry(Vector3D *positions, int *spins, int max_motors);

#endif /* BOARD_H */

