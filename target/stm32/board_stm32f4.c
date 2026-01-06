/*
 * STM32F4 Board Implementation
 * Minimal HAL for STM32F405 flight controller
 */

#include "board.h"
#include <stdint.h>

/* ===== Hardware Definitions ===== */

/* RCC - Reset and Clock Control */
#define RCC_BASE           0x40023800
#define RCC_AHB1ENR        (*(volatile uint32_t *)(RCC_BASE + 0x30))
#define RCC_AHB1ENR_GPIOCEN (1 << 2)

/* GPIO Port C */
typedef struct {
    volatile uint32_t MODER;
    volatile uint32_t OTYPER;
    volatile uint32_t OSPEEDR;
    volatile uint32_t PUPDR;
    volatile uint32_t IDR;
    volatile uint32_t ODR;
    volatile uint32_t BSRR;
    volatile uint32_t LCKR;
    volatile uint32_t AFR[2];
} GPIO_TypeDef;

#define GPIOC ((GPIO_TypeDef *)0x40020800)

/* System timer (SysTick) */
#define SYSTICK_BASE       0xE000E010
#define SYSTICK_CTRL       (*(volatile uint32_t *)(SYSTICK_BASE + 0x00))
#define SYSTICK_LOAD       (*(volatile uint32_t *)(SYSTICK_BASE + 0x04))
#define SYSTICK_VAL        (*(volatile uint32_t *)(SYSTICK_BASE + 0x08))
#define SYSTICK_CALIB      (*(volatile uint32_t *)(SYSTICK_BASE + 0x0C))

/* Clock frequency (HSI = 16MHz by default) */
#define SYSTEM_CLOCK_HZ 16000000

/* ===== Global State ===== */
static volatile uint32_t systick_millis = 0;

/* ===== LED Control (for debugging) ===== */
#define LED_PIN 8  /* PC8 = Blue LED */

static void led_init(void) {
    /* Enable GPIOC clock */
    RCC_AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    
    /* Configure PC8 as output */
    GPIOC->MODER &= ~(3 << (LED_PIN * 2));
    GPIOC->MODER |= (1 << (LED_PIN * 2));
}

static void led_on(void) {
    GPIOC->BSRR = (1 << LED_PIN);
}

static void led_off(void) {
    GPIOC->BSRR = (1 << (LED_PIN + 16));
}

static void led_toggle(void) {
    GPIOC->ODR ^= (1 << LED_PIN);
}

/* ===== Timing ===== */

void SysTick_Handler(void) {
    systick_millis++;
}

static void systick_init(void) {
    /* Configure SysTick for 1ms interrupts */
    SYSTICK_LOAD = (SYSTEM_CLOCK_HZ / 1000) - 1;
    SYSTICK_VAL = 0;
    SYSTICK_CTRL = 0x07;  /* Enable, interrupt, use processor clock */
}

float board_time_seconds(void) {
    return (float)systick_millis / 1000.0f;
}

void board_delay_ms(uint32_t ms) {
    uint32_t start = systick_millis;
    while ((systick_millis - start) < ms) {
        __asm__ volatile ("nop");
    }
}

/* ===== IMU (MPU6000 via SPI - STUB) ===== */

static void imu_init(void) {
    /* TODO: Initialize SPI and MPU6000 */
    /* For now, this is a stub */
}

static void imu_read(ImuSample *sample) {
    /* TODO: Read from MPU6000 via SPI */
    /* For now, return zeros (simulator will override) */
    sample->accel[0] = 0.0f;
    sample->accel[1] = 0.0f;
    sample->accel[2] = GRAVITY;  /* Pointing up at rest */
    
    sample->gyro[0] = 0.0f;
    sample->gyro[1] = 0.0f;
    sample->gyro[2] = 0.0f;
    
    sample->timestamp = board_time_seconds();
}

/* ===== Motors (PWM - STUB) ===== */

static void motors_init(void) {
    /* TODO: Initialize timers for PWM output */
    /* For now, this is a stub */
}

static void motors_write(float *commands, int count) {
    /* TODO: Convert thrust commands to PWM duty cycles */
    /* For now, just blink LED based on first motor command */
    if (count > 0 && commands[0] > 5.0f) {
        led_on();
    } else {
        led_off();
    }
}

/* ===== Board Interface Implementation ===== */

void board_init(void) {
    /* Initialize system tick */
    systick_init();
    
    /* Initialize LED (for debugging) */
    led_init();
    
    /* Initialize IMU */
    imu_init();
    
    /* Initialize motors */
    motors_init();
    
    /* Startup blink sequence */
    for (int i = 0; i < 5; i++) {
        led_toggle();
        board_delay_ms(100);
    }
}

SensorReadings board_read_sensors(void) {
    SensorReadings readings;
    
    /* Read IMU */
    imu_read(&readings.imu);
    
    /* No barometer yet */
    readings.altitude = -1e9f;  /* Invalid */
    
    readings.timestamp = board_time_seconds();
    
    return readings;
}

void board_write_actuators(float *commands, int count) {
    motors_write(commands, count);
}

int board_motor_geometry(Vector3D *positions, int *spins, int max_motors) {
    /* Standard X configuration quadcopter */
    if (max_motors < 4) {
        return 0;
    }
    
    float L = 0.15f;  /* Arm length in meters */
    float diag = L / 1.41421356f;  /* L / sqrt(2) */
    
    positions[0] = vec3_new(diag, diag, 0);      /* Front-right */
    positions[1] = vec3_new(-diag, diag, 0);     /* Front-left */
    positions[2] = vec3_new(-diag, -diag, 0);    /* Back-left */
    positions[3] = vec3_new(diag, -diag, 0);     /* Back-right */
    
    spins[0] = 1;   /* CW */
    spins[1] = -1;  /* CCW */
    spins[2] = 1;   /* CW */
    spins[3] = -1;  /* CCW */
    
    return 4;
}

