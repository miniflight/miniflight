/*
 * LED Demo - Blue FC LED on PC8
 * 
 * VICTORY! We found the blue LED: PC8
 * This is a clean, documented demo showing full control
 */

#include <stdint.h>

/* ===== HARDWARE DEFINITIONS ===== */

/* RCC - Reset and Clock Control */
#define RCC_AHB1ENR   (*(volatile uint32_t *)0x40023830)
#define RCC_AHB1ENR_GPIOCEN  (1 << 2)  /* Bit 2 = GPIOC clock enable */

/* GPIO Port C */
typedef struct {
    volatile uint32_t MODER;    /* Mode register */
    volatile uint32_t OTYPER;   /* Output type register */
    volatile uint32_t OSPEEDR;  /* Output speed register */
    volatile uint32_t PUPDR;    /* Pull-up/pull-down register */
    volatile uint32_t IDR;      /* Input data register */
    volatile uint32_t ODR;      /* Output data register */
    volatile uint32_t BSRR;     /* Bit set/reset register */
    volatile uint32_t LCKR;     /* Configuration lock register */
    volatile uint32_t AFR[2];   /* Alternate function registers */
} GPIO_TypeDef;

#define GPIOC ((GPIO_TypeDef *)0x40020800)

/* Blue LED */
#define LED_BLUE_PORT  GPIOC
#define LED_BLUE_PIN   8

/* ===== HELPER FUNCTIONS ===== */

void delay_ms(uint32_t ms) {
    /* Rough delay, ~1ms per call at default clock (16MHz HSI) */
    volatile uint32_t count = ms * 4000;
    while(count--) {
        __asm__ volatile ("nop");
    }
}

void led_init(void) {
    /* Enable GPIOC clock */
    RCC_AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    
    /* Configure PC8 as output */
    /* Clear mode bits [17:16] */
    LED_BLUE_PORT->MODER &= ~(3 << (LED_BLUE_PIN * 2));
    /* Set as output (01) */
    LED_BLUE_PORT->MODER |= (1 << (LED_BLUE_PIN * 2));
    
    /* Set output type as push-pull (default, but explicit) */
    LED_BLUE_PORT->OTYPER &= ~(1 << LED_BLUE_PIN);
    
    /* Set output speed to medium */
    LED_BLUE_PORT->OSPEEDR &= ~(3 << (LED_BLUE_PIN * 2));
    LED_BLUE_PORT->OSPEEDR |= (1 << (LED_BLUE_PIN * 2));
    
    /* No pull-up/pull-down */
    LED_BLUE_PORT->PUPDR &= ~(3 << (LED_BLUE_PIN * 2));
}

void led_on(void) {
    /* Use BSRR for atomic bit set */
    LED_BLUE_PORT->BSRR = (1 << LED_BLUE_PIN);
}

void led_off(void) {
    /* Use BSRR for atomic bit reset (upper 16 bits) */
    LED_BLUE_PORT->BSRR = (1 << (LED_BLUE_PIN + 16));
}

void led_toggle(void) {
    /* Toggle using ODR */
    LED_BLUE_PORT->ODR ^= (1 << LED_BLUE_PIN);
}

/* ===== PATTERNS ===== */

void pattern_heartbeat(void) {
    /* Heartbeat: 2 quick blinks, pause */
    led_on();
    delay_ms(100);
    led_off();
    delay_ms(100);
    led_on();
    delay_ms(100);
    led_off();
    delay_ms(700);
}

void pattern_sos(void) {
    /* SOS in morse code */
    /* S: dot dot dot */
    for(int i = 0; i < 3; i++) {
        led_on();
        delay_ms(200);
        led_off();
        delay_ms(200);
    }
    delay_ms(400);
    
    /* O: dash dash dash */
    for(int i = 0; i < 3; i++) {
        led_on();
        delay_ms(600);
        led_off();
        delay_ms(200);
    }
    delay_ms(400);
    
    /* S: dot dot dot */
    for(int i = 0; i < 3; i++) {
        led_on();
        delay_ms(200);
        led_off();
        delay_ms(200);
    }
    delay_ms(1000);
}

void pattern_breathing(void) {
    /* Breathing effect - fade in and out */
    /* (PWM would be better, but this is a simple on/off demo) */
    for(int i = 0; i < 20; i++) {
        led_on();
        delay_ms(i * 2);
        led_off();
        delay_ms(40 - (i * 2));
    }
    for(int i = 20; i > 0; i--) {
        led_on();
        delay_ms(i * 2);
        led_off();
        delay_ms(40 - (i * 2));
    }
}

/* ===== MAIN ===== */

void main(void) {
    /* Initialize the LED */
    led_init();
    
    /* Turn off to start clean */
    led_off();
    delay_ms(500);
    
    /* Victory celebration: 10 rapid blinks */
    for(int i = 0; i < 10; i++) {
        led_toggle();
        delay_ms(100);
    }
    delay_ms(1000);
    
    /* Demo different patterns forever */
    while(1) {
        /* Heartbeat pattern (3 cycles) */
        for(int i = 0; i < 3; i++) {
            pattern_heartbeat();
        }
        delay_ms(1000);
        
        /* SOS pattern (2 cycles) */
        for(int i = 0; i < 2; i++) {
            pattern_sos();
        }
        delay_ms(1000);
        
        /* Breathing pattern (2 cycles) */
        for(int i = 0; i < 2; i++) {
            pattern_breathing();
        }
        delay_ms(1000);
        
        /* Simple blink (5 seconds) */
        for(int i = 0; i < 10; i++) {
            led_on();
            delay_ms(250);
            led_off();
            delay_ms(250);
        }
        delay_ms(2000);
    }
}

