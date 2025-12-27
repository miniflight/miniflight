/*
 * Pin Finder - Test each pin individually to find the blue FC LED
 * This will blink ONE pin at a time for 3 seconds each
 */

#include <stdint.h>

/* RCC */
#define RCC_AHB1ENR   (*(volatile uint32_t *)0x40023830)

/* GPIO Ports */
#define GPIOA_MODER   (*(volatile uint32_t *)0x40020000)
#define GPIOA_ODR     (*(volatile uint32_t *)0x40020014)
#define GPIOB_MODER   (*(volatile uint32_t *)0x40020400)
#define GPIOB_ODR     (*(volatile uint32_t *)0x40020414)
#define GPIOC_MODER   (*(volatile uint32_t *)0x40020800)
#define GPIOC_ODR     (*(volatile uint32_t *)0x40020814)

void delay(volatile uint32_t count) {
    while(count--) {
        __asm__ volatile ("nop");
    }
}

void test_pin(volatile uint32_t *MODER, volatile uint32_t *ODR, int pin) {
    /* Configure pin as output */
    *MODER &= ~(3 << (pin * 2));
    *MODER |= (1 << (pin * 2));
    
    /* Blink 3 times quickly to identify */
    for(int i = 0; i < 6; i++) {
        *ODR ^= (1 << pin);
        delay(500000);  /* ~500ms */
    }
    
    /* Turn off */
    *ODR &= ~(1 << pin);
}

void main(void) {
    /* Enable clocks for GPIO A, B, C */
    RCC_AHB1ENR |= (1 << 0) | (1 << 1) | (1 << 2);
    
    /* Test common LED pins */
    while(1) {
        /* Test PA13 (very common for user LED) */
        test_pin(&GPIOA_MODER, &GPIOA_ODR, 13);
        delay(2000000);  /* 2 second pause */
        
        /* Test PC13 (also common) */
        test_pin(&GPIOC_MODER, &GPIOC_ODR, 13);
        delay(2000000);
        
        /* Test PA5 */
        test_pin(&GPIOA_MODER, &GPIOA_ODR, 5);
        delay(2000000);
        
        /* Test PB12 */
        test_pin(&GPIOB_MODER, &GPIOB_ODR, 12);
        delay(2000000);
        
        /* Test PB13 */
        test_pin(&GPIOB_MODER, &GPIOB_ODR, 13);
        delay(2000000);
        
        /* Long pause before repeating */
        delay(5000000);
    }
}

