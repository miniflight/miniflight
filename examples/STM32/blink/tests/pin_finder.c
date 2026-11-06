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

typedef struct {
    volatile uint32_t *mode;
    volatile uint32_t *output;
    int pin;
} PinCandidate;

static const PinCandidate candidates[] = {
    {&GPIOA_MODER, &GPIOA_ODR, 13},
    {&GPIOC_MODER, &GPIOC_ODR, 13},
    {&GPIOA_MODER, &GPIOA_ODR, 5},
    {&GPIOB_MODER, &GPIOB_ODR, 12},
    {&GPIOB_MODER, &GPIOB_ODR, 13},
};

void main(void) {
    /* Enable clocks for GPIO A, B, C */
    RCC_AHB1ENR |= (1 << 0) | (1 << 1) | (1 << 2);
    
    for (;;) {
        for (unsigned i = 0; i < sizeof(candidates) / sizeof(candidates[0]); i++) {
            const PinCandidate *candidate = &candidates[i];
            test_pin(candidate->mode, candidate->output, candidate->pin);
            delay(2000000);
        }
        delay(5000000);
    }
}

