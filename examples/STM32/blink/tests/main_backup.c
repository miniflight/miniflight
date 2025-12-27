/*
 * Bare metal LED blink for STM32F405
 * 
 * This example blinks an LED on GPIO pin PA5
 * Common on many STM32 development boards
 * 
 * No HAL, no libraries - just direct register access
 */

#include <stdint.h>

/* RCC (Reset and Clock Control) Base Address */
#define RCC_BASE            0x40023800

/* RCC Registers */
#define RCC_CR              (*(volatile uint32_t *)(RCC_BASE + 0x00))  /* Clock control */
#define RCC_PLLCFGR         (*(volatile uint32_t *)(RCC_BASE + 0x04))  /* PLL config */
#define RCC_CFGR            (*(volatile uint32_t *)(RCC_BASE + 0x08))  /* Clock config */
#define RCC_AHB1ENR         (*(volatile uint32_t *)(RCC_BASE + 0x30))  /* AHB1 enable */

/* GPIO Base Addresses */
#define GPIOA_BASE          0x40020000
#define GPIOB_BASE          0x40020400
#define GPIOC_BASE          0x40020800
#define GPIOD_BASE          0x40020C00
#define GPIOE_BASE          0x40021000

/* GPIO Port A Registers */
#define GPIOA_MODER         (*(volatile uint32_t *)(GPIOA_BASE + 0x00))
#define GPIOA_ODR           (*(volatile uint32_t *)(GPIOA_BASE + 0x14))

/* GPIO Port B Registers */
#define GPIOB_MODER         (*(volatile uint32_t *)(GPIOB_BASE + 0x00))
#define GPIOB_ODR           (*(volatile uint32_t *)(GPIOB_BASE + 0x14))

/* GPIO Port C Registers */
#define GPIOC_MODER         (*(volatile uint32_t *)(GPIOC_BASE + 0x00))
#define GPIOC_ODR           (*(volatile uint32_t *)(GPIOC_BASE + 0x14))

/* GPIO Port D Registers */
#define GPIOD_MODER         (*(volatile uint32_t *)(GPIOD_BASE + 0x00))
#define GPIOD_ODR           (*(volatile uint32_t *)(GPIOD_BASE + 0x14))

/* GPIO Port E Registers */
#define GPIOE_MODER         (*(volatile uint32_t *)(GPIOE_BASE + 0x00))
#define GPIOE_ODR           (*(volatile uint32_t *)(GPIOE_BASE + 0x14))

/* Pin definitions */
#define LED_PIN             5   /* PA5 */

/* Watchdog registers */
#define IWDG_KR             (*(volatile uint32_t *)0x40003000)
#define IWDG_KEY_RELOAD     0xAAAA
#define IWDG_KEY_START      0xCCCC

/*
 * Simple delay function
 * Not accurate, depends on optimization and clock speed
 * Good enough for LED blinking
 */
void delay(volatile uint32_t count) {
    while(count--) {
        __asm__ volatile ("nop");  /* No operation - prevents optimization */
    }
}

/*
 * Initialize GPIO - TEST VERSION: Configure ALL ports (A,B,C,D,E) as outputs
 */
void gpio_init(void) {
    /* Enable ALL GPIO clocks */
    RCC_AHB1ENR |= (1 << 0);  /* GPIOA */
    RCC_AHB1ENR |= (1 << 1);  /* GPIOB */
    RCC_AHB1ENR |= (1 << 2);  /* GPIOC */
    RCC_AHB1ENR |= (1 << 3);  /* GPIOD */
    RCC_AHB1ENR |= (1 << 4);  /* GPIOE */
    
    /* Configure ALL pins of ALL ports as outputs */
    GPIOA_MODER = 0x55555555;  /* All outputs */
    GPIOB_MODER = 0x55555555;
    GPIOC_MODER = 0x55555555;
    GPIOD_MODER = 0x55555555;
    GPIOE_MODER = 0x55555555;
}

/* Old functions removed - not needed for this test */

/*
 * Main function - called from startup code
 * TEST VERSION: Blink ALL GPIO ports - find that white LED!
 */
void main(void) {
    /* Disable watchdog if it was enabled by bootloader */
    volatile uint32_t *IWDG_KR_ptr = (volatile uint32_t *)0x40003000;
    *IWDG_KR_ptr = IWDG_KEY_RELOAD;
    
    /* Initialize ALL GPIO ports */
    gpio_init();
    
    /* Blink ALL ports at VERY SLOW rate (2 seconds) */
    /* If white LED changes speed = WE FOUND IT! */
    while(1) {
        /* Feed watchdog */
        *IWDG_KR_ptr = IWDG_KEY_RELOAD;
        
        /* Toggle ALL pins on ALL ports */
        GPIOA_ODR ^= 0xFFFF;
        GPIOB_ODR ^= 0xFFFF;
        GPIOC_ODR ^= 0xFFFF;
        GPIOD_ODR ^= 0xFFFF;
        GPIOE_ODR ^= 0xFFFF;
        
        delay(2000000);  /* 2 seconds = SUPER SLOW! */
    }
}

/*
 * If main returns (it shouldn't), the startup code will loop forever
 */

