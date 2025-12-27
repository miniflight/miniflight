/*
 * Minimal startup code for STM32F405
 * Bare metal - no libraries
 */

.syntax unified
.cpu cortex-m4
.fpu fpv4-sp-d16
.thumb

.global Reset_Handler
.global Vector_Table

/* Vector Table - Cortex-M4 requires this at address 0x00000000 (or 0x08000000 in flash) */
.section .isr_vector,"a",%progbits
.type Vector_Table, %object

Vector_Table:
    .word _estack                    /* 0x0000 - Initial Stack Pointer */
    .word Reset_Handler              /* 0x0004 - Reset Handler */
    .word NMI_Handler                /* 0x0008 - NMI Handler */
    .word HardFault_Handler          /* 0x000C - Hard Fault Handler */
    .word MemManage_Handler          /* 0x0010 - MPU Fault Handler */
    .word BusFault_Handler           /* 0x0014 - Bus Fault Handler */
    .word UsageFault_Handler         /* 0x0018 - Usage Fault Handler */
    .word 0                          /* 0x001C - Reserved */
    .word 0                          /* 0x0020 - Reserved */
    .word 0                          /* 0x0024 - Reserved */
    .word 0                          /* 0x0028 - Reserved */
    .word SVC_Handler                /* 0x002C - SVCall Handler */
    .word DebugMon_Handler           /* 0x0030 - Debug Monitor Handler */
    .word 0                          /* 0x0034 - Reserved */
    .word PendSV_Handler             /* 0x0038 - PendSV Handler */
    .word SysTick_Handler            /* 0x003C - SysTick Handler */
    
    /* External Interrupts - STM32F405 has 82 total */
    .word 0                          /* 0x0040 - WWDG */
    .word 0                          /* 0x0044 - PVD */
    /* ... add more as needed ... */
    .word 0
    .word 0
    .word 0
    .word 0
    .word 0
    .word 0
    .word 0
    .word 0
    .word 0
    .word 0
    .word 0
    .word 0
    .word 0
    .word 0
    .word 0
    .word 0
    .word 0
    .word 0
    .word 0
    .word 0

.size Vector_Table, .-Vector_Table

/*
 * Reset Handler - First code executed after reset
 */
.section .text.Reset_Handler
.type Reset_Handler, %function

Reset_Handler:
    /* Copy .data section from flash to SRAM */
    ldr r0, =_sdata          /* Destination start */
    ldr r1, =_edata          /* Destination end */
    ldr r2, =_sidata         /* Source start (in flash) */
    movs r3, #0
    b copy_data_check

copy_data_loop:
    ldr r4, [r2, r3]         /* Load from flash */
    str r4, [r0, r3]         /* Store to SRAM */
    adds r3, r3, #4          /* Increment by 4 bytes */

copy_data_check:
    adds r4, r0, r3          /* Calculate current dest address */
    cmp r4, r1               /* Compare with end */
    bcc copy_data_loop       /* Continue if not done */

    /* Zero out .bss section */
    ldr r0, =_sbss           /* Start of .bss */
    ldr r1, =_ebss           /* End of .bss */
    movs r2, #0
    b zero_bss_check

zero_bss_loop:
    str r2, [r0]             /* Store 0 */
    adds r0, r0, #4          /* Next word */

zero_bss_check:
    cmp r0, r1
    bcc zero_bss_loop

    /* Enable FPU (optional, but we have it) */
    ldr r0, =0xE000ED88      /* CPACR (Coprocessor Access Control Register) */
    ldr r1, [r0]
    orr r1, r1, #(0xF << 20) /* Enable CP10 and CP11 (FPU) */
    str r1, [r0]
    dsb
    isb

    /* Call main() */
    bl main

    /* If main returns, loop forever */
infinite_loop:
    b infinite_loop

.size Reset_Handler, .-Reset_Handler

/*
 * Default exception handlers - Weak so they can be overridden
 */
.weak NMI_Handler
.weak HardFault_Handler
.weak MemManage_Handler
.weak BusFault_Handler
.weak UsageFault_Handler
.weak SVC_Handler
.weak DebugMon_Handler
.weak PendSV_Handler
.weak SysTick_Handler

.type NMI_Handler, %function
.type HardFault_Handler, %function
.type MemManage_Handler, %function
.type BusFault_Handler, %function
.type UsageFault_Handler, %function
.type SVC_Handler, %function
.type DebugMon_Handler, %function
.type PendSV_Handler, %function
.type SysTick_Handler, %function

NMI_Handler:
HardFault_Handler:
MemManage_Handler:
BusFault_Handler:
UsageFault_Handler:
SVC_Handler:
DebugMon_Handler:
PendSV_Handler:
SysTick_Handler:
    b .

.size NMI_Handler, .-NMI_Handler
.size HardFault_Handler, .-HardFault_Handler
.size MemManage_Handler, .-MemManage_Handler
.size BusFault_Handler, .-BusFault_Handler
.size UsageFault_Handler, .-UsageFault_Handler
.size SVC_Handler, .-SVC_Handler
.size DebugMon_Handler, .-DebugMon_Handler
.size PendSV_Handler, .-PendSV_Handler
.size SysTick_Handler, .-SysTick_Handler

