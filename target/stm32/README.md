# Miniflight STM32 firmware

This directory contains the current C firmware experiment for STM32F405.
It is separate from the Python companion stack.

Current C components include math primitives, attitude estimation, PID control,
a board interface, and an STM32F4 board implementation. Hardware drivers and
flight validation are incomplete.

Build the firmware with an ARM embedded toolchain:

```bash
make
```

The next firmware work must add the actual IMU and motor drivers. Do not infer
hardware readiness from a successful build.
