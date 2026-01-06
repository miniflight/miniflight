# miniflight

flight controller firmware. has a mahony filter for orientation and PIDs for control. python version prototypes fast, C version is what you flash.

the sim does rigid body physics with RK4 integration. renders in browser with three.js over websockets. quad actually flies around, you can crash it.

## run

python:
```bash
TARGET=sitl_python python -m miniflight.main
```

C (build once, then run):
```bash
cd target/stm32 && make -f Makefile.lib && cd ../..
TARGET=sitl_c python -m miniflight.main
```

WASD throttle/yaw, arrows pitch/roll.

## configurator

web UI for tuning. serves over websockets, plots IMU data in real time. 

```bash
cd config && python serve.py
# localhost:8080
```

useful for seeing what the filter is doing. can tune PIDs (soon, not implemented yet) without restarting. 

## how it works

100Hz loop: read sensors → mahony filter → PID controllers → motors

mahony takes gyro+accel, integrates to quaternion. no magnetometer so yaw drifts. needs 20 samples on startup to average the gravity vector.

PIDs: altitude uses collective thrust (feedforward gravity + error correction), attitude uses quaternion error converted to body frame. integral terms are clamped.

the C version is the same code. compiles to a .dylib that python loads. when you run sitl_c, python calls C functions for control but keeps python for physics. proves C works before you flash to hardware.

## stm32

bare metal, ~1MB binary. register access for GPIO. SysTick for timing. no IMU or motor drivers yet - just blinks an LED. proved the toolchain works (arm-none-eabi-gcc, openocd).

## what's missing

- IMU driver (MPU6000 over SPI)
- motor output (PWM or DShot)  
- proper calibration routines
- safety checks (arm/disarm logic, state machine)
- logging
- hitl testing

code is structured like it should work but hasn't touched real hardware beyond LED blink.
