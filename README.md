# miniflight

This repository contains notes about machines

flight controller firmware. has a mahony filter for orientation and PIDs for control. python version prototypes fast, C version is what you flash.

## flight programs

Miniflight programs are plug-and-play behaviors. A program receives calibrated
IMU and camera observations in one canonical body frame. It returns collective
acceleration and body-rate intent. It cannot address motors or board drivers.

The runtime owns calibration, vehicle adaptation, actuator limits, arming, and
mixing. `hover` is the first planned program. The current real-drone path is
read-only. It must prove units, axes, timestamps, and replay before Miniflight
can deploy a program to a flight controller.

The line counter measures all project Python files. It ignores comments and docstrings.

```bash
python3 sz.py
MAX_LINE_COUNT=400 python3 sz.py
```

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

## live configurator

the localhost configurator reads a connected Betaflight controller over USB.
it shows attitude, controller health, power, motors, and raw IMU signals.

```bash
python3 -m config.serve
# http://127.0.0.1:8002
```

plug in the controller and open the URL. no driver selection is necessary on macOS.

`lock ground` creates a local display reference after the disarmed drone becomes still.
the score reports the quality of that reference. `imu scope` stops the 3D renderer and
shows the gyroscope and accelerometer signals.

the configurator has an explicit read-only MSP allowlist. it cannot send calibration,
settings, motor, arming, reboot, or firmware commands.

remove propellers before general bench work. keep the drone disarmed.


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
