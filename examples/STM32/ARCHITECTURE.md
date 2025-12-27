# MiniFlight: Minimal Hackable Flight Stack Architecture

## Vision

Build a minimal, hackable flight controller firmware inspired by:
- **Betaflight**: Performance and real-time scheduling
- **Ardupilot**: Modularity and abstraction
- **iNav**: Navigation and state machines
- **Comma.ai Openpilot**: Fingerprinting, logging, and error handling

---

## Part 1: Understanding Existing Flight Controller Firmware

### 1.1 Betaflight Architecture

**Core Loop Structure:**
```
Main Loop (runs at ~8kHz on STM32F4):
├── Gyro Read (1kHz-8kHz)      ← Read IMU sensor
├── PID Controller (1kHz-8kHz)  ← Calculate motor adjustments
├── Motor Mixer (1kHz-8kHz)     ← Convert PID output to motor values
├── Motor Output (1kHz-8kHz)    ← Write PWM to ESCs
└── Scheduler                    ← Run slower tasks (100Hz, 50Hz, 10Hz, 1Hz)
```

**Key Components:**
- **Scheduler**: Time-sliced task execution (gyro at 8kHz, GPS at 10Hz, OSD at 12Hz)
- **Blackbox Logging**: High-speed data logging for analysis
- **CLI**: Serial command interface for configuration
- **Targets**: Board-specific configurations (pinout, sensors, features)

**File Structure:**
```
betaflight/src/
├── main/
│   ├── fc/          # Flight control (PID, rates, modes)
│   ├── sensors/     # IMU, baro, mag, GPS drivers
│   ├── drivers/     # Low-level hardware (SPI, I2C, UART, DMA)
│   ├── scheduler/   # Task scheduling
│   ├── rx/          # Receiver (SBUS, CRSF, etc.)
│   └── telemetry/   # SmartPort, CRSF telemetry
└── target/          # Board-specific configs
```

**Betaflight Insights:**
- ✅ Very tight main loop (125µs at 8kHz)
- ✅ Excellent performance on STM32F4/F7
- ✅ Scheduler allows flexible task priorities
- ❌ Monolithic codebase (hard to extract pieces)
- ❌ Heavy use of preprocessor macros

---

### 1.2 Ardupilot Architecture

**Layered Design:**
```
Application Layer
├── ArduCopter/ArduPlane/ArduRover  # Vehicle-specific logic
│   ├── Mode_*.cpp                   # Flight modes (Stabilize, Loiter, Auto)
│   ├── GCS_Mavlink.cpp             # Ground station communication
│   └── vehicle.cpp                  # Main vehicle loop
↓
HAL (Hardware Abstraction Layer)
├── AP_HAL/                         # Interface definitions
├── AP_HAL_ChibiOS/                 # STM32 implementation (uses ChibiOS RTOS)
└── AP_HAL_Linux/                   # Linux implementation (for sim)
↓
Drivers
└── AP_InertialSensor/              # IMU drivers
    AP_Baro/                        # Barometer
    AP_GPS/                         # GPS
    AP_Compass/                     # Magnetometer
```

**Key Libraries:**
- **AP_Math**: Vector, matrix, quaternion math
- **AP_NavEKF**: Extended Kalman Filter for state estimation
- **AP_Motors**: Motor mixing and output
- **AP_AHRS**: Attitude/Heading Reference System

**Ardupilot Insights:**
- ✅ Excellent abstraction (runs on STM32, Linux, SITL simulator)
- ✅ Modular library system
- ✅ Mature navigation (EKF, missions, waypoints)
- ✅ Uses ChibiOS RTOS for threading
- ❌ Large codebase (~800K lines)
- ❌ Complex build system

---

### 1.3 iNav Architecture

**Similar to Betaflight but with:**
- **Navigation Layer**: GPS, waypoints, return-to-home
- **State Machines**: Clear mode transitions
- **INAV Estimator**: Position/velocity estimation from GPS + baro + IMU

**iNav Insights:**
- ✅ Good balance of performance and features
- ✅ Better navigation than Betaflight
- ✅ Cleaner state management
- ❌ Still monolithic like Betaflight

---

### 1.4 Comma.ai Openpilot Architecture

**Modular Process-Based Design:**
```
cereal (messaging)
├── boardd         # Hardware interface (CAN, IMU, GPS)
├── sensord        # Sensor fusion
├── modeld         # Neural network model runner
├── plannerd       # Path planning
├── controlsd      # Control outputs
├── loggerd        # High-speed logging (rlog format)
└── ui             # User interface
```

**Key Concepts:**
1. **Processes communicate via ZMQ** (zero-copy message queues)
2. **Cereal**: Cap'n Proto schema for typed messages
3. **Fingerprinting**: Auto-detect car model from CAN messages
4. **Rlog**: Structured logging with replay capability
5. **Panda**: Separate safety board (independent watchdog)

**Openpilot Insights for Flight Controller:**
- ✅ **Modularity**: Each process can be developed/tested independently
- ✅ **Logging**: Every message logged, perfect replay
- ✅ **Fingerprinting**: Auto-detect board/sensors
- ✅ **Safety**: Separate watchdog process
- ✅ **Testability**: Can run in simulator easily
- ❌ Requires more RAM/CPU (not critical for STM32F4)

---

## Part 2: MiniFlight Architecture Proposal

### 2.1 Core Philosophy

**Principles:**
1. **Bare Metal First**: No RTOS initially (add later if needed)
2. **Modular Design**: Each component is a separate module
3. **Message Passing**: Components communicate via queues/buffers
4. **Explicit State**: Clear state machines, no hidden state
5. **Loggable Everything**: All events/data can be logged
6. **Board Agnostic**: HAL separates hardware from logic
7. **Test First**: Simulator and unit tests from day 1

---

### 2.2 System Architecture

```
┌─────────────────────────────────────────────────────┐
│                  Main Scheduler                      │
│  (Fixed-rate loop: 1kHz, 500Hz, 100Hz, 10Hz, 1Hz)  │
└─────────────────────────────────────────────────────┘
           ↓            ↓            ↓
    ┌──────────┐  ┌──────────┐  ┌──────────┐
    │  Sensor  │  │  State   │  │ Control  │
    │  Read    │→ │ Estimate │→ │  Loop    │→ Motors
    │  (1kHz)  │  │ (500Hz)  │  │ (1kHz)   │
    └──────────┘  └──────────┘  └──────────┘
                       ↓
                  ┌──────────┐
                  │  Logger  │
                  │ (async)  │
                  └──────────┘
```

**Module Breakdown:**

1. **HAL (Hardware Abstraction Layer)**
   - GPIO, SPI, I2C, UART, Timer, DMA drivers
   - Board-specific pin definitions
   - Sensor interfaces (gyro, accel, baro, mag)

2. **Sensors Module**
   - IMU driver (MPU6000/BMI270/ICM426xx)
   - Barometer (BMP280/MS5611)
   - Magnetometer (HMC5883/QMC5883)
   - GPS (UBLOX/NMEA parser)

3. **State Estimator**
   - Sensor fusion (complementary/Kalman filter)
   - Attitude estimation (quaternion-based)
   - Position/velocity estimation

4. **Control Module**
   - PID controllers (roll, pitch, yaw, altitude)
   - Rate controllers
   - Angle controllers
   - Flight modes (Rate, Angle, Horizon, GPS Hold)

5. **Motor Mixer**
   - Quadcopter X/H configuration
   - Tricopter, Hex, Octo support
   - PWM/DShot/OneShot output

6. **Receiver (RX) Module**
   - SBUS, CRSF, PPM parsers
   - Failsafe logic
   - Channel mapping

7. **Telemetry**
   - MAVLink protocol
   - CRSF/SmartPort telemetry
   - Ground station communication

8. **Logger**
   - High-speed blackbox logging
   - Structured log format (inspired by rlog)
   - SD card / flash storage

9. **CLI / Configuration**
   - Serial command interface
   - Parameter storage (flash)
   - Calibration routines

10. **Safety Monitor**
    - Watchdog timer
    - Voltage monitoring
    - Failsafe triggers
    - Error reporting

---

### 2.3 File Structure

```
miniflight/
├── src/
│   ├── core/
│   │   ├── scheduler.c       # Main loop & task scheduling
│   │   ├── state.c           # Global state machine
│   │   └── config.c          # Configuration management
│   ├── hal/
│   │   ├── stm32f4/
│   │   │   ├── gpio.c
│   │   │   ├── spi.c
│   │   │   ├── timer.c
│   │   │   └── dma.c
│   │   └── interface.h        # HAL interface definitions
│   ├── sensors/
│   │   ├── imu.c             # IMU abstraction
│   │   ├── gyro_mpu6000.c    # Specific sensor drivers
│   │   ├── baro_bmp280.c
│   │   └── gps_ublox.c
│   ├── estimator/
│   │   ├── attitude.c        # Attitude estimation
│   │   ├── position.c        # Position estimation
│   │   └── kalman.c          # EKF implementation
│   ├── control/
│   │   ├── pid.c             # PID controller
│   │   ├── rate_controller.c
│   │   ├── angle_controller.c
│   │   └── modes.c           # Flight modes
│   ├── mixer/
│   │   └── motor_mixer.c     # Quad/hex/tri mixers
│   ├── rx/
│   │   ├── sbus.c
│   │   ├── crsf.c
│   │   └── failsafe.c
│   ├── telemetry/
│   │   ├── mavlink.c
│   │   └── crsf_telem.c
│   ├── logger/
│   │   └── blackbox.c
│   ├── cli/
│   │   └── commands.c
│   └── safety/
│       └── watchdog.c
├── boards/
│   ├── generic_f405/         # Your board
│   │   ├── board.h           # Pin definitions
│   │   ├── config.h          # Default config
│   │   └── startup.s         # Startup code
│   └── betaflight_f4/
├── sim/
│   └── simulator.py          # Physics simulator
├── test/
│   ├── test_pid.c
│   ├── test_math.c
│   └── test_estimator.c
└── tools/
    ├── flasher/
    │   └── dfu_flash.py      # Custom flasher
    └── configurator/
        └── web_gui.html      # Browser-based config tool
```

---

### 2.4 Data Flow

**Message Structure (inspired by cereal/rlog):**
```c
typedef struct {
    uint64_t timestamp_us;     // Microsecond timestamp
    uint8_t msg_type;          // Message type ID
    uint16_t length;           // Payload length
    uint8_t data[];            // Payload
} Message;

// Message types
#define MSG_IMU_DATA         1
#define MSG_STATE_ESTIMATE   2
#define MSG_CONTROL_OUTPUT   3
#define MSG_RX_INPUT         4
#define MSG_ERROR            5
```

**Circular buffers for inter-module communication:**
```c
typedef struct {
    Message buffer[BUFFER_SIZE];
    volatile uint32_t read_idx;
    volatile uint32_t write_idx;
} MessageQueue;
```

---

### 2.5 Timing & Scheduling

**Main Loop (1kHz base rate):**
```c
void main_loop(void) {
    uint32_t loop_counter = 0;
    
    while(1) {
        uint64_t loop_start = micros();
        
        // 1kHz tasks (every loop)
        read_gyro();
        update_attitude();
        run_rate_controller();
        mix_and_output_motors();
        
        // 500Hz tasks (every 2 loops)
        if(loop_counter % 2 == 0) {
            read_accelerometer();
        }
        
        // 100Hz tasks (every 10 loops)
        if(loop_counter % 10 == 0) {
            read_baro();
            process_rx_input();
            update_telemetry();
        }
        
        // 10Hz tasks (every 100 loops)
        if(loop_counter % 100 == 0) {
            read_gps();
            update_position_estimate();
            check_failsafe();
        }
        
        // 1Hz tasks (every 1000 loops)
        if(loop_counter % 1000 == 0) {
            update_leds();
            check_battery();
        }
        
        loop_counter++;
        
        // Wait for next 1ms tick
        wait_until(loop_start + 1000);  // 1000us = 1ms
    }
}
```

---

### 2.6 Board Fingerprinting (Inspired by Openpilot)

**Auto-detect sensors on boot:**
```c
typedef struct {
    const char *name;
    uint8_t spi_bus;
    uint8_t cs_pin;
    uint8_t who_am_i_reg;
    uint8_t expected_value;
} SensorFingerprint;

const SensorFingerprint gyro_fingerprints[] = {
    {"MPU6000",  SPI1, PA4, 0x75, 0x68},
    {"ICM20689", SPI1, PA4, 0x75, 0x98},
    {"BMI270",   SPI1, PA4, 0x00, 0x24},
};

void detect_hardware(void) {
    // Try each known sensor
    for(int i = 0; i < NUM_GYROS; i++) {
        uint8_t who_am_i = spi_read_register(
            gyro_fingerprints[i].spi_bus,
            gyro_fingerprints[i].cs_pin,
            gyro_fingerprints[i].who_am_i_reg
        );
        
        if(who_am_i == gyro_fingerprints[i].expected_value) {
            board.gyro = &gyro_fingerprints[i];
            break;
        }
    }
}
```

---

### 2.7 Error Logging & Recovery

**Error codes with context:**
```c
typedef enum {
    ERR_NONE = 0,
    ERR_IMU_INIT_FAILED,
    ERR_IMU_DATA_TIMEOUT,
    ERR_RX_SIGNAL_LOST,
    ERR_LOW_VOLTAGE,
    ERR_HIGH_TEMPERATURE,
    ERR_CRASH_DETECTED,
} ErrorCode;

void log_error(ErrorCode code, uint32_t context) {
    Error err = {
        .timestamp = micros(),
        .code = code,
        .context = context,
        .stack_trace = get_stack_trace(),
    };
    
    write_to_blackbox(&err);
    
    // Take recovery action
    handle_error(code);
}
```

---

## Part 3: Development Roadmap

### Phase 1: Foundation (Weeks 1-2)
- ✅ LED blink (done!)
- ⬜ HAL: GPIO, SPI, UART, Timer
- ⬜ Main loop with timing
- ⬜ Simple scheduler

### Phase 2: Sensors (Weeks 3-4)
- ⬜ IMU driver (MPU6000 or BMI270)
- ⬜ Read gyro at 1kHz
- ⬜ Calibration routine
- ⬜ Blackbox logging to SD card

### Phase 3: Attitude (Weeks 5-6)
- ⬜ Complementary filter
- ⬜ Quaternion math
- ⬜ Attitude estimation

### Phase 4: Control (Weeks 7-8)
- ⬜ PID controllers
- ⬜ Rate mode
- ⬜ Angle mode
- ⬜ Motor mixing & output

### Phase 5: Receiver (Weeks 9-10)
- ⬜ SBUS/CRSF parser
- ⬜ Channel mapping
- ⬜ Failsafe logic

### Phase 6: Flight Test (Week 11-12)
- ⬜ Bench test (motors spinning)
- ⬜ First hover
- ⬜ Tuning

### Phase 7: Advanced Features (Months 4+)
- ⬜ GPS & position hold
- ⬜ Barometer & altitude hold
- ⬜ Magnetometer & heading hold
- ⬜ Return-to-home
- ⬜ Autonomous waypoints

---

## Part 4: Why This Approach?

**Compared to using existing firmware:**

| Aspect | Betaflight | Ardupilot | MiniFlight |
|--------|-----------|-----------|------------|
| Code size | ~200KB | ~500KB | ~20KB (start) |
| Complexity | High | Very High | Minimal |
| Hackability | Medium | Medium | **Extreme** |
| Learning | Hard | Hard | **Step-by-step** |
| Custom features | Difficult | Medium | **Easy** |
| Understanding | Opaque | Opaque | **Complete** |

**You will understand EVERYTHING:**
- Every line of code YOU wrote
- Every register access
- Every timing decision
- Every control algorithm

**This is your flight controller. You own it.** 🚀

---

## Next Steps

1. **Find the exact LED pin** (test code running now)
2. **Clean up the blink code** (make it modular)
3. **Set up HAL structure**
4. **Add timing & scheduling**
5. **Begin sensor integration**

Welcome to the journey of building MiniFlight! 🎉

