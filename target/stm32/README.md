# MiniFlight C Firmware for STM32F405

A **surgical port** of the Python firmware to bare-metal C for STM32F405 flight controllers.

## 🎯 Purpose

This is an **educational exercise** to:
- Compare Python vs C performance
- Learn first-principles firmware development
- Explore multi-target compilation patterns
- Understand low-level flight controller architecture

## 📁 Structure

```
target/stm32/
├── math.h              # Vector3D, Quaternion (port of common/math.py)
├── types.h             # Data structures (port of common/types.py)
├── estimate.h/c        # Mahony AHRS (port of miniflight/estimate.py)
├── control.h/c         # PID + Stability (port of miniflight/control.py)
├── board.h             # Board HAL interface (port of miniflight/board.py)
├── board_stm32f4.c     # STM32F4 implementation
├── main.c              # Main control loop (port of miniflight/main.py)
└── Makefile            # Build system
```

## 🔄 Comparison with Python

| Component | Python | C |
|-----------|--------|---|
| **Math** | `common/math.py` | `math.h` |
| **Estimator** | `miniflight/estimate.py` | `estimate.h/c` |
| **Controller** | `miniflight/control.py` | `control.h/c` |
| **Board HAL** | `miniflight/board.py` | `board.h` + `board_stm32f4.c` |
| **Main Loop** | `miniflight/main.py` | `main.c` |

**Key Differences:**
- Python uses NumPy arrays → C uses structs + inline functions
- Python uses classes → C uses structs + functions
- Python is dynamic → C is static
- **Same algorithms, different language!**

## 🔧 Building

### Prerequisites

```bash
# Install ARM toolchain
brew install arm-none-eabi-gcc  # macOS
# or
sudo apt install gcc-arm-none-eabi  # Linux

# Install flashing tools (optional)
brew install openocd stlink
```

### Build Firmware

```bash
cd target/stm32
make
```

**Output:**
- `build/miniflight_firmware.elf` - Executable with debug symbols
- `build/miniflight_firmware.hex` - Intel HEX format
- `build/miniflight_firmware.bin` - Raw binary for flashing

### Flash to STM32F405

```bash
# Using OpenOCD
make flash

# Or using st-flash
make flash-stlink
```

## 🧪 Testing

### Current Status

**✅ Implemented:**
- Math primitives (quaternions, vectors)
- Mahony AHRS estimator
- PID controllers
- Stability controller
- Main control loop structure
- Basic board HAL

**🚧 TODO (Stubs):**
- SPI communication with MPU6000 IMU
- PWM generation for motors
- Proper timing calibration
- DShot protocol support
- Blackbox logging

### Validation

To validate correctness vs Python:
1. Run Python firmware in sim
2. Log sensor inputs and control outputs
3. Replay same sensor inputs to C firmware
4. Compare outputs (should be identical within floating-point precision)

## 🎓 Learning Path

### Step 1: Understand the Port ✅
- Read Python code
- Read C code side-by-side
- Verify algorithms match

### Step 2: Hardware Bring-Up (Next)
- Implement IMU reading (SPI)
- Implement motor output (PWM/DShot)
- Test on real hardware

### Step 3: Performance Comparison
- Measure loop timing (Python vs C)
- Profile memory usage
- Compare CPU load

### Step 4: Fingerprinting (Future)
- Auto-detect STM32F4 vs F7 vs H7
- Adapt to different IMUs (MPU6000, ICM20602, BMI270)
- Support multiple board layouts

## 🔬 Key Insights

### Why This Works

**1. Clean Abstraction Layers:**
```
┌─────────────────────────────┐
│   Control Algorithms        │  ← Pure math (portable)
├─────────────────────────────┤
│   Board HAL Interface       │  ← Abstract (portable)
├─────────────────────────────┤
│   Board Implementation      │  ← Hardware-specific
└─────────────────────────────┘
```

**2. Same Data Flow:**
```
Sensors → Estimator → Controller → Actuators
```
Both Python and C follow this exact flow.

**3. Static vs Dynamic:**
- Python allocates at runtime
- C allocates at compile time
- But the **logic is identical**

### Performance Expectations

**Python (sim):**
- ~1-2ms per loop iteration
- Limited by NumPy overhead
- Good enough for 100Hz control

**C (hardware):**
- ~0.1-0.5ms per loop iteration
- Can run at 1-4kHz
- Industry standard for flight controllers

## 📝 Notes

- This is a **minimal viable firmware**
- Real production firmware needs:
  - Proper sensor drivers
  - Calibration routines
  - Safety checks (arm/disarm)
  - Configuration system
  - Telemetry
  - Failsafes
  - More robust timing

- But it demonstrates the **core principles**!

## 🚀 Next Steps

1. **Test build:** `make`
2. **Flash to board:** `make flash`
3. **Observe LED:** Should blink on startup, then respond to "thrust" command
4. **Implement IMU driver:** Read real gyro/accel data
5. **Implement motor output:** Drive real motors
6. **Tune PIDs:** Match Python performance
7. **Benchmark:** Compare Python vs C

## 🙏 Credits

Inspired by:
- **George Hotz (comma.ai)** - Runtime fingerprinting approach
- **Andrej Karpathy** - Educational first-principles teaching
- **Betaflight** - Industry-standard open-source flight controller
- **tinygrad** - Multi-backend compilation patterns

Educational exercise by MiniFlight project.

