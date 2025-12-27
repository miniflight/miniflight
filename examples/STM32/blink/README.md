# STM32F405 Bare Metal LED Blink

**VICTORY!** Successfully identified and controlled the blue FC LED on PC8.

## 🎯 Quick Start

```bash
# Build
make

# Flash (enter DFU mode: hold BOOT, plug USB, release BOOT)
dfu-util -a 0 -s 0x08000000:leave -D build/blink.bin
```

## 📁 Project Structure

```
blink/
├── main.c                    # LED demo code (PC8 patterns)
├── startup_stm32f405.s       # Boot code (vector table, reset handler)
├── stm32f405.ld             # Linker script (memory layout)
├── Makefile                  # Build configuration
├── .gitignore               # Git ignore rules
├── betaflight_backup.bin    # ⚠️ IMPORTANT: Original firmware backup
├── build/                    # Compiled binaries (gitignored)
├── docs/                     # Documentation
│   ├── README.md            # Detailed usage guide
│   ├── EXPLAINED.md         # Deep technical explanation
│   ├── QUICKSTART.md        # Fast-track guide
│   └── HARDWARE.md          # Board pinout and config
└── tests/                    # Test utilities
    ├── check_toolchain.sh   # Verify ARM GCC installation
    ├── pin_finder.c         # Pin identification code
    ├── led_demo.c           # LED pattern demo
    └── main_backup.c        # Previous version backup
```

## 🔧 Hardware Configuration

- **Microcontroller**: STM32F405VGT6
- **Flash**: 1MB @ 0x08000000
- **RAM**: 192KB @ 0x20000000
- **Blue LED**: PC8 (Port C, Pin 8)

## 📚 Documentation

- **[docs/HARDWARE.md](docs/HARDWARE.md)** - Board pinout, LED mapping, boot modes
- **[docs/EXPLAINED.md](docs/EXPLAINED.md)** - Line-by-line code walkthrough
- **[docs/QUICKSTART.md](docs/QUICKSTART.md)** - Fast setup guide
- **[docs/README.md](docs/README.md)** - Full documentation

## 🛡️ Safety

**IMPORTANT**: `betaflight_backup.bin` contains your original firmware.

To restore:
```bash
# Enter DFU mode
dfu-util -a 0 -s 0x08000000:leave -D betaflight_backup.bin
```

## 🚀 What's Next

This is the foundation for **MiniFlight** - a minimal, hackable flight controller.

See `../ARCHITECTURE.md` for the full roadmap.

## 🎓 What You Learned

- ✅ Bare metal C programming
- ✅ ARM Cortex-M4 architecture
- ✅ Vector tables and startup code
- ✅ Linker scripts and memory layout
- ✅ Register-level GPIO control
- ✅ DFU bootloader protocol
- ✅ Binary firmware flashing

**You can now program ANY microcontroller!** 🏆
