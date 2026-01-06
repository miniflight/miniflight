# Makefile for MiniFlight STM32F405 Firmware
# Bare metal flight controller

# Target name
TARGET = miniflight_firmware

# Cross compiler
PREFIX = arm-none-eabi-
CC = $(PREFIX)gcc
AS = $(PREFIX)gcc -x assembler-with-cpp
LD = $(PREFIX)gcc
OBJCOPY = $(PREFIX)objcopy
OBJDUMP = $(PREFIX)objdump
SIZE = $(PREFIX)size

# Source files
C_SOURCES = \
	main.c \
	estimate.c \
	control.c \
	board_stm32f4.c

# Use startup from blink example
ASM_SOURCES = ../../examples/STM32/blink/startup_stm32f405.s

# Linker script from blink example
LDSCRIPT = ../../examples/STM32/blink/stm32f405.ld

# Build directory
BUILD_DIR = build

# CPU and FPU options
CPU = -mcpu=cortex-m4
FPU = -mfpu=fpv4-sp-d16
FLOAT_ABI = -mfloat-abi=hard

# Instruction set
MCU = $(CPU) -mthumb $(FPU) $(FLOAT_ABI)

# C includes
C_INCLUDES = -I.

# Compiler flags
CFLAGS = $(MCU) $(C_INCLUDES)
CFLAGS += -Wall -Wextra -Wpedantic
CFLAGS += -fdata-sections -ffunction-sections
CFLAGS += -O2                                    # Optimization
CFLAGS += -g3                                    # Debug info
CFLAGS += -std=c11

# Assembler flags
ASFLAGS = $(MCU)
ASFLAGS += -g3

# Linker flags
LDFLAGS = $(MCU)
LDFLAGS += -T$(LDSCRIPT)
LDFLAGS += -Wl,--gc-sections
LDFLAGS += -Wl,-Map=$(BUILD_DIR)/$(TARGET).map
LDFLAGS += --specs=nano.specs
LDFLAGS += --specs=nosys.specs
LDFLAGS += -lm  # Math library

# Object files
OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))

# VPATH for source files
vpath %.c $(sort $(dir $(C_SOURCES)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))

# Default target
all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin
	@echo "=== Build complete ==="
	@$(SIZE) $(BUILD_DIR)/$(TARGET).elf

# Create build directory
$(BUILD_DIR):
	mkdir -p $@

# Compile C files
$(BUILD_DIR)/%.o: %.c Makefile | $(BUILD_DIR)
	@echo "CC $<"
	@$(CC) -c $(CFLAGS) $< -o $@

# Assemble files
$(BUILD_DIR)/%.o: %.s Makefile | $(BUILD_DIR)
	@echo "AS $<"
	@$(AS) -c $(ASFLAGS) $< -o $@

# Link
$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) Makefile
	@echo "LD $@"
	@$(LD) $(OBJECTS) $(LDFLAGS) -o $@

# Generate hex file
$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	@echo "HEX $@"
	@$(OBJCOPY) -O ihex $< $@

# Generate binary file
$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	@echo "BIN $@"
	@$(OBJCOPY) -O binary -S $< $@

# Disassembly
disasm: $(BUILD_DIR)/$(TARGET).elf
	$(OBJDUMP) -d $< > $(BUILD_DIR)/$(TARGET).s

# Clean
clean:
	rm -rf $(BUILD_DIR)

# Flash using OpenOCD
flash: all
	openocd -f interface/stlink.cfg -f target/stm32f4x.cfg \
		-c "program $(BUILD_DIR)/$(TARGET).bin 0x08000000 verify reset exit"

# Flash using st-flash
flash-stlink: all
	st-flash write $(BUILD_DIR)/$(TARGET).bin 0x8000000

# Debug with OpenOCD (terminal 1)
openocd:
	openocd -f interface/stlink.cfg -f target/stm32f4x.cfg

# Debug with GDB (terminal 2)
gdb: $(BUILD_DIR)/$(TARGET).elf
	$(PREFIX)gdb -q $< -ex "target extended-remote :3333"

.PHONY: all clean flash flash-stlink openocd gdb disasm

