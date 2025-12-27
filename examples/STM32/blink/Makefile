# Makefile for STM32F405 bare metal LED blink
# No HAL, no libraries - pure register access

# Target name
TARGET = blink

# Cross compiler
PREFIX = arm-none-eabi-
CC = $(PREFIX)gcc
AS = $(PREFIX)gcc -x assembler-with-cpp
LD = $(PREFIX)gcc
OBJCOPY = $(PREFIX)objcopy
OBJDUMP = $(PREFIX)objdump
SIZE = $(PREFIX)size

# Source files
C_SOURCES = main.c
ASM_SOURCES = startup_stm32f405.s

# Linker script
LDSCRIPT = stm32f405.ld

# Build directory
BUILD_DIR = build

# CPU and FPU options
CPU = -mcpu=cortex-m4
FPU = -mfpu=fpv4-sp-d16
FLOAT_ABI = -mfloat-abi=hard

# Instruction set
MCU = $(CPU) -mthumb $(FPU) $(FLOAT_ABI)

# Compiler flags
CFLAGS = $(MCU)
CFLAGS += -Wall -Wextra -Wpedantic
CFLAGS += -fdata-sections -ffunction-sections  # For --gc-sections
CFLAGS += -O2                                    # Optimization level
CFLAGS += -g3                                    # Debug info
CFLAGS += -std=c11

# Assembler flags
ASFLAGS = $(MCU)
ASFLAGS += -g3

# Linker flags
LDFLAGS = $(MCU)
LDFLAGS += -T$(LDSCRIPT)
LDFLAGS += -Wl,--gc-sections                    # Remove unused sections
LDFLAGS += -Wl,-Map=$(BUILD_DIR)/$(TARGET).map  # Generate map file
LDFLAGS += --specs=nano.specs                   # Use newlib-nano (smaller)
LDFLAGS += --specs=nosys.specs                  # No syscalls

# Object files
OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))

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

# Flash using OpenOCD (requires ST-Link)
flash: all
	openocd -f interface/stlink.cfg -f target/stm32f4x.cfg \
		-c "program $(BUILD_DIR)/$(TARGET).bin 0x08000000 verify reset exit"

# Flash using st-flash (alternative)
flash-stlink: all
	st-flash write $(BUILD_DIR)/$(TARGET).bin 0x8000000

# Debug with OpenOCD (terminal 1)
openocd:
	openocd -f interface/stlink.cfg -f target/stm32f4x.cfg

# Debug with GDB (terminal 2, after starting openocd)
gdb: $(BUILD_DIR)/$(TARGET).elf
	$(PREFIX)gdb -q $< -ex "target extended-remote :3333"

# Erase chip
erase:
	openocd -f interface/stlink.cfg -f target/stm32f4x.cfg \
		-c "init" -c "reset halt" -c "flash erase_sector 0 0 11" -c "exit"

.PHONY: all clean flash flash-stlink openocd gdb disasm erase

