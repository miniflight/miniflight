#!/bin/bash
# Check if ARM GCC toolchain is installed

echo "Checking ARM GCC toolchain..."
echo

# Check for compiler
if command -v arm-none-eabi-gcc &> /dev/null; then
    echo "✓ arm-none-eabi-gcc found"
    arm-none-eabi-gcc --version | head -1
else
    echo "✗ arm-none-eabi-gcc NOT found"
    echo "  Install with: brew install --cask gcc-arm-embedded"
    exit 1
fi

echo

# Check for other tools
tools=("arm-none-eabi-ld" "arm-none-eabi-objcopy" "arm-none-eabi-objdump" "arm-none-eabi-size")
for tool in "${tools[@]}"; do
    if command -v "$tool" &> /dev/null; then
        echo "✓ $tool found"
    else
        echo "✗ $tool NOT found"
    fi
done

echo
echo "Optional tools:"

# Check for OpenOCD
if command -v openocd &> /dev/null; then
    echo "✓ openocd found"
    openocd --version 2>&1 | head -1
else
    echo "✗ openocd NOT found (optional, for flashing)"
    echo "  Install with: brew install openocd"
fi

# Check for st-flash
if command -v st-flash &> /dev/null; then
    echo "✓ st-flash found"
    st-flash --version 2>&1 | head -1
else
    echo "✗ st-flash NOT found (optional, alternative to openocd)"
    echo "  Install with: brew install stlink"
fi

echo
echo "Ready to build!"

