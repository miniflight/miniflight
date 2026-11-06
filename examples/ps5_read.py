#!/usr/bin/env python3
"""
Script to print PS5 DualSense controller actions using hidapi.
Requires: pip install hidapi
Pair the controller via Bluetooth before running.
"""
import hid
import sys
import time
from contextlib import contextmanager

VENDOR_ID = 0x054C
PRODUCT_ID = 0x0CE6
REPORT_ID = 0x01

BUTTON_MAP = {
    # USB report mapping: byte index, bit mask
    'Square':   (8,  0x10),
    'Cross':    (8,  0x20),
    'Circle':   (8,  0x40),
    'Triangle': (8,  0x80),
    'L1':       (9,  0x01),
    'R1':       (9,  0x02),
    'L2_btn':   (9,  0x04),
    'R2_btn':   (9,  0x08),
    'Create':   (9,  0x10),
    'Options':  (9,  0x20),
    'L3':       (9,  0x40),
    'R3':       (9,  0x80),
    'PS':       (10, 0x01),
    'Touchpad': (10, 0x02),
    'Mute':     (10, 0x04)
}

# Map hat switch values (0-7) to directional names; 8 means not pressed
HAT_MAP = {
    0: ['Up'],
    1: ['Up','Right'],
    2: ['Right'],
    3: ['Down','Right'],
    4: ['Down'],
    5: ['Down','Left'],
    6: ['Left'],
    7: ['Up','Left'],
    8: []
}

def parse_report(data):
    # data is a list of ints
    lx, ly, rx, ry = data[1], data[2], data[3], data[4]
    # D-pad hat position for USB at byte 8
    hat = data[8] & 0x0F
    buttons = [name for name, (byte, mask) in BUTTON_MAP.items() if data[byte] & mask]
    dpad = HAT_MAP.get(hat, [])
    # Parse touchpad coordinates from extended report (USB)
    # pos fields are little-endian uint32: bits [31:20]=Y, [19:8]=X, [7]=released, [6:0]=seq
    pos0_raw = int.from_bytes(bytes(data[33:37]), 'little')
    x0 = (pos0_raw >> 8) & 0xFFF
    y0 = (pos0_raw >> 20) & 0xFFF
    released0 = (pos0_raw >> 7) & 0x1
    pos1_raw = int.from_bytes(bytes(data[37:41]), 'little')
    x1 = (pos1_raw >> 8) & 0xFFF
    y1 = (pos1_raw >> 20) & 0xFFF
    released1 = (pos1_raw >> 7) & 0x1
    touches = []
    if not released0:
        touches.append((x0, y0))
    if not released1:
        touches.append((x1, y1))
    print(f"LX:{lx:3d} LY:{ly:3d} RX:{rx:3d} RY:{ry:3d} Dpad:{dpad} Buttons:{buttons} Touches:{touches}")


@contextmanager
def open_controller():
    try:
        h = hid.device()
        h.open(VENDOR_ID, PRODUCT_ID)
    except Exception:
        print(f"DualSense not found (VID:PID {VENDOR_ID:04x}:{PRODUCT_ID:04x}). Pair via Bluetooth and retry.")
        sys.exit(1)

    try:
        yield h
    finally:
        h.close()


def main():
    with open_controller() as controller:
        print(f"Opened DualSense Wireless Controller ({VENDOR_ID:04x}:{PRODUCT_ID:04x})")
        print("Reading input reports, press Ctrl+C to exit.")
        try:
            while True:
                data = controller.read(64)
                if not data:
                    time.sleep(0.01)
                    continue
                if data[0] == REPORT_ID:
                    parse_report(data)
        except KeyboardInterrupt:
            print("Exiting...")

if __name__ == "__main__":
    main()
