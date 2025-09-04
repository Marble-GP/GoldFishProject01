# ESP32 Circular LCD Fish Display System

This project implements a system for displaying moving dots (representing fish) on a Waveshare 480×480 circular LCD module, controlled via serial communication from a PC.

## Project Overview

The system consists of:
1. **ESP32 firmware** - Receives fish position data via serial and displays dots on circular LCD
2. **Python controller** - Sends fish position data with random walk/fish movement simulation
3. **Communication protocol** - Custom binary protocol for efficient data transmission

## Hardware Requirements

- Waveshare ESP32-S3-LCD-2.8C module (480×480 circular LCD)
- USB cable for programming and serial communication
- PC with Python environment

## Development Status

### ✅ Completed Features

1. **Basic ESP32 Program** - Simple dot display with fixed positions
2. **Serial Communication Protocol ID 0x02** - Receives and parses fish position data
3. **Python Fish Sender** - Generates and sends random walk fish movement data
4. **Fish Movement Simulation** - Realistic fish-like movement with boundary reflection

### 🔄 Pending Features

5. **Protocol ID 0x01 Support** - Scroll position commands (low priority)
6. **WASD/Arrow Key Control** - Manual scroll control from PC (low priority)

## File Structure

```
PlotControlTest/
├── src/main.cpp              # ESP32 firmware
├── fish_sender.py            # Python fish movement simulator
├── test_protocol.py          # Protocol testing utility
├── platformio.ini            # PlatformIO configuration
├── lv_conf.h                 # LVGL configuration (if needed)
└── README.md                 # This file
```

## Communication Protocol

### Protocol ID 0x02 - Fish Position Data

**Packet Structure:**
- Byte 0: Protocol ID (0x02)
- Bytes 1-4: Timestamp (uint32_t, little endian)
- Byte 5: Fish count (uint8_t)
- For each fish (13 bytes):
  - Byte 0: Sprite ID (uint8_t)
  - Bytes 1-2: Reserved (2 bytes)
  - Bytes 3-6: X position (int32_t, little endian)
  - Bytes 7-10: Y position (int32_t, little endian)
  - Bytes 11-12: Direction (int16_t, little endian)

**Example packet for 2 fish:**
```
0x02 0x55 0x03 0xF7 0x41 0x02 0x00 0x00 0x00 0xC8 0x00 0x00 0x00 0xC8 0x00 0x00 0x00 0x2D 0x00 0x01 0x00 0x00 0x2C 0x01 0x00 0x00 0xFA 0x00 0x00 0x00 0x87 0x00
```

## Usage Instructions

### 1. Build and Upload ESP32 Firmware

```bash
cd /path/to/PlotControlTest
~/.platformio/penv/bin/platformio run --target upload
```

### 2. Run Fish Movement Simulation

```bash
# Activate Python virtual environment
/home/marble/myPyEnv/py3123/bin/python3 fish_sender.py /dev/ttyUSB0
```

Replace `/dev/ttyUSB0` with your ESP32's serial port.

### 3. Test Protocol Implementation

```bash
/home/marble/myPyEnv/py3123/bin/python3 test_protocol.py
```

## Display Specifications

- **Resolution:** 480×480 pixels
- **Shape:** Circular display area
- **Center:** (240, 240)
- **Radius:** 240 pixels
- **Color Format:** RGB565 for hardware implementation

## Fish Movement Features

- **Random Walk:** Brownian motion with velocity constraints
- **Boundary Reflection:** Fish bounce off circular display edges
- **Multiple Fish:** Support for up to 10 fish simultaneously
- **Sprite Support:** Different colors/sprites for different fish types
- **Real-time Updates:** Configurable update rate (default: 5 Hz)

## Development Tools

- **PlatformIO:** ESP32 development and building
- **Python 3.12+:** Fish simulation and serial communication
- **PySerial:** Serial communication library

## Technical Notes

### ESP32 Implementation
- Currently uses serial output for debugging (no actual LCD driver yet)
- Implements complete protocol parsing
- Validates circular boundary constraints
- Memory-efficient packet handling

### Python Implementation
- Physics-based fish movement simulation
- Configurable fish count and behavior parameters
- Proper little-endian binary protocol implementation
- Real-time position updates with boundary detection

## Next Steps for Full Implementation

1. **Integrate actual LCD driver** - Replace debug output with real LCD drawing
2. **Add LVGL graphics** - Use LVGL library for efficient circular display rendering
3. **Implement sprite graphics** - Replace dots with actual fish sprites
4. **Add scroll support** - Implement protocol ID 0x01 for viewport scrolling
5. **Performance optimization** - Optimize for real-time fish animation

## Dependencies

### ESP32
- Arduino framework
- PlatformIO build system

### Python
- Python 3.12+
- pyserial library

## Installation

```bash
# Install PySerial in virtual environment
/home/marble/myPyEnv/py3123/bin/pip install pyserial

# Build ESP32 firmware
~/.platformio/penv/bin/platformio run
```

This implementation provides a solid foundation for the fish display system with proper communication protocol, movement simulation, and hardware abstraction.