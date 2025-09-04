# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Voice Commands
- When processing is stopped or awaiting approval: `agent_voice -f`
- When process is completed: `agent_voice -f`  
- Just before waiting for approval: `agent_voice -a`

## Project Overview

This is a comprehensive fish display system for ESP32 circular LCD with data-driven movement simulation. The project combines real-time hardware display, advanced computer vision analysis, and physics-based fish behavior modeling.

**Main Components:**
1. **ESP32 Fish Display** - Real-time fish visualization on 480×480 circular LCD using LVGL
2. **Data-Driven Fish Simulation** - Python controllers with realistic movement based on video analysis
3. **Fish Video Modeling System** - Computer vision analysis of real fish behavior for parameter extraction

## System Architecture

### ESP32 Hardware Display (`src/main.cpp`)
- **Target Hardware**: Waveshare ESP32-S3-LCD-2.8C (480×480 circular display)
- **Graphics Library**: LVGL 9.3.0 with real LCD drawing via ST7701 driver
- **Protocol Handling**: Binary serial communication (Protocol ID 0x02) for fish position data
- **Fish Rendering**: LVGL objects with HSV color mapping based on sprite ID
- **Memory Management**: PSRAM-optimized with safety bounds for circular display area

### Python Fish Controllers

#### Basic Controller (`fish_sender.py`)
- Simple random walk movement with boundary reflection
- Configurable fish count and update rates
- Brownian motion physics with velocity constraints

#### Realistic Controller (`fish_sender_realistic.py`) 
- **100% Data-Driven**: Parameters extracted from real fish video analysis
- **Time-Stretch Physics**: Coefficient K allows relaxed movement (K=0.2 = 5x slower)
- **Gaussian Statistical Modeling**: Velocity, acceleration, and turning parameters from `fish_body_cache.pkl`
- **Physics-Equivalent Scaling**: Maintains natural characteristics while adjusting speed

### Fish Video Modeling System (`FishVideoModeling/`)
- **Computer Vision Analysis** (`fish_body_analysis.py`): Extracts fish body orientation, curvature, movement patterns
- **Physics-Based Simulation** (`fish_simulation.py`): Bayesian parameter estimation from video data  
- **Training Data**: `fish_body_cache.pkl` contains 296 frames of real fish movement statistics
- **HSV Color Detection**: Tracks red markers on fish for position/orientation analysis

## Build and Development Commands

### ESP32 Development
```bash
# Set toolchain PATH (required)
export PATH="$PATH:$HOME/.platformio/packages/tool-esptoolpy:$HOME/.platformio/packages/toolchain-esp32/bin"

# Build firmware
~/.platformio/penv/bin/pio run

# Upload to ESP32-S3
~/.platformio/penv/bin/pio run --target upload

# Clean build
~/.platformio/penv/bin/pio run --target clean

# Monitor serial output
~/.platformio/penv/bin/pio device monitor
```

### Python Development Environment
```bash
# Use prepared Python virtual environment
/home/marble/myPyEnv/py3123/bin/python3

# Run basic fish simulation
/home/marble/myPyEnv/py3123/bin/python3 fish_sender.py /dev/ttyUSB0

# Run realistic data-driven simulation (recommended)
/home/marble/myPyEnv/py3123/bin/python3 fish_sender_realistic.py /dev/ttyUSB0

# Test movement behavior without hardware
/home/marble/myPyEnv/py3123/bin/python3 test_fish_console.py

# Analyze real fish data parameters
/home/marble/myPyEnv/py3123/bin/python3 analyze_fish_data.py
```

### Fish Video Modeling System
```bash
# Navigate to modeling directory
cd FishVideoModeling

# Run fish body analysis (generates fish_body_cache.pkl)
/home/marble/myPyEnv/py3123/bin/python3 fish_body_analysis.py

# Run movement simulation analysis
/home/marble/myPyEnv/py3123/bin/python3 fish_simulation.py
```

## Communication Protocol

### Serial Configuration
- **Baud Rate**: 115200
- **Format**: 8N1 (8 data bits, no parity, 1 stop bit)  
- **Byte Order**: Little-endian for multi-byte values

### Protocol ID 0x02 - Fish Position Data (IMPLEMENTED)
**Packet Structure (6 + N×13 bytes):**
- Byte 0: Protocol ID (0x02)
- Bytes 1-4: Timestamp (uint32_t, little endian)
- Byte 5: Fish count N (uint8_t, max 10)
- Per fish (13 bytes):
  - Byte 0: Sprite ID (uint8_t)
  - Bytes 1-2: Reserved (padding)  
  - Bytes 3-6: X position (int32_t, little endian)
  - Bytes 7-10: Y position (int32_t, little endian)
  - Bytes 11-12: Direction (int16_t, little endian, degrees)

### Protocol ID 0x01 - Scroll Commands (NOT IMPLEMENTED)
Reserved for future viewport scrolling functionality.

## Hardware Configuration

### ESP32-S3 Board Settings (`platformio.ini`)
```ini
[env:esp32s3dev]
platform = espressif32
board = esp32-s3-devkitc-1  # Critical: NO UF2 bootloader
framework = arduino
lib_deps = 
    lvgl/lvgl@^9.3.0
    mathertel/OneButton@^2.6.1
monitor_speed = 115200
board_build.arduino.memory_type = qio_opi
build_flags = 
    -DBOARD_HAS_PSRAM
    -DCONFIG_SPIRAM_BOOT_INIT=1  
    -DCONFIG_SPIRAM_USE_MALLOC=1
    -mfix-esp32-psram-cache-issue
```

### Display Specifications
- **Resolution**: 480×480 pixels
- **Shape**: Circular visible area (center: 240,240, radius: 240px)
- **Color Format**: RGB565 (16-bit)
- **Graphics**: LVGL 9.3.0 with ST7701 driver integration

## Data-Driven Fish Behavior System

### Statistical Parameter Extraction
The realistic fish controller uses actual fish movement data:

**Real Fish Statistics from Video Analysis (296 frames):**
- Mean velocity: 8.482 pixels/frame
- Acceleration std: 7.690 pixels/frame²  
- Angular velocity: 22.411°/frame
- Movement patterns: Gaussian distribution with autocorrelation-based decay

**Time-Stretch Implementation:**
- **K = 0.2**: 5x slower than original video (relaxed goldfish pace)
- **Physics Scaling Laws**: 
  - Velocity: `v' = v × K`
  - Acceleration: `a' = a × K²` 
  - Angular velocity: `ω' = ω × K`
  - Decay rate: `λ' = 1 - (1-λ) × K`

### Training Data Pipeline
1. **Video Input**: `sample_video.mp4` with red-marked fish
2. **Computer Vision**: HSV color detection → position/orientation extraction  
3. **Statistical Analysis**: Gaussian parameter estimation from movement data
4. **Cache Storage**: `fish_body_cache.pkl` for fast parameter loading
5. **Physics Simulation**: Time-stretched parameters applied to realistic movement

## Key Files and Structure

### Core Implementation
- `src/main.cpp` - ESP32 firmware with LVGL fish display
- `fish_sender_realistic.py` - Data-driven fish simulation (recommended)
- `fish_sender.py` - Basic random walk simulation
- `platformio.ini` - ESP32-S3 build configuration

### Fish Modeling System  
- `FishVideoModeling/fish_body_analysis.py` - Computer vision analysis
- `FishVideoModeling/fish_body_cache.pkl` - Real fish movement statistics
- `FishVideoModeling/sample_video.mp4` - Training video data

### Hardware Drivers (Waveshare Official)
- `src/Display_ST7701.cpp` - LCD driver with RGB interface
- `src/LVGL_Driver.cpp` - LVGL 9.3.0 integration
- `src/TCA9554PWR.cpp` - I/O expander driver
- `include/` - Header files for all drivers

### Development Tools
- `test_fish_console.py` - Behavior comparison without hardware
- `analyze_fish_data.py` - Statistical analysis of fish movement
- `demo/` - Official Waveshare examples and documentation

## Development Notes

### Current Implementation Status
- ✅ **ESP32 Firmware**: Complete with LVGL 9.3.0 real LCD display
- ✅ **Protocol ID 0x02**: Fish position data parsing and display  
- ✅ **Data-Driven Simulation**: 100% statistical fish behavior with time-stretch
- ✅ **Hardware Integration**: Working ST7701 driver with circular boundary validation
- ✅ **Memory Optimization**: PSRAM usage with efficient LVGL object management
- ❌ **Protocol ID 0x01**: Scroll commands (future feature)

### Memory Management
- **Current Usage**: ~26.8% RAM, ~19.2% Flash
- **PSRAM**: Enabled with cache issue fixes
- **LVGL Objects**: Efficient fish rendering with hidden/visible state management

### Critical Configuration Notes
- **Board Selection**: Must use `esp32-s3-devkitc-1` (avoid UF2 bootloader variants)
- **LVGL Version**: Locked to 9.3.0 for API compatibility
- **Time-Stretch Coefficient**: `K=0.2` provides optimal goldfish-like movement speed

## Important Development Paths

### Build Tools
- **PlatformIO**: `~/.platformio/penv/bin/pio`
- **Python Environment**: `/home/marble/myPyEnv/py3123/bin/python3`
- **LVGL Library**: `.pio/libdeps/esp32s3dev/lvgl/`

### Video Analysis Dependencies
- numpy, opencv-python, matplotlib, scipy, scikit-learn
- All analysis scripts use the prepared Python virtual environment

This system provides authentic fish movement based on real video analysis while maintaining efficient real-time display performance on ESP32 hardware.