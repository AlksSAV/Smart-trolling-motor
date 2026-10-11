# Smart Trolling Motor - Architecture

## Overview
Electronic anchor system for boats. Maintains position using bow-mounted trolling motor with servo steering.

## Hardware
- **MCU**: ESP32-S3-N16R8
- **IMU**: BNO085 (I2C, 20Hz)
- **GNSS**: NEO-M10N (UART1, 5Hz)
- **ESC**: Motor controller (GPIO 15, PWM)
- **Servo**: Steering +/-45 deg (GPIO 16, PWM)

## Software

### Three Operating Modes

**1. MANUAL**
- RC receiver controls motor and steering directly
- ESC and servo at neutral (1500us)

**2. HEADING_HOLD**
- IMU maintains current heading
- Servo compensates for wind/current
- ESC at neutral

**3. ANCHOR_HOLD**
- GNSS maintains GPS position
- Both ESC and servo active
- Uses ArduPilot-inspired loiter logic

### ArduPilot Loiter Logic
- Loiter radius: 2m dead zone
- P-controller for speed based on distance
- Yaw error reduction: slows motor when heading error > 45 deg
- Bearing calculation for target direction

## Control Loop
- IMU reads at 20Hz
- GNSS reads at 5Hz
- Control calculations at 20Hz
- Status display at 1Hz

## Future
- Reverse support (1000-1500us backward)
- Dead zone around neutral (1480-1520us)
- RC receiver integration
- Full PID tuning
- OLED display