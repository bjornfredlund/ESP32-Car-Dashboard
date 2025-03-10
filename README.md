# ESP32-Car dashboard

## Overview

Project utilizing my custom [ESP32 hardware platform](https://github.com/bjornfredlund/Car-dashboard) as a development platform to build a custom bluetooth enabled car dashboard to vizualise telemetry and stream audio. Integrates multiple communication interfaces and enables features such as **motion tracking, GNSS navigation, LCD-based display modes, and FM transmission**.

## Features

- **LIS2DH12TR Accelerometer** – Motion detection & orientation tracking.
- **UART communication with u-blox NEO-6M-0-001** – GNSS-based positioning.
- **I2C communication with PCF8574** – Drives an LCD display.
- **Finite State Machine (FSM) for display mode toggling**.
- **Bluetooth & FM Transmission (In Development)**.

## Requirements
- [**ESP32 Custom hardware development platform**](https://github.com/bjornfredlund/Car-dashboard)
- **ESP-IDF v5.3** – Required for firmware development and deployment.

## Applications

- **Motion Tracking** – Uses accelerometer data for movement detection.
- **GNSS Navigation** – Interfaces with a u-blox receiver over UART.
- **LCD-Based Interface** – Displays sensor and GNSS data.
- **FM Transmission & Bluetooth** – (Upcoming features).

## Getting Started

#### 1. **Hardware Setup**
- Power the PCB via **USB or an external battery**.
- Connect peripherals (LCD, GNSS module, etc.).

#### 2. **Firmware Development**
- Use **ESP-IDF 5.3v** to develop and flash the firmware.

## Repository Structure

```
/project-root
│-- main/               # Startup code
│   │-- components/     # Software modules
│   │   │-- common/     # Shared utilities and helpers
│   │   │-- fsm/        # Finite State Machine implementation
│   │   │-- screen/     # LCD display handling
│   │   │-- thermometer/ # Temperature sensor integration
│   │   │-- accelerometer/ # Additional components
|   |   |-- led/        # dummy led tasks to highlight functionality
|   |   |-- GNSS/       # gps communication
│-- README.md           # This file
```

#### 3. I2C Device addresses 
    Screen: 0x27
    Temperature Sensor: 0x48
    PCM5102: 0xC0
    Accelerometer: 0x19