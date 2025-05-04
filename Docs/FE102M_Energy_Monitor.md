# FE102M Energy Monitor

![License](https://img.shields.io/github/license/Chamil1983/FE102M-Energy-Monitor)
![Stars](https://img.shields.io/github/stars/Chamil1983/FE102M-Energy-Monitor)
![Forks](https://img.shields.io/github/forks/Chamil1983/FE102M-Energy-Monitor)
![Issues](https://img.shields.io/github/issues/Chamil1983/FE102M-Energy-Monitor)

The SM Energy FE102M Energy Monitor is a high-precision energy metering solution built around the ESP32 WROOM 32 microcontroller and ATM90E26 energy monitoring IC. This repository contains documentation, code examples, and resources for working with the FE102M Energy Monitor.

## Table of Contents

- [Features](#features)
- [Hardware Specifications](#hardware-specifications)
- [Pin Configuration](#pin-configuration)
- [Getting Started](#getting-started)
- [Example Code](#example-code)
- [Documentation](#documentation)
- [Supported Current Transformers](#supported-current-transformers)
- [Power Modes](#power-modes)
- [Contributing](#contributing)
- [License](#license)
- [Contact](#contact)

## Features

### Core Components
- **ESP32 WROOM 32**: Wireless and Bluetooth capabilities
- **ATM90E26**: Energy Monitor IC fully compliant with IEC62052-11, IEC62053-21, and IEC62053-23 standards

### Metering Features
- **High Accuracy**: ±0.1% for active energy and ±0.2% for reactive energy over a dynamic range of 6000:1
- **Automatic Temperature Compensation**: Temperature coefficient of 6 ppm/℃ (typical) for on-chip reference voltage
- **Simplified Calibration**: Single-point calibration on each phase for active energy; no calibration needed for reactive/apparent energy
- **Advanced Non-linearity Compensation**: Three current (RMS value)-based segments with programmable thresholds for each phase with independent gain and phase angle compensation
- **Comprehensive Measurement**: Less than ±0.5% fiducial error for Vrms, Irms, mean active/reactive/apparent power, frequency, power factor, and phase angle
- **Interrupt Support**: Integrated with ESP32 for multiple energy and fault monitoring functions
  - Active Energy pulse interrupt
  - Reactive Energy pulse interrupt
  - WarnOut fault interrupt
  - Voltage/Current Zero Crossing interrupt

### Additional Features
- **EEPROM**: 24C256 (I2C)
- **LCD Module**: I2C 20X4 LCD
- **Current Measurement**: Compatible with various CT clamps (see [Supported Current Transformers](#supported-current-transformers))
- **Voltage Input**: AC Low Voltage Input (Voltage < 250V)
- **Programming Support**: Arduino ESP32 Flashing and Programming Compatibility
- **Energy Pulse Outputs**: OPTO Outputs for Meter Active and Reactive Energy pulse
- **Environmental Sensing**: On-board DHT22 Temperature and Humidity Sensor
- **Connectivity**: MODBUS and WiFi
- **User Interface**: 2 × User GPIO (SET and MODE User Programmable Buttons)
- **Status Indicators**: Meter status LEDs and Power LED
- **Programming Options**: Auto/Manual program and serial communication through USB interface
- **Physical Dimensions**: 150mm × 105mm × 35mm

## Hardware Specifications

| Component | Specification |
|-----------|---------------|
| Microcontroller | ESP32 WROOM 32 |
| Energy Monitor IC | ATM90E26 |
| EEPROM | 24C256 (I2C) |
| Display | I2C 20X4 LCD Module |
| Sensors | DHT22 Temperature and Humidity |
| Communication | WiFi, Bluetooth, MODBUS (RS485) |
| Input Voltage | AC Low Voltage (< 250V) |
| Power Supply | 9-12V DC input |
| Dimensions | 150mm × 105mm × 35mm |

## Pin Configuration

### ESP32 PIN Configuration and Functions

| ESP32 Pin No | Pin Name | Pin Function |
|--------------|----------|--------------|
| 4 | (SENSOR_VP) GPIO36 | SET input Switch |
| 5 | (SENSOR_VN) GPIO39 | ATM90E26 IRQ Interrupt |
| 6 | GPIO34 | ATM90E26 WarnOut: Fatal Error Warning Interrupt |
| 7 | GPIO35 | ATM90E26 CF1: Active Energy Pulse Output Interrupt |
| 8 | GPIO32 | ATM90E26 ZX: Voltage Zero-Crossing Output Interrupt |
| 9 | GPIO33 | MODE input Switch |
| 10 | GPIO25 | ATM90E26 CF2: Reactive Energy Pulse Output Interrupt |
| 11 | GPIO26 | ATM90E26 UART RX pin (Should be selected from SW5 Switch to UART Mode) |
| 12 | GPIO27 | ATM90E26 UART TX pin (Should be selected from SW5 Switch to UART Mode) |
| 13 | GPIO14 | WiFi LED |
| 14 | GPIO12 | RUN LED |
| 16 | GPIO13 | MAX485 TXRX Control pin for MODBUS (RS485) |
| 23 | GPIO15 | MODBUS LED |
| 24 | GPIO2 | FAULT LED |
| 26 | GPIO4 | DHT22 (AM2302) Sensor Data pin |
| 27 | GPIO16 | MAX485 RO pin (RXD) |
| 28 | GPIO17 | MAX485 DI pin (TXD) |
| 29 | GPIO5 | ATM90E26 SPI Chip Select Pin |
| 30 | GPIO18 | ATM90E26 SPI SCLK |
| 31 | GPIO19 | ATM90E26 SPI MISO |
| 33 | GPIO21 | I2C SDA |
| 34 | RXD0 | Debug/Programming (USB) RX |
| 35 | TXD0 | Debug/Programming (USB) TX |
| 36 | GPIO22 | I2C SCL |
| 37 | GPIO23 | ATM90E26 SPI MOSI |

## Getting Started

### Hardware Setup

1. Connect a compatible CT clamp to the monitor (see [Supported Current Transformers](#supported-current-transformers)).
2. Connect the device to a 9-12V DC power supply.
3. For voltage measurement, connect the AC voltage input (ensure voltage is < 250V).
4. Optional: Connect to a computer via USB for programming and debugging.

### Software Setup

1. Install the Arduino IDE (if not already installed).
2. Add ESP32 board support to Arduino IDE by following [these instructions](https://github.com/espressif/arduino-esp32/blob/master/docs/arduino-ide/boards_manager.md).
3. Install required libraries:
   - ATM90E26 library
   - DHT sensor library
   - WiFi library for ESP32
   - ModbusMaster library (if using MODBUS)
4. Clone this repository or download the example code.
5. Open the example sketch in Arduino IDE.
6. Set the board to "ESP32 Dev Module" or "WEMOS D1 MINI ESP32" in the Arduino IDE.
7. Set BAUD rate to 921600 for faster flashing.
8. Connect your device via USB and select the correct COM port.
9. Upload the sketch to your device.

### Important Notes

- The Energy Monitor functions require external 12V DC power to operate, even when connected via USB.
- When using voltage output type current sensors (SCT-013-030, SCT-013-050), JP1 and JP2 jumpers should be open.
- For current output type sensors, the jumpers should be shorted.

## Supported Current Transformers

The FE102M Energy Monitor is compatible with the following CT clamps:

| Model | Specification |
|-------|---------------|
| SCT-006 | 20A/25mA |
| SCT-013-030 | 30A/1V (JP1 and JP2 jumper Should be Open) |
| SCT-013-050 | 50A/1V (JP1 and JP2 jumper Should be Open) |
| SCT-010 | 80A/26.6mA |
| SCT-013-000 | 100A/50mA |
| SCT-016 | 120A/40mA |
| SCT-024 | 200A/100mA |
| SCT-024 | 200A/50mA |

**Note**: JP1 and JP2 Links should be open when using voltage output type current sensors. For current output types, these jumpers should be shorted.

## Power Modes

The ATM90E26 has four power modes, defined by the PM1 and PM0 pins:

| PM1:PM0 Value | Power Mode | Description |
|---------------|------------|-------------|
| 11 | Normal Mode | Full functionality |
| 10 | Partial Measurement (M mode) | Reduced power consumption with limited functionality |
| 01 | Detection (D mode) | Low power, only detects presence of signals |
| 00 | Idle (I mode) | Minimal power consumption, most functions disabled |

## Example Code

The repository includes several example sketches to demonstrate the use of the FE102M Energy Monitor:

- [Basic Energy Monitoring](./examples/BasicEnergyMonitoring/BasicEnergyMonitoring.ino) - Simple reading of voltage, current, and power
- [WiFi Data Logging](./examples/WiFiDataLogging/WiFiDataLogging.ino) - Send energy data to a server via WiFi
- [MODBUS Interface](./examples/ModbusInterface/ModbusInterface.ino) - Example for using the MODBUS communication
- [Web Dashboard](./examples/WebDashboard/WebDashboard.ino) - Create a web interface to monitor energy consumption
- [Temperature Compensation](./examples/TemperatureCompensation/TemperatureCompensation.ino) - Using the DHT22 sensor for temperature compensation

## Documentation

- [ATM90E26 Datasheet](./docs/ATM90E26_Datasheet.pdf)
- [ESP32 Pinout Reference](./docs/ESP32_Pinout.pdf)
- [Calibration Guide](./docs/Calibration_Guide.md)
- [Hardware Reference Manual](./docs/Hardware_Reference_Manual.md)
- [Software Library Reference](./docs/Software_Library_Reference.md)

## Repository Structure

```
FE102M-Energy-Monitor/
├── examples/                  # Example Arduino sketches
│   ├── BasicEnergyMonitoring/
│   ├── WiFiDataLogging/
│   ├── ModbusInterface/
│   ├── WebDashboard/
│   └── TemperatureCompensation/
├── libraries/                 # Required libraries
│   ├── ATM90E26_Library/
│   └── FE102M_Library/
├── hardware/                  # Hardware design files
│   ├── schematics/
│   └── pcb_layout/
├── docs/                      # Documentation
│   ├── datasheets/
│   ├── user_manual/
│   └── images/
├── src/                       # Source code for advanced implementations
│   ├── main/
│   └── utilities/
├── LICENSE                    # License file
└── README.md                  # Main documentation file
```

## Contributing

Contributions to the FE102M Energy Monitor project are welcome! Please feel free to submit pull requests, create issues, or suggest improvements.

1. Fork the repository
2. Create your feature branch (`git checkout -b feature/amazing-feature`)
3. Commit your changes (`git commit -m 'Add some amazing feature'`)
4. Push to the branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

## License

This project is distributed under the MIT License. See the `LICENSE` file for more information.

## Contact

Chamil Vithanage - Microcode Embedded Solutions, Australia  
Electronics Engineer | Software Developer | R&D Support | RF Engineering | Product Certification and Testing

For questions or support, please use the GitHub Issues section or contact the repository maintainers directly.

---

**Note**: All test code is OPEN SOURCE and although it is not intended for real-world use, it may be freely used or modified as needed. It is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
