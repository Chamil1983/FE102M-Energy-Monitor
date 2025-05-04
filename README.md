## SM Energy ESP32 based ATM90E26 Energy Monitor SDK Model FE102M
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Arduino Compatible](https://img.shields.io/badge/Arduino-Compatible-blue.svg)](https://www.arduino.cc/)
[![ESP32](https://img.shields.io/badge/ESP32-WROOM32-red.svg)](https://www.espressif.com/)
[![ESPHome Compatible](https://img.shields.io/badge/ESPHome-Compatible-green.svg)](https://esphome.io/)
[![Home Assistant Compatible](https://img.shields.io/badge/Home_Assistant-Compatible-green.svg)](https://www.home-assistant.io/)
[![Release Version](https://img.shields.io/badge/Release-v1.1.0-green.svg)](https://github.com/mesa-automation/cortex-link-a8r-m/releases)

GTEM Overview
![Display-Type-B](https://github.com/Chamil1983/FE102M-Energy-Monitor/blob/main/Pictures/IMG_4226.jpg?raw=true)

![Display-Type-B](https://github.com/Chamil1983/FE102M-Energy-Monitor/blob/main/Pictures/IMG_4206.jpg?raw=true)
![Display-Type-B](https://github.com/Chamil1983/FE102M-Energy-Monitor/blob/main/Pictures/IMG_4207.jpg?raw=true)

**FE102M Power Energy Monitor Board SDK Overview**

**FE102M Main Board**
![Display-Type-B](Main_Board.jpg?raw=true)

**FE102M Front LCD Board**
![Display-Type-B](Front_Panel.jpg?raw=true)

The SM Energy FE102M is a high-precision energy monitoring solution built around the ESP32 microcontroller and ATM90E26 energy monitoring IC. This repository contains the Arduino library, documentation, and example code for working with the FE102M Energy Monitor.

## Features

- **High Accuracy Energy Measurement**: ±0.1% for active energy and ±0.2% for reactive energy over a dynamic range of 6000:1
- **Comprehensive Metering**: Voltage, current, power (active, reactive, apparent), power factor, frequency, and energy
- **Dual Current Channels**: L-line and N-line current measurement for imbalance detection
- **Environmental Monitoring**: Integrated DHT22 temperature and humidity sensor
- **Multiple Communication Interfaces**: WiFi, MODBUS RTU (RS485), and USB
- **Flexible Connectivity**: Simple integration with home automation systems, energy management platforms, and custom systems
- **User Interface**: 20x4 LCD display and LED status indicators
- **Compatible with Various Current Sensors**: Works with a wide range of current transformer clamps
- **Open Source**: All code is open source and freely modifiable

## Technical Specifications

| Parameter | Specification |
|-----------|---------------|
| Microcontroller | ESP32 WROOM 32 |
| Energy Monitor IC | ATM90E26 |
| Voltage Input | AC Low Voltage (<250V) |
| Current Input | Various CT clamps (20A to 200A) |
| Accuracy | ±0.1% for active energy, ±0.2% for reactive energy |
| Display | I2C 20x4 LCD Module |
| Environmental Sensor | DHT22 Temperature and Humidity |
| Memory | 24C256 EEPROM (I2C) |
| Communication | WiFi, Bluetooth, MODBUS RTU (RS485), USB |
| Dimensions | 150mm × 105mm × 35mm |
| Power Supply | 9-12V DC |

## Getting Started

### Hardware Requirements

- SM Energy FE102M Energy Monitor
- Compatible CT clamp (SCT-013-000, SCT-006, etc.)
- 9-12V DC power supply
- USB cable (for programming and debugging)

### Software Requirements

- [Arduino IDE](https://www.arduino.cc/en/software)
- ESP32 board support
- FE102M library (this repository)

### Installation

1. Install the [Arduino IDE](https://www.arduino.cc/en/software)
2. Add ESP32 board support to Arduino IDE:
   - Open Arduino IDE
   - Go to File > Preferences
   - Add `https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json` to Additional Board Manager URLs
   - Go to Tools > Board > Board Manager
   - Search for "esp32" and install the ESP32 package
3. Install required libraries:
   - Go to Sketch > Include Library > Manage Libraries
   - Install the following libraries:
     - DHT sensor library
     - ModbusRTU (by eModbus)
     - AsyncTCP
     - ESPAsyncWebServer
     - ArduinoJson
4. Clone or download this repository
5. Copy the `FE102M` folder to your Arduino libraries folder

### Basic Usage

```cpp
#include <FE102M.h>

// Create FE102M instance
FE102M energyMonitor;

void setup() {
  Serial.begin(115200);
  
  // Initialize the energy monitor
  energyMonitor.begin();
}

void loop() {
  // Read energy and environmental data
  energyMonitor.readEnergyData();
  energyMonitor.readEnvironmentalData();
  
  // Print measurements
  Serial.print("Voltage: ");
  Serial.print(energyMonitor.getLineVoltage());
  Serial.println(" V");
  
  Serial.print("Current: ");
  Serial.print(energyMonitor.getLineCurrent());
  Serial.println(" A");
  
  Serial.print("Active Power: ");
  Serial.print(energyMonitor.getActivePower());
  Serial.println(" W");
  
  // Update the display
  energyMonitor.updateDisplay();
  
  delay(1000);
}
```

## Example Code

The repository includes several example sketches to demonstrate the use of the FE102M Energy Monitor:

- [Basic Energy Monitoring](examples/FE102M_Basic_Example/FE102M_Basic_Example.ino): Simple reading of voltage, current, and power
- [WiFi Data Logging](examples/FE102M_WiFi_Example/FE102M_WiFi_Example.ino): Send energy data to a web server via WiFi
- [MODBUS Interface](examples/FE102M_MODBUS_Example/FE102M_MODBUS_Example.ino): Example for using the MODBUS communication
- [Calibration](examples/FE102M_Calibration/FE102M_Calibration.ino): How to calibrate the energy monitor for maximum accuracy

## Documentation

- [API Reference](docs/API_Reference.md): Complete library API documentation
- [Calibration Guide](Docs/FE102M_Calibration_Guide.md): Detailed instructions for calibrating the FE102M
- [ATM90E26 Datasheet](/Datasheets_Information/ATM90E26_Datasheet.pdf): Energy IC specifications
- [Hardware Reference](docs/Hardware_Reference.md): Hardware details and pin mapping
- [Web Interface Guide](docs/Web_Interface_Guide.md): Setting up and using the WiFi web interface

## Supported Current Transformers

The FE102M Energy Monitor is compatible with the following CT clamps:

| Model | Specification | Jumper Configuration |
|-------|---------------|---------------------|
| SCT-006 | 20A/25mA | JP1 and JP2 shorted |
| SCT-013-030 | 30A/1V | JP1 and JP2 open |
| SCT-013-050 | 50A/1V | JP1 and JP2 open |
| SCT-010 | 80A/26.6mA | JP1 and JP2 shorted |
| SCT-013-000 | 100A/50mA | JP1 and JP2 shorted |
| SCT-016 | 120A/40mA | JP1 and JP2 shorted |
| SCT-024 | 200A/100mA | JP1 and JP2 shorted |
| SCT-024 | 200A/50mA | JP1 and JP2 shorted |

**Note**: JP1 and JP2 jumpers configuration determines whether the CT input is configured for current output or voltage output sensors. Refer to the documentation for proper settings.

## Repository Structure

```
FE102M-Energy-Monitor/
├── examples/                  # Example Arduino sketches
│   ├── FE102M_Basic_Example/
│   ├── FE102M_WiFi_Example/
│   ├── FE102M_MODBUS_Example/
│   └── FE102M_Calibration/
├── src/                       # Library source code
│   ├── FE102M.h
│   └── FE102M.cpp
├── docs/                      # Documentation
│   ├── images/
│   ├── datasheets/
│   ├── API_Reference.md
│   ├── Calibration_Guide.md
│   ├── Hardware_Reference.md
│   └── Web_Interface_Guide.md
├── web/                       # Web interface files for WiFi server
│   ├── index.html
│   ├── css/
│   └── js/
├── LICENSE                    # License file
└── README.md                  # This file
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

***
**To buy this product please go to link below;**
https://www.ebay.com/itm/364752632441?itmmeta=01HQFTH7M3MHXT4NJK38QJCGWM&hash=item54ecf37a79:g:GPYAAOSw1thl2nWW&itmprp=enc%3AAQAIAAAAoNAZ4YFZEGNemjINAiAc2TXCD%2FZ9bZbDn9b%2F6fbWSt6ACkSpT0qp2crkCRfwgh3qR9JtyJ0Z2Nsg27rwpCdgPi0evkEj%2BymLC0yI%2BM%2Frz4GUaUa2ZwZg1ha5jvhLDkzK1U4i2VcE2z2E0IoohdqPBfaUZbl7f2i6vejEGGpUZnrjYWwH5Pj7mhnDFA%2FkL3x7szatGkkJ6tqpZwT2gfWYA6A%3D%7Ctkp%3ABFBMpPrE-rtj

We value our Customers, Users of our designs, and STEM Communities, all over the World. Should you have any other questions, or feedback to share with others, please feel free to:

***Chamil Vithanage, Microcode Embedded Solutions, Australia.***

Electronics Engineer | Software Developer | R&D Support | RF Engineering | Product Certification and Testing 

