## SM Energy ESP32 based ATM90E26 Energy Monitor SDK Model FE102M


GTEM Overview
![Display-Type-B](https://github.com/Chamil1983/FE102M-Energy-Monitor/blob/main/Pictures/IMG_4226.jpg?raw=true)

![Display-Type-B](https://github.com/Chamil1983/FE102M-Energy-Monitor/blob/main/Pictures/IMG_4206.jpg?raw=true)
![Display-Type-B](https://github.com/Chamil1983/FE102M-Energy-Monitor/blob/main/Pictures/IMG_4207.jpg?raw=true)

**FE102M Power Energy Monitor Board SDK Overview**

**FE102M Main Board**
![Display-Type-B](Main_Board.jpg?raw=true)

**FE102M Front LCD Board**
![Display-Type-B](Front_Panel.jpg?raw=true)


**The SM Energy FE102M Energy Monitor has the main features:**

- ESP32 WROOM 32 -Wireless and Bluetooth
ATM90E26 (Energy Monitor) Metering Features:
- Metering features are fully in compliance with the requirements of IEC62052-11, IEC62053-21, and IEC62053-23.
- Interrupts connected to ESP32 Active Energy pulse interrupt, Reactive Energy pulse interrupt, WarnOut fault interrupt, Voltage/Current Zero Crossing interrupt.
- Accuracy of ±0.1% for active energy and ±0.2% for reactive energy over a dynamic range of 6000:1.
- The temperature coefficient is 6 ppm/ ℃ (typ.) for on-chip reference voltage. Automatically temperature compensated.
- Single-point calibration on each phase over the whole dynamic range for active energy; no calibration is needed for reactive/ apparent energy. 
- Flexible piece-wise non-linearity compensation: three current (RMS value)-based segments with two programmable thresholds for each phase. Independent gain and phase angle compensation for each segment. 
- Electrical parameters measurement: less than ±0.5% fiducial error for Vrms, Irms, mean active/ reactive/ apparent power, frequency, power factor, and phase angle. 
- Active (forward/reverse), reactive (forward/reverse), apparent energy with independent energy registers. • Programmable startup and no-load power thresholds
- 24C256 EEPROM (I2C)
- I2C 20X4 LCD Module 
- Current Clamp Input
**Compatible CT clamps (20A/25mA SCT-006, 30A/1V SCT-013-030 (JP1 and JP2 jumper Should be Open) ,50A/1V SCT-013-050 (JP1 and JP2  jumper Should be Open), 80A/26.6mA SCT-010,100A/50mA SCT-013-000,120A/40mA: SCT-016,200A/100mA SCT-024, 200A/50mA SCT-024)**
- AC Low Voltage Input (Voltage < 250V)

- **Arduino ESP32 Flashing and Programming Compatibility**
- OPTO Outputs
**Meter Active and Reactive Energy pulse**
- On Board DHT22 Temperature and Humidity Sensor
- Reset Button
- **MODBUS and WiFi Connectivity**
- 2 X User GPIO (SET and MODE User Programmable Button)
- Meter status LEDs and Power LED.
- Meter designed to Auto/Manual program and serial communication through a USB interface. 
- Size 150mm x 105mm x 35mm
    
**Remember!**
- Our SDKs for the ESP32 Dev module are fully compatible with Arduino IDE
- Set the BOARD to ESP32 Dev Module, 'WEMOS D1 MINI ESP32' DEV Module (or similar).
- You can also set the BAUD rate to 921600 to speed up flashing.
- The SDK does need external power to flash.  First, Connect the DC power 9-12V and Select the serial com port in Arduino.

***You will need to provide an external 12V DC to power up the Energy Monitor functions.*
*You will need to provide a CT Current Clamp.  Ideally YHDC SCT-013-000***

All test code is OPEN SOURCE and although it is not intended for real-world use, it may be freely used, or modified as needed.  It is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.

See Example Code https://github.com/Chamil1983/FE102M-Energy-Monitor/tree/main/Sample_Code/


## **Further Information**

Additional information, and other technical details on this project, may be found in the related repository pages.

**Repository Folders**

 - **Code** *(Code examples for Arduino  IDE and Visual Studio)*
 -  **Datasheets and Information** *(Component Datasheets, Photos and Technical Documentation)*
 - **Certification** *(Related Repository Project or Part, Certification Information)*

**Repository Tabs**

 - **Wiki** *(Related Repository Wiki pages and Technical User Information)*
 - **Discussions** *(Related Repository User Discussion Forum)*
 - **Issues** *(Related Repository Technical Issues and Fixes)*

***
**To buy this product please go to link below;**
https://www.ebay.com/itm/364752632441?itmmeta=01HQFTH7M3MHXT4NJK38QJCGWM&hash=item54ecf37a79:g:GPYAAOSw1thl2nWW&itmprp=enc%3AAQAIAAAAoNAZ4YFZEGNemjINAiAc2TXCD%2FZ9bZbDn9b%2F6fbWSt6ACkSpT0qp2crkCRfwgh3qR9JtyJ0Z2Nsg27rwpCdgPi0evkEj%2BymLC0yI%2BM%2Frz4GUaUa2ZwZg1ha5jvhLDkzK1U4i2VcE2z2E0IoohdqPBfaUZbl7f2i6vejEGGpUZnrjYWwH5Pj7mhnDFA%2FkL3x7szatGkkJ6tqpZwT2gfWYA6A%3D%7Ctkp%3ABFBMpPrE-rtj

We value our Customers, Users of our designs, and STEM Communities, all over the World. Should you have any other questions, or feedback to share with others, please feel free to:

***Chamil Vithanage, Microcode Embedded Solutions, Australia.***

Electronics Engineer | Software Developer | R&D Support | RF Engineering | Product Certification and Testing 

