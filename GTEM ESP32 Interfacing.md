**SM Energy FE102M Energy Monitoring System**

------------

**ESP32 Interfacing**

The ESP 32 interfaces to the ATM90E26 via either:
- SPI
- UART (connected to ESP32 UART2)

*This is user selectable using J3 and SW5 Com Mode Selector Switch  on the FE102M Main board.  By default, SPI is selected.*

The ESP32 interfaces to the AT24C256 EEPROM via I2C.  It has an address of 0x50

Flashing and Debugging connection to the ESP32, is via the USB UART (CP2102N).

**Pads**

The GTEM board has a number of solder pads on the bottom.

Default settings:
- **SPI** ATM90E26 Interface Enabled from **J3 and SW5**
- **Metering Mode L+N** 
- **CT** Clamp **Burden** Resistor **Enabled**
- **SPI Pullup Enebled (Default)**



The defaults are shown on below image.

![Display-Type-B](https://github.com/Chamil1983/GTEM-Grid-Tie-Energy-Monitor/blob/main/Datasheets%20and%20Information/GTEM%20Power%20Energy%20Monitor%20Board%20Overview.jpg?raw=true)


**ESP32 GPIO Configuration**

- **21**	**SDA**	I2C.  Pulled High via 4.7K
- **22**	**SCL**	I2C.  Pulled High via 4.7K

------------


- **12**		**RUN LED**	Strapping Pin. Boot fail if pulled HIGH
- **2**	    **FAULT LED**	Strapping Pin. Connected to on-board ESP LED
- **14**		**WiFi LED**	Note: ESP outputs PWM at boot
- **15**		**MODBUS LED**	Strapping Pin. Outputs PWM at boot

------------


- **19**	**MISO**		V-SPI
- **23**	**MOSI**		V-SPI
- **18**	**SCK**	V-SPI
- **5**		**CS**	V-SPI.  Strapping Pin. Outputs PWM at boot

------------
- **1**	TX0	**USB_TXD**	Debug output at boot
- **3**	RX0	**USB_RXD**	HIGH at boot

------------


- **17**	TX2	**TTL_TXD**	MAX485 DI pin (TXD)
- **16**	RX2	**TTL_RXD**	MAX485 RO pin (RXD)

------------


- **0**	**GP0- ESP32 Boot programming Enable**	Strapping Pin. Outputs PWM at boot
- **13**		**MAX485 TXRX Control pin for MODBUS (RS485)**	
- **4**	**DHT22 (AM2302) Sensor Data pin**	Strapping Pin. 

------------


- **25**	ADC 2 CH8	**ATM_CF2** Reactive Energy Pulse Output Interrupt.	Ignore 0 - 0.1V and 3.2 - 3.3V
- **32**	ADC 1 CH4	**ATM_ZX**	Voltage/Current Zero-Crossing Output Interrupt. Ignore 0 - 0.1V and 3.2 - 3.3V
- **33**	ADC 1 CH5	**MODE input Switch**	Ignore 0 - 0.1V and 3.2 - 3.3V
- **34**	ADC 1 CH6	**ATM_WO**	WarnOut: Fatal Error Warning Interrupt. Input only.Ignore 0 - 0.1V and 3.2 - 3.3V
- **35**	ADC 1 CH7	**ATM_CF1**	CF1: Active Energy Pulse Output Interrupt. input only. Ignore 0 - 0.1V and 3.2 - 3.3V
- **36**	ADC 1 CH0	**SET input Switch.**	Input only. Ignore 0 - 0.1V and 3.2 - 3.3V
- **39**	ADC 1 CH3	**ATM_IRQ Interrupt.**	Input only. Ignore 0 - 0.1V and 3.2 - 3.3V
------------


- **26**	GPIO26 **ATM90E26 UART RX pin**	(Should be selected from SW5 Switch to UART Mode and J3 UART side)
- **27**	GPIO27 **ATM90E26 UART TX pin**	 (Should be selected from SW5 Switch to UART Mode and J3 UART side)


------------

The below GPIO must be in the following states when ESP32 boots, or it will fail to boot.

- GP0	High
- GP3	High
- GP5	High
- GP12	Low
- GP15	High



Further information can be found:

- **GitHub**  https://github.com/Chamil1983/



