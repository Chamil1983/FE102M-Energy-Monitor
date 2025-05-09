/*
  Dave Williams, DitroniX 2019-2022 (ditronix.net)
  GTEM-1 ATM90E26 Energy Monitoring Energy Monitor  v1.0
  Features include ESP32 GTEM ATM90E26 16bit ADC EEPROM OPTO CT-Clamp Current Voltage Frequency Power Factor GPIO I2C OLED SMPS D1 USB
  PCA 1.2212-104 - Test Code Firmware v1 - 30th December 2022

  The purpose of this test code is to cycle through the various main functions of the board, as shown below, as part of board bring up testing.

  Simplified Board Bring Up Test -GTEM ATM90E26 Energy Monitor ASIC - Basic Calibration Limits.  Requires Calibration.
  Additional diagnostic serial reporting has been included, for reference and expanded detail.

  Code Based on the excellent work from Tisham Dhar, whatnick | ATM90E26 Energy Monitor | Updated Date Williams

  Remember!
  Set the BOARD to ESP32, 'WEMOS D1 MINI ESP32' DEV Module (or similar).
  You may set the BAUD rate to 921600 to speed up flashing.
  The SDK does NOT need external power to flash.  It will take Power from the USB 5V.

  Note: In the default state, upon first power up and during reset, the Green LED may be partially lit. Once programmed and the GPIO defined, the Green LED will go off after power up.

  This test code is OPEN SOURCE and formatted for easier viewing.  Although is is not intended for real world use, it may be freely used, or modified as needed.
  It is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.

  Further information, details and examples can be found on our website wiki pages ditronix.net/wiki and github.com/DitroniX
*/

// Libraries
#include <Arduino.h>
#include <SPI.h>

// ****************  VARIABLES / DEFINES / STATIC / STRUCTURES / CONSTANTS ****************

// Registers
#define SoftReset 0x00  //Software Reset
#define SysStatus 0x01  //System Status
#define FuncEn 0x02     //Function Enable
#define SagTh 0x03      //Voltage Sag Threshold
#define SmallPMod 0x04  //Small-Power Mode
#define LastData 0x06   //Last Read/Write SPI/UART Value
#define LSB 0x08        //RMS/Power 16-bit LSB
#define CalStart 0x20   //Calibration Start Command
#define PLconstH 0x21   //High Word of PL_Constant
#define PLconstL 0x22   //Low Word of PL_Constant
#define Lgain 0x23      //L Line Calibration Gain
#define Lphi 0x24       //L Line Calibration Angle
#define Ngain 0x25      //N Line Calibration Gain
#define Nphi 0x26       //N Line Calibration Angle
#define PStartTh 0x27   //Active Startup Power Threshold
#define PNolTh 0x28     //Active No-Load Power Threshold
#define QStartTh 0x29   //Reactive Startup Power Threshold
#define QNolTh 0x2A     //Reactive No-Load Power Threshold
#define MMode 0x2B      //Metering Mode Configuration
#define CSOne 0x2C      //Checksum 1
#define AdjStart 0x30   //Measurement Calibration Start Command
#define Ugain 0x31      //Voltage rms Gain
#define IgainL 0x32     //L Line Current rms Gain
#define IgainN 0x33     //N Line Current rms Gain
#define Uoffset 0x34    //Voltage Offset
#define IoffsetL 0x35   //L Line Current Offset
#define IoffsetN 0x36   //N Line Current Offset
#define PoffsetL 0x37   //L Line Active Power Offset
#define QoffsetL 0x38   //L Line Reactive Power Offset
#define PoffsetN 0x39   //N Line Active Power Offset
#define QoffsetN 0x3A   //N Line Reactive Power Offset
#define CSTwo 0x3B      //Checksum 2
#define APenergy 0x40   //Forward Active Energy
#define ANenergy 0x41   //Reverse Active Energy
#define ATenergy 0x42   //Absolute Active Energy
#define RPenergy 0x43   //Forward (Inductive) Reactive Energy
#define Rnenerg 0x44    //Reverse (Capacitive) Reactive Energy
#define Rtenergy 0x45   //Absolute Reactive Energy
#define EnStatus 0x46   //Metering Status
#define Irms 0x48       //L Line Current rms
#define Urms 0x49       //Voltage rms
#define Pmean 0x4A      //L Line Mean Active Power
#define Qmean 0x4B      //L Line Mean Reactive Power
#define Freq 0x4C       //Voltage Frequency
#define PowerF 0x4D     //L Line Power Factor
#define Pangle 0x4E     //Phase Angle between Voltage and L Line Current
#define Smean 0x4F      //L Line Mean Apparent Power
#define IrmsTwo 0x68    //N Line Current rms
#define PmeanTwo 0x6A   //N Line Mean Active Power
#define QmeanTwo 0x6B   //N Line Mean Reactive Power
#define PowerFTwo 0x6D  //N Line Power Factor
#define PangleTwo 0x6E  //Phase Angle between Voltage and N Line Current
#define SmeanTwo 0x6F   //N Line Mean Apparent Power

// pins used for the connection with the sensor
// the other you need are controlled by the SPI library):
// const int energy_IRQ = 2;
#if defined(ARDUINO_ESP8266_WEMOS_D1MINI) // WeMos mini and D1 R2
const int energy_CS = D8;                 // WEMOS SS pin
#elif defined(ARDUINO_ESP8266_ESP12)      // Adafruit Huzzah
const int energy_CS = 15; // HUZZAH SS pins ( 0 or 15)
#elif defined(ARDUINO_ARCH_SAMD)          // M0 board
const int energy_CS = 10; // M0 SS pin
#elif defined(__AVR_ATmega32U4__)         // 32u4 board
const int energy_CS = 10; // 32u4 SS pin
#else
const int energy_CS = SS; // Use default SS pin for unknown Arduino
#endif // defined(ARDUINO_ESP8266_WEMOS_D1MINI)

// **************** FUNCTIONS / ROUTINES / CLASSES ****************
class ATM90E26_SPI {
public:
	ATM90E26_SPI(int pin = energy_CS);

	double GetLineVoltage();
	double GetLineCurrent();
	double GetActivePower();
	double GetFrequency();
	double GetPowerFactor();
	double GetReactivePower();
	double GetApparentPower();
	double GetPhaseAngle();
	double GetImportEnergy();
	double GetExportEnergy();
	double GetAbsActiveEnergy();
	double GetAbsReactiveEnergy();
	double GetReactivefwdEnergy();
	double GetReactivervsEnergy();
	double GetNLineCurrent();
	double GetNLineActivePower();
	double GetNLineReactivePower();
	double GetNLinePowerFactor();
	double GetNLinePhaseAngle();
	double GetNLineApparentPower();

	void SetUGain(unsigned short);
	void SetLGain(unsigned short);
	void SetIGain(unsigned short);
	void SetCRC1(unsigned short);
	void SetCRC2(unsigned short);
	void InitEnergyIC();
	void InitCallibrate();



	unsigned short GetSysStatus();
	unsigned short GetMeterStatus();
	unsigned short GetCalStartStatus();
	unsigned short GetLSBStatus();
	unsigned short GetMModeStatus();
	unsigned short GetCS1Status();
	unsigned short GetCS2Status();
	unsigned short GetCS1Calculated();
	unsigned short GetCS2Calculated();

	unsigned short GetRegisterValue(unsigned char address);


	unsigned short GetUGain();
	unsigned short GetLGain();
	unsigned short GetIGain();

public:
	unsigned short CommEnergyIC(unsigned char RW, unsigned char address, unsigned short val);
	int _cs;
	unsigned short _lgain;
	unsigned short _ugain;
	unsigned short _igain;
	unsigned short _crc1;
	unsigned short _crc2;
	unsigned short _actpwrthr;
	
};
