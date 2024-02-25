// Modbus_Serial.h

#ifndef _MODBUS_SERIAL_h
#define _MODBUS_SERIAL_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#include <darray.h>
#include <Modbus.h>
#include <ModbusAPI.h>
#include <ModbusRTU.h>
#include <ModbusSettings.h>



extern uint16_t GetHoldreg(TRegister* reg, uint16_t val);
extern uint16_t SetHoldreg(TRegister* reg, uint16_t val);


extern float GetIreg(TRegister* reg, float val);
extern float SetIreg(TRegister* reg, float val);

extern void Modbus_Poling(void);

extern void Load_Reg_CalConfiguration();
extern void Load_Power_Registers(void);
extern void Setup_Modbus(void);
extern void InitModbus(void);
extern void Configure_MODBUS_Registers(void);
String getMacAddress();
extern uint16_t Read_EnergyDataUnsigned(unsigned char address);
extern int16_t Read_EnergyDataSigned(unsigned char address);
extern void floatconv(float input, uint16_t* ha, uint16_t* hb);
extern void led_ModbusFlash(int ton, int toff);
//extern char Stringconv(char* input, char strconv[30]);
//extern char StrtoCharconv(String str);
#endif

