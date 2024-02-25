#pragma once
//Modbus_Defs.h+

//#include <ModbusSerial.h>
//#include <Modbus.h>
#include "HardwareSerial.h"
#include <ModbusRTU.h>

ModbusRTU MODBUS;

#define Modbus_Port Serial2 // define serial port used, Serial most of the time, or Serial1, Serial2 ... if available

const uint8_t TxenPin = 13;
const uint8_t SLAVE_ID = 1;
const uint32_t Baudrate = 38400;



const uint8_t rxPin = 16;
const uint8_t txPin = 17;


//ModbusSerial MODBUS(Modbus_Port, SlaveId, TxenPin);



long ts;






//==============================================================================================================
// Hardware Variables Digital Outputs
uint16_t LEDCoils[4], RelayCoils[8];
uint16_t Soft_Reset, CF1, CF2, ZX, IRQ, WO;
uint16_t WiFi_State, Fault_State, EEPROM_Connected_State, EEPROM_Bussy_State, DHT_State;

#define Soft_Reset	0x0100 //Read/Write Coil

#define LEDCoils[0] 0x0200
#define LEDCoils[1] 0x0201
#define LEDCoils[2] 0x0202
#define LEDCoils[3] 0x0203


#define RelayCoils[0] 0x0220
#define RelayCoils[1] 0x0221
#define RelayCoils[2] 0x0222
#define RelayCoils[3] 0x0223
#define RelayCoils[4] 0x0224
#define RelayCoils[5] 0x0225
#define RelayCoils[6] 0x0226
#define RelayCoils[7] 0x0227

#define CF1 0x0300
#define CF2 0x0301
#define ZX 0x0302
#define IRQ 0x0303
#define WO 0x0304

#define WiFi_State 0x0305
#define Fault_State 0x0306
#define EEPROM_Connected_State 0x0307
#define EEPROM_Bussy_State 0x0308
#define DHT_State 0x0309

//==============================================================================================================

// Hardware Variables Discrete Inputs

uint16_t Mode_Switch, Set_Switch;
uint16_t External_Inputs[10];




//Read Device Identification
#define VendorNameAdd           0x000  //10	R
#define ProductCodeAdd          0x01E  //10	R
#define RevisionNumberAdd		0x02D  //5	R



#define meter_nameAdd		0x082	//Iregister 130
#define modelAdd			0x096	//Iregister
#define manufacturerAdd		0x0AA	//Iregister	
#define serial_noAdd		0x0C8	//Iregister		
#define hardware_revAdd		0x0E6	//Iregister
#define firmware_verAdd		0x0EE	//Iregister	
#define mac_addAdd			0x102	//Iregister
#define nodeAdd				0x120	//Iregister
#define node_IDAdd			0x13E	//Iregister



#define meterDate_TimeAdd       240  //40


#define EnergyPulseDurationAdd				0x0850  //uint16_t 1 reg
#define ActiveEPulseOutputEnableAdd			0x0852  //uint16_t 1 reg
#define ReactiveEPulseOutputEnableAdd       0x0854  //uint16_t 1 reg

//Communications

#define ProtocolAdd					0x1901 //
#define Protocol_AddressAdd			0x1902 //
#define BaudRateAdd					0x1903 //
#define ParityAdd					0x1904 //

//Digital Inputs

#define DigitalInput_1CtrlModeAdd	0x1C69 //
#define DigitalInput_2CtrlModeAdd	0x1C81 //
#define DigitalInputStatusAdd		0x22C8 //

//Current, voltage, power, power factor and frequency

#define L_LineCurrentrmsAdd		0 //
#define N_LineCurrentrmsAdd		2 //
#define VoltagermsAdd			4 //
#define VoltageFrequencyAdd		6 //




/*
enum
{
	L_LineCurrentrmsAdd_A = 0x07D1,
	L_LineCurrentrmsAdd_B,
	N_LineCurrentrmsAdd_A,
	N_LineCurrentrmsAdd_B,
	VoltagermsAdd_A,
	VoltagermsAdd_B,
	VoltageFrequencyAdd_A,
	VoltageFrequencyAdd_B,

	IPWRREGS_SIZE
};

uint16_t InputPwrReg[IPWRREGS_SIZE];
*/



union
{
	long Val;
	struct
	{	
		uint16_t L_LineCurrentrms_A;
		uint16_t L_LineCurrentrms_B;
	};

}ILrms;

union
{
	long Val;
	struct
	{
		uint16_t N_LineCurrentrms_A;
		uint16_t N_LineCurrentrms_B;
	};

}INrms;

union
{
	long Val;
	struct
	{
		uint16_t Voltagerms_A;
		uint16_t Voltagerms_B;
	};

}Vrms;


union
{
	long Val;
	struct
	{
		uint16_t VoltageFrequency_A;
		uint16_t VoltageFrequency_B;
	};

}FREQ;

//================================================================================================

union
{
	long Val;
	struct
	{
		uint16_t L_LineMeanActivePower_A;
		uint16_t L_LineMeanActivePower_B;
	};

}L_MeanActPwr;

union
{
	long Val;
	struct
	{
		uint16_t L_LineMeanReactivePower_A;
		uint16_t L_LineMeanReactivePower_B;
	};

}L_MeanReactPwr;

union
{
	long Val;
	struct
	{
		uint16_t L_LineMeanApparentPowerPower_A;
		uint16_t L_LineMeanApparentPowerPower_B;
	};

}L_MeanapePwr;

union
{
	long Val;
	struct
	{
		uint16_t N_LineMeanActivePower_A;
		uint16_t N_LineMeanActivePower_B;
	};

}N_MeanActPwr;

union
{
	long Val;
	struct
	{
		uint16_t N_LineMeanReactivePower_A;
		uint16_t N_LineMeanReactivePower_B;
	};

}N_MeanReactPwr;

union
{
	long Val;
	struct
	{
		uint16_t N_LineMeanApparentPowerPower_A;
		uint16_t N_LineMeanApparentPowerPower_B;
	};

}N_MeanapePwr;

union
{
	long Val;
	struct
	{
		uint16_t L_LinePowerFactor_A;
		uint16_t L_LinePowerFactor_B;
	};

}PfacL;

union
{
	long Val;
	struct
	{
		uint16_t N_LinePowerFactor_A;
		uint16_t N_LinePowerFactor_B;
	};

}PfacN;

union
{
	long Val;
	struct
	{
		uint16_t PhaseAngL_A;
		uint16_t PhaseAngL_B;
	};

}PangL;

union
{
	long Val;
	struct
	{
		uint16_t PhaseAngN_A;
		uint16_t PhaseAngN_B;
	};

}PangN;



//================================================================================================


#define L_LineMeanActivePowerAdd				8 //
#define L_LineMeanReactivePowerAdd				10 //
#define L_LineMeanApparentPowerPowerAdd			12 //
#define N_LineMeanActivePowerAdd				14 //
#define N_LineMeanReactivePowerAdd				16 //
#define N_LineMeanApparentPowerAdd				18 //
#define L_LinePowerFactorAdd					20 //
#define N_LinePowerFactorAdd					22 //
#define PhaseAngLAdd							24 //
#define PhaseAngNAdd							26 //

//#define TemperatureAdd			0x0835 // 
#define HumidityAdd				32 //
#define TemperatureFAdd			34 //
#define TemperatureFAFAdd		36 //
#define HicAdd					38 //
#define HifAdd					40 //


long scale = 10;
long lconv = 0;



enum
{
	TempA  = 20,      
	TempB,      
	HumA,      
	HumB,      
	TempFA,     
	TempFB,     

	INPUT_REGS_SIZE
};

uint16_t InputReg[INPUT_REGS_SIZE];


//Energy and input metering
#define EnergyRstDate_TimeAdd					0x0CB3 //

#define ForwardActiveEnergyAdd					61 //
#define ReverseActiveEnergyAdd					63 //
#define AbsoluteActiveEnergyAdd					65 //
#define Forward_InductiveReactiveEnergyAdd		67 //
#define Reverse_CapacitiveReactiveEnergyAdd		69 //
#define AbsoluteReactiveEnergyAdd				71 //

#define MeteringStatusAdd						75 //




union
{
	unsigned long Val;
	struct
	{
		uint16_t ForwardActiveEnergy_A;
		uint16_t ForwardActiveEnergy_B;
	};

}ImpEnergy;

union
{
	unsigned long Val;
	struct
	{
		uint16_t ReverseActiveEnergy_A;
		uint16_t ReverseActiveEnergy_B;
	};

}ExpEnergy;


union
{
	unsigned long Val;
	struct
	{
		uint16_t AbsoluteActiveEnergy_A;
		uint16_t AbsoluteActiveEnergy_B;
	};

}AbsActEnergy;

union
{
	unsigned long Val;
	struct
	{
		uint16_t Forward_InductiveReactiveEnergy_A;
		uint16_t Forward_InductiveReactiveEnergy_B;
	};

}ReactfwdEnergy;


union
{
	unsigned long Val;
	struct
	{
		uint16_t Reverse_CapacitiveReactiveEnergy_A;
		uint16_t Reverse_CapacitiveReactiveEnergy_B;
	};

}ReactrvsEnergy;


union
{
	unsigned long Val;
	struct
	{
		uint16_t AbsoluteReactiveEnergy_A;
		uint16_t AbsoluteReactiveEnergy_B;
	};

}AbsReactEnergy;



//Status,Metering Calibration and Configuration

#define SysStatusAdd						24 // R
#define FuncEnAdd							25 // R

#define VoltageSagThresholdAdd				0 //
#define L_LineCalibrationGainAdd			1 //
#define L_LineCalibrationAngleAdd			2 //
#define N_LineCalibrationGainAdd			3 //
#define N_LineCalibrationAngleAdd			4 //
#define ActiveStartupPowerThresholdAdd		5 //
#define ActiveNoLoadPowerThresholdAdd		6 //
#define ReactiveStartupPowerThresholdAdd	7 //
#define ReactiveNoLoadPowerThresholdsAdd	8 //
#define MeasurementCalibrationStartAdd		9 //
#define VoltagermsGainAdd					10 //
#define L_LineCurrentrmsGainAdd				11 //
#define VoltageOffsetAdd					12 //
#define L_LineCurrentOffsetAdd				13 //
#define N_LineCurrentOffsetAdd				14 //
#define L_LineActivePowerOffsetAdd			15 //
#define L_LineReactivePowerOffsetAdd		16 //
#define N_LineActivePowerOffsetAdd			17 //
#define N_LineReactivePowerOffsetAdd		18 //
#define N_LineCurrentrmsGainAdd				19 //

#define Checksum1Add						22 //
#define Checksum2Add						23 //





