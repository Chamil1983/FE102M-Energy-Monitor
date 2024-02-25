 
#include "Modbus_Serial.h" 
#include <MicrocontrollerID.h> 
#include "config.h" 
#include <WiFi.h> 
#include <WiFiUdp.h> 
#include <WiFiServer.h> 
#include <WiFiScan.h> 
#include <WiFiMulti.h> 
#include <WiFiClient.h> 
#include <WiFiAP.h> 
#include "EnergyATM90E26.h" 
#include "Modbus_Defs.h" 
 
 
 
 
void Setup_Modbus(void) { 
 
 
	Modbus_Port.begin(Baudrate, SERIAL_8E1);
 
 
 
#if defined(ESP32) || defined(ESP8266) 
	MODBUS.begin(&Modbus_Port, TxenPin); 
#else 
	// mb.begin(&Serial); 
	mb.begin(&Serial2, RXD2, TXD2); //or use RX/TX direction control pin (if required) 
	mb.setBaudrate(9600); 
#endif 
 
 
 
	//MODBUS.begin(&Modbus_Port,TxenPin); 
	MODBUS.setBaudrate(Baudrate); 
	MODBUS.slave(SLAVE_ID);
	Configure_MODBUS_Registers();
	InitModbus();

	 
 
 
	 
} 
 
 
 
 
void Configure_MODBUS_Registers(void) { 
 
 
	Load_Reg_CalConfiguration();
	Load_Power_Registers(); 
	 
	 
	MODBUS.addHreg(VoltageSagThresholdAdd, VoltageSagThreshold);


	MODBUS.addHreg(VoltageSagThresholdAdd, VoltageSagThreshold); 
	MODBUS.addHreg(L_LineCalibrationGainAdd, L_LineCalibrationGain); 
	MODBUS.addHreg(L_LineCalibrationAngleAdd, L_LineCalibrationAngle); 
	MODBUS.addHreg(N_LineCalibrationGainAdd, N_LineCalibrationGain); 
	MODBUS.addHreg(N_LineCalibrationAngleAdd, N_LineCalibrationAngle); 
	MODBUS.addHreg(ActiveStartupPowerThresholdAdd, ActiveStartupPowerThreshold); 
	MODBUS.addHreg(ActiveNoLoadPowerThresholdAdd, ActiveNoLoadPowerThreshold); 
	MODBUS.addHreg(ReactiveStartupPowerThresholdAdd, ReactiveStartupPowerThreshold); 
	MODBUS.addHreg(ReactiveNoLoadPowerThresholdsAdd, ReactiveNoLoadPowerThreshold); 
	MODBUS.addHreg(MeasurementCalibrationStartAdd, MeasurementCalibrationStart); 
	MODBUS.addHreg(VoltagermsGainAdd, VoltagermsGain); 
	MODBUS.addHreg(L_LineCurrentrmsGainAdd, L_LineCurrentrmsGain); 
	MODBUS.addHreg(VoltageOffsetAdd, VoltageOffset); 
	MODBUS.addHreg(L_LineCurrentOffsetAdd, L_LineCurrentOffset); 
	MODBUS.addHreg(N_LineCurrentOffsetAdd, N_LineCurrentOffset); 
	MODBUS.addHreg(L_LineActivePowerOffsetAdd, L_LineActivePowerOffset); 
	MODBUS.addHreg(L_LineReactivePowerOffsetAdd, L_LineReactivePowerOffset); 
	MODBUS.addHreg(N_LineActivePowerOffsetAdd, N_LineActivePowerOffset); 
	MODBUS.addHreg(N_LineReactivePowerOffsetAdd, N_LineReactivePowerOffset); 
	MODBUS.addHreg(N_LineCurrentrmsGainAdd, N_LineCurrentrmsGain); 
 
	MODBUS.addHreg(Checksum1Add, Checksum1); 
	MODBUS.addHreg(Checksum2Add, Checksum2); 
 
 
 
	ILrms.Val = lround(L_LineCurrentrms * 1000); 
	INrms.Val = lround(N_LineCurrentrms * 1000); 
	Vrms.Val = lround(Voltagerms * 100); 
	FREQ.Val = lround(VoltageFrequency * 100); 
 
 
	L_MeanActPwr.Val = lround(L_LineMeanActivePower * 1); 
	L_MeanReactPwr.Val = lround(L_LineMeanReactivePower * 1); 
	L_MeanapePwr.Val = lround(L_LineMeanApparentPower * 1); 
	N_MeanActPwr.Val = lround(N_LineMeanActivePower * 1); 
	N_MeanReactPwr.Val = lround(N_LineMeanReactivePower * 1); 
	N_MeanapePwr.Val = lround(N_LineMeanApparentPower * 1); 
 
	PfacL.Val = lround(L_LinePowerFactor * 1000); 
	PfacN.Val = lround(N_LinePowerFactor * 1000); 
	PangL.Val = lround(PhaseAnglebetweenVoltageandL_LineCurrent * 100); 
	PangN.Val = lround(PhaseAnglebetweenVoltageandN_LineCurrent * 100); 
 
 
	ImpEnergy.Val = lround(ForwardActiveEnergy * 10000); 
	ExpEnergy.Val = lround(ReverseActiveEnergy * 10000); 
	AbsActEnergy.Val = lround(AbsoluteActiveEnergy * 10000); 
	ReactfwdEnergy.Val = lround(Forward_InductiveReactiveEnergy * 10000); 
	ReactrvsEnergy.Val = lround(Reverse_CapacitiveReactiveEnergy * 10000); 
	AbsReactEnergy.Val = lround(AbsoluteReactiveEnergy * 10000); 
 
 
	
		//Assign Parameter registers for MODBUS 
 
	MODBUS.addIreg(L_LineCurrentrmsAdd, ILrms.Val,2); 
	MODBUS.addIreg(N_LineCurrentrmsAdd, INrms.Val, 2); 
	MODBUS.addIreg(VoltagermsAdd, Vrms.Val, 2); 
	MODBUS.addIreg(VoltageFrequencyAdd, FREQ.Val, 2); 
 
 
 
	/// <summary> 
	/// /////////////// 
	/// </summary> 
	/// <param name=""></param> 
 
	MODBUS.addIreg(L_LineMeanActivePowerAdd, L_MeanActPwr.Val,2); 
	MODBUS.addIreg(L_LineMeanReactivePowerAdd, L_MeanReactPwr.Val, 2); 
	MODBUS.addIreg(L_LineMeanApparentPowerPowerAdd, L_MeanapePwr.Val, 2); 
	MODBUS.addIreg(N_LineMeanActivePowerAdd, N_MeanActPwr.Val, 2); 
	MODBUS.addIreg(N_LineMeanReactivePowerAdd, N_MeanReactPwr.Val, 2); 
	MODBUS.addIreg(N_LineMeanApparentPowerAdd, N_MeanapePwr.Val, 2); 
	MODBUS.addIreg(L_LinePowerFactorAdd, PfacL.Val, 2); 
	MODBUS.addIreg(N_LinePowerFactorAdd, PfacN.Val, 2); 
	MODBUS.addIreg(PhaseAngLAdd, PangL.Val, 2); 
	MODBUS.addIreg(PhaseAngNAdd, PangN.Val, 2); 
	////////////////////////////////////////////////// 
 
 
	MODBUS.addIreg(ForwardActiveEnergyAdd, ImpEnergy.Val, 2); 
	MODBUS.addIreg(ReverseActiveEnergyAdd, ExpEnergy.Val, 2); 
	MODBUS.addIreg(AbsoluteActiveEnergyAdd, AbsActEnergy.Val, 2); 
	MODBUS.addIreg(Forward_InductiveReactiveEnergyAdd, ReactfwdEnergy.Val, 2); 
	MODBUS.addIreg(Reverse_CapacitiveReactiveEnergyAdd, ReactrvsEnergy.Val, 2); 
	MODBUS.addIreg(AbsoluteReactiveEnergyAdd, AbsReactEnergy.Val, 2); 
	

 
 
	MODBUS.addIreg(L_LineCurrentrmsAdd);
	MODBUS.addIreg(N_LineCurrentrmsAdd);
	MODBUS.addIreg(VoltagermsAdd);
	MODBUS.addIreg(VoltageFrequencyAdd);



	ILrms.Val = lround(L_LineCurrentrms * 1000);
	INrms.Val = lround(N_LineCurrentrms * 1000);
	Vrms.Val = lround(Voltagerms * 100);
	FREQ.Val = lround(VoltageFrequency * 100);




	/// <summary> 
	/// /////////////// 
	/// </summary> 
	/// <param name=""></param> 

	MODBUS.addIreg(L_LineMeanActivePowerAdd);
	MODBUS.addIreg(L_LineMeanReactivePowerAdd);
	MODBUS.addIreg(L_LineMeanApparentPowerPowerAdd);
	MODBUS.addIreg(N_LineMeanActivePowerAdd);
	MODBUS.addIreg(N_LineMeanReactivePowerAdd);
	MODBUS.addIreg(N_LineMeanApparentPowerAdd);
	MODBUS.addIreg(L_LinePowerFactorAdd);
	MODBUS.addIreg(N_LinePowerFactorAdd);
	MODBUS.addIreg(PhaseAngLAdd);
	MODBUS.addIreg(PhaseAngNAdd);
	MODBUS.addIreg(ForwardActiveEnergyAdd);
	MODBUS.addIreg(ReverseActiveEnergyAdd);
	MODBUS.addIreg(AbsoluteActiveEnergyAdd);
	MODBUS.addIreg(Forward_InductiveReactiveEnergyAdd);
	MODBUS.addIreg(Reverse_CapacitiveReactiveEnergyAdd);
	MODBUS.addIreg(AbsoluteReactiveEnergyAdd);

 


	floatconv(Temp, &InputReg[TempA], &InputReg[TempB]); 
	floatconv(Temp_F, &InputReg[TempFA], &InputReg[TempFB]); 
	floatconv(Hum, &InputReg[HumA], &InputReg[HumB]); 
 
	MODBUS.addIreg(TempA); 
	MODBUS.addIreg(TempB); 
	MODBUS.addIreg(TempFA); 
	MODBUS.addIreg(TempFB); 
	MODBUS.addIreg(HumA); 
	MODBUS.addIreg(HumB); 
	 
	 
} 
 
 
 
void InitModbus(void) { 
 
 
	MODBUS.Hreg(VoltageSagThresholdAdd, VoltageSagThreshold); 
	MODBUS.Hreg(L_LineCalibrationGainAdd, L_LineCalibrationGain); 
	MODBUS.Hreg(L_LineCalibrationAngleAdd, L_LineCalibrationAngle); 
	MODBUS.Hreg(N_LineCalibrationGainAdd, N_LineCalibrationGain); 
	MODBUS.Hreg(N_LineCalibrationAngleAdd, N_LineCalibrationAngle); 
	MODBUS.Hreg(ActiveStartupPowerThresholdAdd, ActiveStartupPowerThreshold); 
	MODBUS.Hreg(ActiveNoLoadPowerThresholdAdd, ActiveNoLoadPowerThreshold); 
	MODBUS.Hreg(ReactiveStartupPowerThresholdAdd, ReactiveStartupPowerThreshold); 
	MODBUS.Hreg(ReactiveNoLoadPowerThresholdsAdd, ReactiveNoLoadPowerThreshold); 
	MODBUS.Hreg(MeasurementCalibrationStartAdd, MeasurementCalibrationStart);
	MODBUS.Hreg(VoltagermsGainAdd, VoltagermsGain); 
	MODBUS.Hreg(L_LineCurrentrmsGainAdd, L_LineCurrentrmsGain); 
	MODBUS.Hreg(VoltageOffsetAdd, VoltageOffset); 
	MODBUS.Hreg(L_LineCurrentOffsetAdd, L_LineCurrentOffset); 
	MODBUS.Hreg(N_LineCurrentOffsetAdd, N_LineCurrentOffset); 
	MODBUS.Hreg(L_LineActivePowerOffsetAdd, L_LineActivePowerOffset); 
	MODBUS.Hreg(L_LineReactivePowerOffsetAdd, L_LineReactivePowerOffset); 
	MODBUS.Hreg(N_LineActivePowerOffsetAdd, N_LineActivePowerOffset); 
	MODBUS.Hreg(N_LineReactivePowerOffsetAdd, N_LineReactivePowerOffset); 
	MODBUS.Hreg(N_LineCurrentrmsGainAdd, N_LineCurrentrmsGain); 
 	MODBUS.Hreg(Checksum1Add, Checksum1); 
	MODBUS.Hreg(Checksum2Add, Checksum2); 


	 
 
	/// <summary> 
	/// /////////////// 
	/// </summary> 
	/// <param name=""></param> 



	MODBUS.Ireg(L_LineCurrentrmsAdd, L_LineCurrentrms);
	MODBUS.Ireg(N_LineCurrentrmsAdd, N_LineCurrentrms);
	MODBUS.Ireg(VoltagermsAdd, Voltagerms);
	MODBUS.Ireg(VoltageFrequencyAdd, VoltageFrequency);



	/// <summary>
	/// ///////////////////////////////////
	/// </summary>
	/// <param name=""></param>
	/// 
	/// 

 
	MODBUS.Ireg(L_LineMeanActivePowerAdd, L_LineMeanActivePower);
	MODBUS.Ireg(L_LineMeanReactivePowerAdd, L_LineMeanReactivePower);
	MODBUS.Ireg(L_LineMeanApparentPowerPowerAdd, L_LineMeanApparentPower);
	MODBUS.Ireg(N_LineMeanActivePowerAdd, N_LineMeanActivePower);
	MODBUS.Ireg(N_LineMeanReactivePowerAdd, N_LineMeanReactivePower);
	MODBUS.Ireg(N_LineMeanApparentPowerAdd, N_LineMeanApparentPower);
	MODBUS.Ireg(L_LinePowerFactorAdd, L_LinePowerFactor);
	MODBUS.Ireg(N_LinePowerFactorAdd, N_LinePowerFactor);
	MODBUS.Ireg(PhaseAngLAdd, PhaseAnglebetweenVoltageandL_LineCurrent);
	MODBUS.Ireg(PhaseAngNAdd, PhaseAnglebetweenVoltageandN_LineCurrent);
 	MODBUS.Ireg(ForwardActiveEnergyAdd, ForwardActiveEnergy);
	MODBUS.Ireg(ReverseActiveEnergyAdd, ReverseActiveEnergy);
	MODBUS.Ireg(AbsoluteActiveEnergyAdd, AbsoluteActiveEnergy);
	MODBUS.Ireg(Forward_InductiveReactiveEnergyAdd, Forward_InductiveReactiveEnergy);
	MODBUS.Ireg(Reverse_CapacitiveReactiveEnergyAdd, Reverse_CapacitiveReactiveEnergy);
	MODBUS.Ireg(AbsoluteReactiveEnergyAdd, AbsoluteReactiveEnergy);


 
 
 
	floatconv(Temp, &InputReg[TempA], &InputReg[TempB]); 
	floatconv(Temp_F, &InputReg[TempFA], &InputReg[TempFB]); 
	floatconv(Hum, &InputReg[HumA], &InputReg[HumB]); 
 
	MODBUS.Ireg(TempA, InputReg[TempA]); 
	MODBUS.onSetIreg(TempA, SetIreg);
	MODBUS.onGetIreg(TempA, GetIreg);
	MODBUS.Ireg(TempB, InputReg[TempB]); 
	MODBUS.onSetIreg(TempB, SetIreg);
	MODBUS.onGetIreg(TempB, GetIreg);
	MODBUS.Ireg(TempFA, InputReg[TempFA]); 
	MODBUS.onSetIreg(TempFA, SetIreg);
	MODBUS.onGetIreg(TempFA, GetIreg);
	MODBUS.Ireg(TempFB, InputReg[TempFB]); 
	MODBUS.onSetIreg(TempFB, SetIreg);
	MODBUS.onGetIreg(TempFB, GetIreg);
	MODBUS.Ireg(HumA, InputReg[HumA]); 
	MODBUS.onSetIreg(HumA, SetIreg);
	MODBUS.onGetIreg(HumA, GetIreg);
	MODBUS.Ireg(HumB, InputReg[HumB]); 
	MODBUS.onSetIreg(HumB, SetIreg);
	MODBUS.onGetIreg(HumB, GetIreg);

	/*
		MODBUS.Ireg(TemperatureFAdd, Temp);
	MODBUS.onSetIreg(TemperatureFAdd, SetIreg);
	MODBUS.onGetIreg(TemperatureFAdd, GetIreg);

	MODBUS.Ireg(TemperatureFAFAdd, Temp_F);
	MODBUS.onSetIreg(TemperatureFAFAdd, SetIreg);
	MODBUS.onGetIreg(TemperatureFAFAdd, GetIreg);

	MODBUS.Ireg(HumidityAdd, Hum);
	MODBUS.onSetIreg(HumidityAdd, SetIreg);
	MODBUS.onGetIreg(HumidityAdd, GetIreg);
	*/


	 
		 
} 
 
 
 
void floatconv(float input, uint16_t* ha, uint16_t* hb) { 
	long lconv = lround(input * scale); 
	memcpy((uint8_t*)ha, (uint8_t*)&lconv, 2); 
	memcpy((uint8_t*)hb, (uint8_t*)&lconv + 2, 2); 
 
} 
 
 
void Modbus_Poling(void) { 
 
	MODBUS.slave(SLAVE_ID);
	Configure_MODBUS_Registers();
	InitModbus();
	MODBUS.task(); 
	yield(); 
	led_ModbusFlash(10, 50); 
 
} 
 
 
 
uint16_t SetHoldreg(TRegister* reg, uint16_t val) {


	return val;
}


//Holding register 0 อ่านฟังก์ชันเรียกกลับเหตุการณ์ข้อมูล Address 0
uint16_t GetHoldreg(TRegister* reg, uint16_t val) {

	return val;
}


float SetIreg(TRegister* reg, float val) {


	return val;
}


//Holding register 0 อ่านฟังก์ชันเรียกกลับเหตุการณ์ข้อมูล Address 0
float GetIreg(TRegister* reg, float val) {

	return val;
}
 
 
 
/* 
//Convert character string to char arry 
char Stringconv(char *input, char strconv[30]) { 
 
 
		sscanf(input, "%s", strconv); 
 
		return *strconv; 
 
 
} 
 
 
//Convert Strings to char arry 
char StrtoCharconv(String str) { 
 
 
 
	// Length (with one extra character for the null terminator) 
	int str_len = str.length() + 1; 
 
	// Prepare the character array (the buffer) 
	char char_array[str_len]; 
 
	// Copy it over 
	str.toCharArray(char_array, str_len); 
 
	return (*char_array); 
 
 
 
} 
 
*/ 
 
 
 
