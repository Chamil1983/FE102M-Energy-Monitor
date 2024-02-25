float voltage, current;
float Power,Imp_Energy, Exp_Energy;
float PFactor, temp, Frequency;
char measurement[16];


void ReadPower_Data(void) {

	voltage = eic.GetLineVoltage();
	current = eic.GetLineCurrent();
	Frequency = eic.GetFrequency();
	PFactor = eic.GetPowerFactor();
	Power = eic.GetActivePower();
	Imp_Energy = eic.GetImportEnergy();
	Exp_Energy = eic.GetExportEnergy();

	Serial.println();

	Serial.println("-----------");

	yield();
	Serial.print("Line Voltage \t\t\t(Urms 0x49):\t\t");
	Serial.print(voltage);
	Serial.println(" V");


	yield();
	Serial.print("Line Current \t\t\t(Irms 0x48):\t\t");
	Serial.print(eic.GetLineCurrent());
	Serial.println(" A");


	yield();
	Serial.print("Line Frequency \t\t\t(Freq 0x4C):\t\t");
	Serial.print(eic.GetFrequency());
	Serial.println(" Hz");


	yield();
	Serial.print("Power Factor \t\t\t(PowerF 0x4D):\t\t");
	Serial.println(eic.GetPowerFactor());


	yield();
	Serial.print("Active power \t\t\t(Pmean 0x4A):\t\t");
	Serial.print(eic.GetActivePower());
	Serial.println(" W");

	yield();
	Serial.print("Import Energy \t\t\t(APenergy 0x40):\t");
	Serial.print(eic.GetImportEnergy());
	Serial.println(" kWh");

	yield();
	Serial.print("Export Energy \t\t\t(ANenergy 0x41):\t");
	Serial.print(eic.GetExportEnergy());
	Serial.println(" kWh");

	Serial.println("-----------");

	yield();
	Serial.print("Abs Active Energy \t\t(ATenergy 0x42):\t");
	Serial.print(eic.GetAbsActiveEnergy());
	Serial.println(" kWh");

	yield();
	Serial.print("Abs Reactive Energy \t\t(Rtenergy 0x45):\t");
	Serial.print(eic.GetAbsReactiveEnergy());
	Serial.println(" kVarh");

	yield();
	Serial.print("Abs Reactive Forward Energy \t(RPenergy 0x43):\t");
	Serial.print(eic.GetReactivefwdEnergy());
	Serial.println(" KVarh");

	lcd.clear();
	dtostrf(voltage, 6, 2, measurement);
	lcd.setCursor(0, 0);
	lcd.print("V_Line : " + String(measurement) + "V");

	dtostrf(current, 6, 2, measurement);
	lcd.setCursor(0, 1);
	lcd.print("I_Line : " + String(measurement) + "A");

	dtostrf(Frequency, 6, 2, measurement);
	lcd.setCursor(0, 2);
	lcd.print("Freq : " + String(measurement) + "Hz");

	dtostrf(PFactor, 5, 2, measurement);
	lcd.setCursor(0, 3);
	lcd.print("Pfac : " + String(measurement));


	delay(2000);
	lcd.clear();

	dtostrf(Power, 7, 2, measurement);
	lcd.setCursor(0, 0);
	lcd.print("P : " + String(measurement)+"W");

	dtostrf(Imp_Energy, 7, 2, measurement);
	lcd.setCursor(0, 1);
	lcd.print("E-Imp : " + String(measurement) + "kW");

	dtostrf(Exp_Energy, 7, 2, measurement);
	lcd.setCursor(0, 2);
	lcd.print("E-Exp : " + String(measurement) + "kW");


}

//======================================================================
String ReadStringCmd_FLASH(uint8_t* FlashPointer, uint8_t Lenght, boolean PrintCR, boolean NoPrint) {
	uint8_t k;
	char    myChar;
	String  TempString;

	for (k = 0; k < Lenght; k++) {
		myChar = pgm_read_byte_near(FlashPointer + k);
		if (NoPrint == FALSE) {
			Serial.print(myChar);
		}
		TempString += myChar;
	}
	if (NoPrint == FALSE) {
		if (PrintCR == TRUE) {
			Serial.print("\n");
		}
	}
	return(TempString);
}
//======================================================================