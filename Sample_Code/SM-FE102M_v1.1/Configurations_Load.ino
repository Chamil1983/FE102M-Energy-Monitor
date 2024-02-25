
void Load_Reg_CalConfiguration(void) {

    Wire.begin();

    VoltageSagThreshold = eic.GetRegisterValue(SagTh);  // Load Voltage Sag Threshold
    L_LineCalibrationGain = eic.GetRegisterValue(Lgain);  // Load L Line Calibration Gain
    L_LineCalibrationAngle = eic.GetRegisterValue(Lphi);  // Load L Line Calibration Angle
    N_LineCalibrationGain = eic.GetRegisterValue(Ngain);  // Load N Line Calibration Gain
    N_LineCalibrationAngle = eic.GetRegisterValue(Nphi);  // Load N Line Calibration Angle
    ActiveStartupPowerThreshold = eic.GetRegisterValue(PStartTh);  // Load Active Startup Power Threshold
    ActiveNoLoadPowerThreshold = eic.GetRegisterValue(PNolTh);  // Load Active No-Load Power Threshold
    ReactiveStartupPowerThreshold = eic.GetRegisterValue(QStartTh);  // Load Reactive Startup Power Threshold
    ReactiveNoLoadPowerThreshold = eic.GetRegisterValue(QNolTh);  // Load Reactive No-Load Power Threshold
    MeasurementCalibrationStart = eic.GetRegisterValue(AdjStart);  // Load Measurement Calibration Start Command
    VoltagermsGain = eic.GetRegisterValue(Ugain);  // Load Voltage rms Gain
    L_LineCurrentrmsGain = eic.GetRegisterValue(IgainL);  // Load L Line Current rms Gain
    N_LineCurrentrmsGain = eic.GetRegisterValue(IgainN);  // Load N Line Current rms Gain
    VoltageOffset = eic.GetRegisterValue(Uoffset);  // Load Voltage Offset
    L_LineCurrentOffset = eic.GetRegisterValue(IoffsetL);  // Load L Line Current Offset
    N_LineCurrentOffset = eic.GetRegisterValue(IoffsetN);  // Load N Line Current Offset
    L_LineActivePowerOffset = eic.GetRegisterValue(PoffsetL);  // Load L Line Active Power Offset
    L_LineReactivePowerOffset = eic.GetRegisterValue(QoffsetL);  // Load L Line Reactive Power Offset
    N_LineActivePowerOffset = eic.GetRegisterValue(PoffsetN);  // Load N Line Active Power Offset
    N_LineReactivePowerOffset = eic.GetRegisterValue(QoffsetN);  // Load N Line Reactive Power Offset
    Checksum1 = eic.GetRegisterValue(CSOne);  // Load Checksum 1
    Checksum2 = eic.GetRegisterValue(CSTwo);  // Load Checksum 2

    MeteringModeConfiguration = eic.GetRegisterValue(MMode);  // Load Metering Mode Configuration



}

void Load_Power_Registers(void) {

    Voltagerms = eic.GetLineVoltage();  // Load Voltage rms
    L_LineCurrentrms = eic.GetLineCurrent();  // Load L Line Current rms
    N_LineCurrentrms = eic.GetNLineCurrent();  // Load N Line Current rms
    VoltageFrequency = eic.GetFrequency();  // Load Voltage Frequency
    L_LinePowerFactor = eic.GetPowerFactor();  // Load L Line Power Factor
    PhaseAnglebetweenVoltageandL_LineCurrent = eic.GetPhaseAngle();  // Load Phase Angle between Voltage and L Line Curren
    N_LinePowerFactor = eic.GetNLinePowerFactor();  // Load N Line Power Factor
    PhaseAnglebetweenVoltageandN_LineCurrent = eic.GetNLinePhaseAngle();  // Load Phase Angle between Voltage and N Line Current
    L_LineMeanActivePower = eic.GetActivePower();  // Load L Line Mean Active Power
    L_LineMeanReactivePower = eic.GetReactivePower();  // EEPROM L Line Mean Reactive Power
    L_LineMeanApparentPower = eic.GetApparentPower();  // Load L Line Mean Apparent Power
    N_LineMeanActivePower = eic.GetNLineActivePower();  // Load N Line Mean Active Power
    N_LineMeanReactivePower = eic.GetNLineReactivePower();  // Load N Line Mean Reactive Power
    N_LineMeanApparentPower = eic.GetNLineApparentPower();  // Load N Line Mean Apparent Power



    ForwardActiveEnergy = eic.GetImportEnergy();  // Load Forward Active Energy
    ReverseActiveEnergy = eic.GetExportEnergy();  // Load Reverse Active Energy
    AbsoluteActiveEnergy = eic.GetAbsActiveEnergy();  // Load Absolute Active Energy
    Forward_InductiveReactiveEnergy = eic.GetReactivefwdEnergy();  // Load Forward (Inductive) Reactive Energy
    Reverse_CapacitiveReactiveEnergy = eic.GetReactivervsEnergy();  // Load Reverse (Capacitive) Reactive Energy
    AbsoluteReactiveEnergy = eic.GetAbsReactiveEnergy();  // Load Absolute Reactive Energy
    MeteringStatus = eic.GetMeterStatus();  // Load Metering Status

   


}


unsigned long startMillis;
unsigned long currentMillis;


// -------------------------------------------------------------------
// SETUP
// -------------------------------------------------------------------
void energy_meter_setup() {

    /*Get values from web interface and assign them if populated*/
    //if (voltage_cal.toInt() > 0) voltageread = voltage_cal.toInt();


    /*Initialise the ATM90E32 & Pass CS pin and calibrations to its library -
      the 2nd (B) current channel is not used with the split phase meter */
    Serial.println("Start ATM90E26");
    eic.InitEnergyIC();
    delay(1000);


    startMillis = millis();  //initial start time

} // end setup




void Read_DHT_Data(void) {

    // Reading temperature or humidity takes about 250 milliseconds!
// Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
    Hum = dht.readHumidity();
    // Read temperature as Celsius (the default)
    Temp = dht.readTemperature();
    // Read temperature as Fahrenheit (isFahrenheit = true)
    Temp_F = dht.readTemperature(true);

    // Check if any reads failed and exit early (to try again).
    if (isnan(Hum) || isnan(Temp) || isnan(Temp_F)) {
        Serial.println(F("Failed to read from DHT sensor!"));
        return;
    }

    // Compute heat index in Fahrenheit (the default)
    hif = dht.computeHeatIndex(Temp_F, Hum);
    // Compute heat index in Celsius (isFahreHumeit = false)
    hic = dht.computeHeatIndex(Temp, Hum, false);

    Serial.println("Humidity: "+ String(Hum,2) +"%");
    Serial.print("Temperature: "+ String(Temp,2));
    Serial.print("� C");
    Serial.print("\t " + String(Temp_F, 2));
    Serial.println("� F");
    Serial.print("Heat index: "+ String(hic,2));
    Serial.print("� C");
    Serial.print("\t" + String(hif, 2));
    Serial.println("� F");
    Serial.println("\n\n");


    

}

void SetupTimer1(void) {

    
    Timer1_Cfg = timerBegin(0, 80, true);
    timerAttachInterrupt(Timer1_Cfg, &Timer1_ISR, true);
    timerAlarmWrite(Timer1_Cfg, 1000000, true);
    timerAlarmEnable(Timer1_Cfg);

}





/*
 * 
 * uint16_t Read_EnergyDataUnsigned(unsigned char address) {

    unsigned short val = eic.CommEnergyIC(1, address, 0xFFFF);


    return val;



}
 * 
 * 
 * int16_t Read_EnergyDataSigned(unsigned char address) {

    unsigned short val = eic.CommEnergyIC(1, address, 0xFFFF);


    return val;



}
 * 
 */


/*
int16_t Get_TempHumData() {

    float Hum1 = dht.readHumidity();
    // Read temperature as Celsius (the default)
    float Temp1 = dht.readTemperature();
    // Read temperature as Fahrenheit (isFahrenheit = true)
    float Temp_F1 = dht.readTemperature(true);

    // Check if any reads failed and exit early (to try again).
    if (isnan(Hum) || isnan(Temp) || isnan(Temp_F)) {
        Serial.println(F("Failed to read from DHT sensor!"));
        return;
    }

    // Compute heat index in Fahrenheit (the default)
    float hif1 = dht.computeHeatIndex(Temp_F, Hum);
    // Compute heat index in Celsius (isFahreHumeit = false)
    float hic1 = dht.computeHeatIndex(Temp, Hum, false);


}

*/
