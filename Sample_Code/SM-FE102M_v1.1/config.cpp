/*
   -------------------------------------------------------------------
   EmonESP Serial to Emoncms gateway
   -------------------------------------------------------------------
   Adaptation of Chris Howells OpenEVSE ESP Wifi
   by Trystan Lea, Glyn Hudson, OpenEnergyMonitor

   Modified to use with the CircuitSetup.us Split Phase Energy Meter by jdeglavina

   All adaptation GNU General Public License as below.

   -------------------------------------------------------------------

   This file is part of OpenEnergyMonitor.org project.
   EmonESP is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 3, or (at your option)
   any later version.
   EmonESP is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with EmonESP; see the file COPYING.  If not, write to the
   Free Software Foundation, Inc., 59 Temple Place - Suite 330,
   Boston, MA 02111-1307, USA.
*/

#include "emonesp.h"
//#include "energy_meter.h"
#include "config.h"
#include <SparkFun_External_EEPROM.h>


ExternalEEPROM ExtMem;

#define EEPROM_ADDRESS 0x50    //Address of 24LC256 eeprom chip


bool eeprom_autoupdate = true; //specify if you wanna control updates
const String terminating_char = "|"; //ensure its something you wont use in any String!!!!!!

// Init and get the time

static int getNodeId()
{
    unsigned long chip_id = ESP.getEfuseMac();
    DBUGVAR(chip_id);
    int chip_tmp = chip_id / 10000;
    chip_tmp = chip_tmp * 10000;
    DBUGVAR(chip_tmp);
    return (chip_id - chip_tmp);
}

const String node_type = NODE_TYPE;
const String node_description = NODE_DESCRIPTION;

uint16_t node_id = getNodeId();
char id[41];

String node_name_default = node_type + String(node_id);
String node_name = "SM-FE102M(WiFi)";
String node_describe = "";


//===================================================================
String Vendor_Name = "Microcode Eng";
String Product_Code = "FE102M";
String Revision_Number = "1.1";

String meter_name = "SM Energy";
String model = "SM-FE102M";
String manufacturer = "Microcode Engineering";
String serial_no = "";
String hardware_rev = "1.1";
String firmware_ver = "SM-FE102M 1.1V";
String mac_add = getMacAddress();
String node = node_name;
uint16_t node_ID = node_id;

//===================================================================



// Wifi Network Strings
String esid = "";
String epass = "";

const char* ssid = "Google NODE"; // Change this to your WiFi SSID
const char* password = "36002016"; // Change this to your WiFi password

//const char* ssid = "SM-FE102M(WiFi)"; // Change this to your WiFi SSID
//const char* password = ""; // Change this to your WiFi password
 

//IPAddress staticIP_1(192, 168, 86, 10);
//IPAddress gateway_1(192, 168, 86, 1);
//IPAddress subnet_1(255, 255, 255, 0);

const char* MQTT_server = "192.168.102.154";
int mqtt_port = 1883;


// Web server authentication (leave blank for none)
String www_username = "admin";
String www_password = "admin";

// EMONCMS SERVER strings
String emoncms_server = "";
String emoncms_path = "";
String emoncms_node = "";
String emoncms_apikey = "";
String emoncms_fingerprint = "";

// MQTT Settings
String mqtt_server = "";
String mqtt_topic = "";
String mqtt_user = "";
String mqtt_pass = "";
String mqtt_feed_prefix = "";

// Calibration Settings
String voltage_cal = "";
String ct1_cal = "";
String ct2_cal = "";
String freq_cal = "";
String gain_cal = "";

#ifdef SOLAR_METER
String svoltage_cal = "";
String sct1_cal = "";
String sct2_cal = "";
#endif


//================================================================================
        // Metering and Measurement CALIBRATION VARIABLE
//================================================================================
unsigned short     VoltageSagThreshold;                //  (u16) Voltage Sag Threshold
unsigned short     L_LineCalibrationGain;              //  (u16) L Line Calibration Gain
unsigned short     L_LineCalibrationAngle;             //  (u16) L Line Calibration Angle
unsigned short     N_LineCalibrationGain;              //  (u16) N Line Calibration Gain
unsigned short     N_LineCalibrationAngle;             //  (u16) N Line Calibration Angle
unsigned short     ActiveStartupPowerThreshold;        //  (u16) Active Startup Power Threshold
unsigned short     ActiveNoLoadPowerThreshold;         //  (u16) Active No-Load Power Threshold
unsigned short     ReactiveStartupPowerThreshold;      //  (u16) Reactive Startup Power Threshold
unsigned short     ReactiveNoLoadPowerThreshold;       //  (u16) Reactive No-Load Power Threshold
unsigned short     MeasurementCalibrationStart;        //  (u16) Measurement Calibration Start Command
unsigned short     VoltagermsGain;                     //  (u16) Voltage rms Gain
unsigned short     L_LineCurrentrmsGain;				//  (u16) L Line Current rms Gain
unsigned short     N_LineCurrentrmsGain;				//  (u16) N Line Current rms Gain
unsigned short     VoltageOffset;						//  (u16) Voltage Offset
unsigned short     L_LineCurrentOffset;				//  (u16) L Line Current Offset
unsigned short     N_LineCurrentOffset;                //  (u16) N Line Current Offset
unsigned short     L_LineActivePowerOffset;            //  (u16) L Line Active Power Offset
unsigned short     L_LineReactivePowerOffset;          //  (u16) L Line Reactive Power Offset
unsigned short     N_LineActivePowerOffset;			//  (u16) N Line Active Power Offset
unsigned short     N_LineReactivePowerOffset;			//  (u16) N Line Reactive Power Offset
unsigned short     Checksum1;                          //  (u16) Checksum 1
unsigned short     Checksum2;                          //  (u16) Checksum 2

unsigned short voltageread;



//================================================================================


    //================================================================================
        // Metering and Measurement VARIABLE
//================================================================================
float    ForwardActiveEnergy;          //  (u16) Forward Active Energy
float    ReverseActiveEnergy;          //  (u16) Reverse Active Energy
float    AbsoluteActiveEnergy;          //  (u16) Absolute Active Energy
float    Forward_InductiveReactiveEnergy;          //  (u16) Forward (Inductive) Reactive Energy
float    Reverse_CapacitiveReactiveEnergy;          //  (u16) Reverse (Capacitive) Reactive Energy
float    AbsoluteReactiveEnergy;          //  (u16) Absolute Reactive Energy
unsigned short      MeteringStatus;                     //  (u16) Metering Status
float    L_LineCurrentrms;          //  (u16) L Line Current rms
float    Voltagerms;          //  (u16) Voltage rms
float    L_LineMeanActivePower;          //  (u16) L Line Mean Active Power
float    L_LineMeanReactivePower;          //  (u16) L Line Mean Reactive Power
float    VoltageFrequency;          //  (u16) Voltage Frequency
float    L_LinePowerFactor;          //  (u16) L Line Power Factor
float    PhaseAnglebetweenVoltageandL_LineCurrent;          //  (u16) Phase Angle between Voltage and L Line Current
float    L_LineMeanApparentPower;          //  (u16) L Line Mean Apparent Power
float    N_LineCurrentrms;          //  (u16) N Line Current rms
float    N_LineMeanActivePower;          //  (u16) N Line Mean Active Power
float    N_LineMeanReactivePower;          //  (u16) N Line Mean Reactive Power
float    N_LinePowerFactor;          //  (u16) N Line Power Factor
float    PhaseAnglebetweenVoltageandN_LineCurrent;          //  (u16) Phase Angle between Voltage and N Line Current
float    N_LineMeanApparentPower;          //  (u16) N Line Mean Apparent Power



//================================================================================
float Hum, Temp, Temp_F, hif, hic;


//================================================================================
//  DESIGN CONFIGURATION VARIABLES
//SystemConfigurationFlag SystemConfiguration;    //  (b32)
//EventConfigurationFlag  EventConfiguration;     //  (b32)

unsigned short      MeteringModeConfiguration;          //  (u16) Metering Mode Configuration



//================================================================================



//================================================================================
        // Metering and Measurement VARIABLE
//================================================================================
float    EEPROM_ForwardActiveEnergy;          //  EEPROM Read Value - Forward Active Energy
float    EEPROM_ReverseActiveEnergy;          //  EEPROM Read Value - Reverse Active Energy
float    EEPROM_AbsoluteActiveEnergy;          //  EEPROM Read Value - Absolute Active Energy
float    EEPROM_Forward_InductiveReactiveEnergy;          //  EEPROM Read Value -  (Inductive) Reactive Energy
float    EEPROM_Reverse_CapacitiveReactiveEnergy;          //  EEPROM Read Value -  (Capacitive) Reactive Energy
float    EEPROM_AbsoluteReactiveEnergy;          //  EEPROM Read Value -  Reactive Energy
unsigned short    EEPROM_MeteringStatus;                     //  EEPROM Read Value - Metering Status
float    EEPROM_L_LineCurrentrms;          //  EEPROM Read Value - L Line Current rms
float    EEPROM_Voltagerms;          //  EEPROM Read Value -  rms
float    EEPROM_L_LineMeanActivePower;          //  EEPROM Read Value -  Line Mean Active Power
float    EEPROM_L_LineMeanReactivePower;          //  EEPROM Read Value -  Line Mean Reactive Power
float    EEPROM_VoltageFrequency;          //  EEPROM Read Value -  Frequency
float    EEPROM_L_LinePowerFactor;          //  EEPROM Read Value -  Line Power Factor
float    EEPROM_PhaseAnglebetweenVoltageandL_LineCurrent;          //  EEPROM Read Value -  Angle between Voltage and L Line Current
float    EEPROM_L_LineMeanApparentPower;          //  EEPROM Read Value - L Line Mean Apparent Power
float    EEPROM_N_LineCurrentrms;          //  EEPROM Read Value -  Line Current rms
float    EEPROM_N_LineMeanActivePower;          //  EEPROM Read Value -  Line Mean Active Power
float    EEPROM_N_LineMeanReactivePower;          //  EEPROM Read Value -  Line Mean Reactive Power
float    EEPROM_N_LinePowerFactor;          //  EEPROM Read Value - N Line Power Factor
float    EEPROM_PhaseAnglebetweenVoltageandN_LineCurrent;          //  EEPROM Read Value - Phase Angle between Voltage and N Line Current
float    EEPROM_N_LineMeanApparentPower;          //  EEPROM Read Value - N Line Mean Apparent Power

float    EEPROM_Temperature_Val;          //  EEPROM Read Value - Temparature Value °C
float    EEPROM_TemperatureF_Val;          //  EEPROM Read Value - Temparature Value ° F
float    EEPROM_Humidity_Val;          //  EEPROM Read Value - Humidity Value %
float    EEPROM_HeatIndex_Temp_Val;          //  EEPROM Read Value - Temparature Value °C
float    EEPROM_HeatIndex_TempF_Val;          //  EEPROM Read Value - Temparature Value °F

//================================================================================

char EEPROM_Vendor_Name[30];
char EEPROM_Product_Code[15];
char EEPROM_Revision_Number[8];

char EEPROM_Meter_name[20];
char EEPROM_Model[20];
char EEPROM_Manufacturer[30];
char EEPROM_Serial_no[30];
char EEPROM_Hardware_rev[8];
char EEPROM_Firmware_ver[20];

char EEPROM_MAC_add[30];
char EEPROM_node[20];
uint16_t EEPROM_node_ID;

//=========================================================================================================================
/*
 ___________________________________________________________________________________________
|                 |                                                                         |
|                 |                                                                         |
|   EEPROM ADD    | DESCRIPTION                                                             |
|-------------------------------------------------------------------------------------------|
|                                         PAGE 0                                            |
|-------------------------------------------------------------------------------------------|
|     0x0000      | (u16) -> WiFi SSID                                                      |
|     0x0002      | (u16) -> WiFi Password                                                  |
|     0x0004      | (u16) -> Gain Active Power                                              |
|     0x0006      | (u16) -> Gain Reactive Power                                            |
|     0x0008      | (u16) -> Offset Current RMS                                             |
|     0x000C      | (u16) -> Offset Active Power                                            |
|-------------------------------------------------------------------------------------------|
|                                         PAGE 1                                            |
|-------------------------------------------------------------------------------------------|
|     0x0010      | (s32) -> Offset Reactive Power                                          |
|     0x0014      | (s16) -> DC Offset Current                                              |
|     0x0016      | (s16) -> Phase Compensation                                             |
|     0x0018      | (u16) -> Apparent Power Divisor                                         |
|     0x001A      | (b32) -> System Configuration                                           |
|     0x001E      | (u16) -> Free (Not Used)                                                |
|-------------------------------------------------------------------------------------------|
|                                         PAGE 2                                            |
|-------------------------------------------------------------------------------------------|
|     0x0020      | (b32) -> Event Configuration                                            |
|     0x0024      | (b32) -> Range                                                          |
|     0x0028      | (u32) -> Calibration Current                                            |
|     0x002C      | (u16) -> Calibration Voltage                                            |
|     0x002E      | (u16) -> Free (Not Used)                                                |
|-------------------------------------------------------------------------------------------|
|                                         PAGE 3                                            |
|-------------------------------------------------------------------------------------------|
|     0x0030      | (u32) -> Calibration Active Power                                       |
|     0x0034      | (u32) -> Calibration Reactive Power                                     |
|     0x0038      | (u16) -> Line Frequency Reference                                       |
|     0x003A      | (u16) -> Accumulator Interval Parameter                                 |
|     0x003C      | (u16) -> Voltage Sag Limit                                              |
|     0x003E      | (u16) -> Voltage Surge Limit                                            |
|-------------------------------------------------------------------------------------------|
|                                         PAGE 4                                            |
|-------------------------------------------------------------------------------------------|
|     0x0040      | (u32) -> OverCurrent Limit                                              |
|     0x0044      | (u32) -> OverPower Limit                                                |
|     0x0048      | (u16) -> Temperature Compensation for Frequency                         |
|     0x004A      | (u16) -> Temperature Compensation for Current                           |
|     0x004C      | (u16) -> Temperature Compensation for Power                             |
|     0x004E      | (u16) -> Ambient Temperature Reference Voltage                          |
|-------------------------------------------------------------------------------------------|
|                                         PAGE 5                                            |
|-------------------------------------------------------------------------------------------|
|     0x0050      | (u16) -> PWM Period                                                     |
|     0x0052      | (u16) -> PWM Duty Cycle                                                 |
|     0x0054      | (u16) -> MinMax Pointer 1                                               |
|     0x0056      | (u16) -> MinMax Pointer 2                                               |
|     0x0058      | (u16) -> OverTemperature Limit                                          |
|     0x005A      | (u16) -> Energy Control                                                 |
|     0x005C      | (u16) -> PWM Control                                                    |
|     0x005E      | (u16) -> No Load Threshold                                              |
|-------------------------------------------------------------------------------------------|
|                                   PAGE 6 - Page 31                                        |
|-------------------------------------------------------------------------------------------|
| 0x0060 - 0x01FF | Reserved (Free Space)                                                   |
|_________________|_________________________________________________________________________|
*/
//=========================================================================================================================





/*
// -------------------------------------------------------------------
// Reset EEPROM, wipes all settings
// -------------------------------------------------------------------
void ResetEEPROM() {
  EEPROM.begin(EEPROM_SIZE);

  //DBUGS.println("Erasing EEPROM");
  for (int i = 0; i < EEPROM_SIZE; ++i) {
    EEPROM.write(i, 0xff);
    //DBUGS.print("#");
  }
  EEPROM.end();
}

void EEPROM_read_string(int start, int count, String & val, String defaultVal = "") {
  byte checksum = CHECKSUM_SEED;
  for (int i = 0; i < count - 1; ++i) {
    byte c = EEPROM.read(start + i);
    if (c != 0 && c != 255) {
      checksum ^= c;
      val += (char) c;
    } else {
      break;
    }
  }

  // Check the checksum
  byte c = EEPROM.read(start + (count - 1));
  DBUGF("Got '%s' %d == %d @ %d:%d", val.c_str(), c, checksum, start, count);
  if (c != checksum) {
    DBUGF("Using default '%s'", defaultVal.c_str());
    val = defaultVal;
  }
}

void EEPROM_write_string(int start, int count, String val) {
  byte checksum = CHECKSUM_SEED;
  for (int i = 0; i < count - 1; ++i) {
    if (i < val.length()) {
      checksum ^= val[i];
      EEPROM.write(start + i, val[i]);
    } else {
      EEPROM.write(start + i, 0);
    }
  }
  EEPROM.write(start + (count - 1), checksum);
  DBUGF("Saved '%s' %d @ %d:%d", val.c_str(), checksum, start, count);
}


*/


// -------------------------------------------------------------------
// Load saved settings from EEPROM
// -------------------------------------------------------------------
void config_load_settings()
{
  //EEPROM.begin(EEPROM_SIZE);

    Wire.begin();

  // Load WiFi values

    ExtMem.get(EEPROM_ESID_START, esid); //location to read, thing to put data into
    ExtMem.get(EEPROM_EPASS_START, epass); //location to read, thing to put data into

  if (ExtMem.begin(EEPROM_ADDRESS, Wire) == false) //And Uno will fail to compile here
  {
      DBUGS.println("No memory detected. Freezing.");
      while (true);
  }

  Load_CalConfiguration();
  
      // EmonCMS settings
  ExtMem.get(EEPROM_EMON_API_KEY_START, emoncms_apikey); //location to read, thing to put data into
  ExtMem.get(EEPROM_EMON_SERVER_START, emoncms_server);
  ExtMem.get(EEPROM_EMON_PATH_START, emoncms_path);
  ExtMem.get(EEPROM_EMON_NODE_START,emoncms_node);
  ExtMem.get(EEPROM_EMON_FINGERPRINT_START,emoncms_fingerprint);

  // MQTT settings
  ExtMem.get(EEPROM_MQTT_SERVER_START, mqtt_server);
  ExtMem.get(EEPROM_MQTT_TOPIC_START, mqtt_topic);
  ExtMem.get(EEPROM_MQTT_FEED_PREFIX_START, mqtt_feed_prefix);
  ExtMem.get(EEPROM_MQTT_USER_START, mqtt_user);
  ExtMem.get(EEPROM_MQTT_PASS_START, mqtt_pass);


  //N_LineReactivePowerOffset = voltage_cal
  ExtMem.get(EEPROM_CAL_VOLTAGE_START, voltage_cal);

  // Calibration settings
  //ExtMem.get(EEPROM_CAL_VOLTAGE_START, voltage_cal);
  ExtMem.get(EEPROM_CAL_CT1_START, ct1_cal);
  ExtMem.get(EEPROM_CAL_CT2_START, ct2_cal);
  ExtMem.get(EEPROM_CAL_FREQ_START,  freq_cal);
  ExtMem.get(EEPROM_CAL_GAIN_START, gain_cal);

#ifdef SOLAR_METER
  ExtMem.get((EEPROM_CAL_SVOLTAGE_START,  svoltage_cal);
  ExtMem.get((EEPROM_CAL_SCT1_START,  sct1_cal);
  ExtMem.get((EEPROM_CAL_SCT2_START,  sct2_cal);
#endif

  // Web server credentials
  ExtMem.get(EEPROM_WWW_USER_START,  www_username);
  ExtMem.get(EEPROM_WWW_PASS_START,  www_password);
  

  
    
}








//for CircuitSetup energy meter
#ifdef SOLAR_METER
void config_save_cal(String voltage, String ct1, String ct2, String freq, String gain, String svoltage, String sct1, String sct2)
{
    Wire.begin();

  voltage_cal = voltage;
  ct1_cal = ct1;
  ct2_cal = ct2;
  freq_cal = freq;
  gain_cal = gain;
  svoltage_cal = svoltage;
  sct1_cal = sct1;
  sct2_cal = sct2;

  EEPROM_Write_String(EEPROM_CAL_VOLTAGE_START, EEPROM_CAL_VOLTAGE_SIZE, voltage_cal);
  EEPROM_Write_String(EEPROM_CAL_CT1_START, EEPROM_CAL_CT1_SIZE, ct1_cal);
  EEPROM_write_string(EEPROM_CAL_CT2_START, EEPROM_CAL_CT2_SIZE, ct2_cal);
  EEPROM_Write_String(EEPROM_CAL_FREQ_START, EEPROM_CAL_FREQ_SIZE, freq_cal);
  EEPROM_Write_String(EEPROM_CAL_GAIN_START, EEPROM_CAL_GAIN_SIZE, gain_cal);
  EEPROM_Write_String(EEPROM_CAL_SVOLTAGE_START, EEPROM_CAL_SVOLTAGE_SIZE, svoltage_cal);
  EEPROM_Write_String(EEPROM_CAL_SCT1_START, EEPROM_CAL_SCT1_SIZE, sct1_cal);
  EEPROM_Write_String(EEPROM_CAL_SCT2_START, EEPROM_CAL_SCT2_SIZE, sct2_cal);


}
#else
void config_save_cal(String voltage, String ct1, String ct2, String freq, String gain)
{
    Wire.begin();

  voltage_cal = voltage;
  ct1_cal = ct1;
  ct2_cal = ct2;
  freq_cal = freq;
  gain_cal = gain;

  EEPROM_Write_String(EEPROM_CAL_VOLTAGE_START,  voltage_cal);
  EEPROM_Write_String(EEPROM_CAL_CT1_START,  ct1_cal);
  EEPROM_Write_String(EEPROM_CAL_CT2_START,  ct2_cal);
  EEPROM_Write_String(EEPROM_CAL_FREQ_START,  freq_cal);
  EEPROM_Write_String(EEPROM_CAL_GAIN_START,  gain_cal);




}
#endif


void Save_CalConfiguration(void) {

    Wire.begin();


    if (ExtMem.begin(EEPROM_ADDRESS, Wire) == false) //And Uno will fail to compile here
    {
        DBUGS.println("No memory detected. Freezing.");
        while (true);
    }

    ExtMem.put(EEPROM_VOLTSAGTH_START, VoltageSagThreshold);
    ExtMem.put(EEPROM_L_Line_CAL_GAIN_START, L_LineCalibrationGain);
    ExtMem.put(EEPROM_L_Line_CAL_ANG_START, L_LineCalibrationAngle);
    ExtMem.put(EEPROM_N_Line_CAL_GAIN_START, N_LineCalibrationGain);
    ExtMem.put(EEPROM_N_Line_CAL_ANG_START, N_LineCalibrationAngle);
    ExtMem.put(EEPROM_ACT_STUP_PWRTH_START, ActiveStartupPowerThreshold);
    ExtMem.put(EEPROM_ACT_NOLOAD_PWRTH_START, ActiveNoLoadPowerThreshold);
    ExtMem.put(EEPROM_REACT_STUP_PWRTH_START, ReactiveStartupPowerThreshold);
    ExtMem.put(EEPROM_REACT_NOLOAD_PWRTH_START, ReactiveNoLoadPowerThreshold);
    ExtMem.put(EEPROM_MESUR_CAL_START, MeasurementCalibrationStart);
    ExtMem.put(EEPROM_VOL_GAIN_START, VoltagermsGain);
    ExtMem.put(EEPROM_L_LINE_CUR_RMSGAIN_START, L_LineCurrentrmsGain);
    ExtMem.put(EEPROM_N_LINE_CUR_RMSGAIN_START, N_LineCurrentrmsGain);
    ExtMem.put(EEPROM_VOLT_OFFSET_START, VoltageOffset);
    ExtMem.put(EEPROM_L_LINE_CUR_OFFSET_START, L_LineCurrentOffset);
    ExtMem.put(EEPROM_N_LINE_CUR_OFFSET_START, N_LineCurrentOffset);
    ExtMem.put(EEPROM_L_LINE_ACT_PWR_OFFSET_START, L_LineActivePowerOffset);
    ExtMem.put(EEPROM_L_LINE_REACT_PWR_OFFSET_START, L_LineReactivePowerOffset);
    ExtMem.put(EEPROM_N_LINE_ACT_PWR_OFFSET_START, N_LineActivePowerOffset);
    ExtMem.put(EEPROM_N_LINE_REACT_PWR_OFFSET_START, N_LineReactivePowerOffset);
    ExtMem.put(EEPROM_CHECKSUM1_START, Checksum1);
    ExtMem.put(EEPROM_CHECKSUM2_START, Checksum2);
    ExtMem.put(EEPROM_METER_MODE_CONFIG_START, MeteringModeConfiguration);



}


void Load_CalConfiguration(void) {


    ExtMem.get(EEPROM_VOLTSAGTH_START, VoltageSagThreshold);
    ExtMem.get(EEPROM_L_Line_CAL_GAIN_START, L_LineCalibrationGain);
    ExtMem.get(EEPROM_L_Line_CAL_ANG_START, L_LineCalibrationAngle);
    ExtMem.get(EEPROM_N_Line_CAL_GAIN_START, N_LineCalibrationGain);
    ExtMem.get(EEPROM_N_Line_CAL_ANG_START, N_LineCalibrationAngle);
    ExtMem.get(EEPROM_ACT_STUP_PWRTH_START, ActiveStartupPowerThreshold);
    ExtMem.get(EEPROM_ACT_NOLOAD_PWRTH_START, ActiveNoLoadPowerThreshold);
    ExtMem.get(EEPROM_REACT_STUP_PWRTH_START, ReactiveStartupPowerThreshold);
    ExtMem.get(EEPROM_REACT_NOLOAD_PWRTH_START, ReactiveNoLoadPowerThreshold);
    ExtMem.get(EEPROM_MESUR_CAL_START, MeasurementCalibrationStart);
    ExtMem.get(EEPROM_VOL_GAIN_START, VoltagermsGain);
    ExtMem.get(EEPROM_L_LINE_CUR_RMSGAIN_START, L_LineCurrentrmsGain);
    ExtMem.get(EEPROM_N_LINE_CUR_RMSGAIN_START, N_LineCurrentrmsGain);
    ExtMem.get(EEPROM_VOLT_OFFSET_START, VoltageOffset);
    ExtMem.get(EEPROM_L_LINE_CUR_OFFSET_START, L_LineCurrentOffset);
    ExtMem.get(EEPROM_N_LINE_CUR_OFFSET_START, N_LineCurrentOffset);
    ExtMem.get(EEPROM_L_LINE_ACT_PWR_OFFSET_START, L_LineActivePowerOffset);
    ExtMem.get(EEPROM_L_LINE_REACT_PWR_OFFSET_START, L_LineReactivePowerOffset);
    ExtMem.get(EEPROM_N_LINE_ACT_PWR_OFFSET_START, N_LineActivePowerOffset);
    ExtMem.get(EEPROM_N_LINE_REACT_PWR_OFFSET_START, N_LineReactivePowerOffset);
    ExtMem.get(EEPROM_CHECKSUM1_START, Checksum1);
    ExtMem.get(EEPROM_CHECKSUM2_START, Checksum2);
    ExtMem.get(EEPROM_METER_MODE_CONFIG_START, MeteringModeConfiguration);


}



void EEPROM_Update_ManufactureInfo(void) {

    Wire.begin();


    if (ExtMem.begin(EEPROM_ADDRESS, Wire) == false) //And Uno will fail to compile here
    {
        DBUGS.println("No memory detected. Freezing.");
        while (true);
    }

    ExtMem.put(EEPROM_VENDOR_NAME_START, Vendor_Name);
    ExtMem.put(EEPROM_PRODUCT_CODEE_START, Product_Code);
    ExtMem.put(EEPROM_REVISION_NUMBER_START, Revision_Number);
    ExtMem.put(EEPROM_METER_NAME_START, meter_name);
    ExtMem.put(EEPROM_MODEL_START, model);
    ExtMem.put(EEPROM_MANUFACTURER_START, manufacturer);
    ExtMem.put(EEPROM_SERIAL_NO_START, serial_no);
    ExtMem.put(EEPROM_HARDWARE_REV_START, hardware_rev);
    ExtMem.put(EEPROM_FIRMWARE_VER_START, firmware_ver);
    ExtMem.put(EEPROM_MAC_ADD_START, mac_add);
    ExtMem.put(EEPROM_NODE_START, node);
    ExtMem.put(EEPROM_NODE_ID_START, node_ID);


}


void EEPROM_READ_ManufactureInfo(void) {

    ExtMem.get(EEPROM_VENDOR_NAME_START, EEPROM_Vendor_Name);
    ExtMem.get(EEPROM_PRODUCT_CODEE_START, EEPROM_Product_Code);
    ExtMem.get(EEPROM_REVISION_NUMBER_START, EEPROM_Revision_Number);
    ExtMem.get(EEPROM_METER_NAME_START, EEPROM_Meter_name);
    ExtMem.get(EEPROM_MODEL_START, EEPROM_Model);
    ExtMem.get(EEPROM_MANUFACTURER_START, EEPROM_Manufacturer);
    ExtMem.get(EEPROM_SERIAL_NO_START, EEPROM_Serial_no);
    ExtMem.get(EEPROM_HARDWARE_REV_START, EEPROM_Hardware_rev);
    ExtMem.get(EEPROM_FIRMWARE_VER_START, EEPROM_Firmware_ver);
    ExtMem.get(EEPROM_MAC_ADD_START, EEPROM_MAC_add);
    ExtMem.get(EEPROM_NODE_START, EEPROM_node);
    ExtMem.get(EEPROM_NODE_ID_START, EEPROM_node_ID);



}


void EEPROM_Update_Power_Registers(void) {

    ExtMem.put(EEPROM_FWD_ACT_ENERGY_START, ForwardActiveEnergy);
    ExtMem.put(EEPROM_RWS_ACT_ENERGY_START, ReverseActiveEnergy);
    ExtMem.put(EEPROM_ABS_ACT_ENERGY_START, AbsoluteActiveEnergy);
    ExtMem.put(EEPROM_FWD_INDREACT_EENERGY_START, Forward_InductiveReactiveEnergy);
    ExtMem.put(EEPROM_RWS_CAPREACT_EENERGY_START, Reverse_CapacitiveReactiveEnergy);
    ExtMem.put(EEPROM_ABS_REACT_ENERGY_START, AbsoluteReactiveEnergy);
    ExtMem.put(EEPROM_MTRING_STATUS_START, MeteringStatus);
    ExtMem.put(EEPROM_L_LINE_CURRENTRMS_START, L_LineCurrentrms);
    ExtMem.put(EEPROM_VOLTAGE_RMS_START, Voltagerms);
    ExtMem.put(EEPROM_L_LINE_MEANACT_POWER_START, L_LineMeanActivePower);
    ExtMem.put(EEPROM_L_LINE_MEANREACT_POWER_START, L_LineMeanReactivePower);
    ExtMem.put(EEPROM_VOLTAGE_FREQUENCY_START, VoltageFrequency);
    ExtMem.put(EEPROM_L_LINE_PWR_FACTOR_START, L_LinePowerFactor);
    ExtMem.put(EEPROM_PH_ANG_VOL_L_CUR_START, PhaseAnglebetweenVoltageandL_LineCurrent);
    ExtMem.put(EEPROM_L_LINE_MEANAP_PWR_START, L_LineMeanApparentPower);
    ExtMem.put(EEPROM_N_LINE_CURRENTRMS_START, N_LineCurrentrms);
    ExtMem.put(EEPROM_N_LINE_MEANACT_POWER_START, N_LineMeanActivePower);
    ExtMem.put(EEPROM_N_LINE_MEAN_REACT_POWER_START, N_LineMeanReactivePower);
    ExtMem.put(EEPROM_N_LINE_PWR_FACTOR_START, N_LinePowerFactor);
    ExtMem.put(EEPROM_PH_ANG_VOL_N_CUR_START, PhaseAnglebetweenVoltageandN_LineCurrent);
    ExtMem.put(EEPROM_N_LINE_MEANAP_PWR_START, N_LineMeanApparentPower);



}


void EEPROM_Updtate_DHT_DATA(float &Temp, float &Temp_F, float &Hum, float &hic, float &hif) {

    ExtMem.put(EEPROM_TEMPERATURE_START, Temp);
    ExtMem.put(EEPROM_TEMPERATURE_F_START, Temp_F);
    ExtMem.put(EEPROM_HUMIDITY_START, Hum);
    ExtMem.put(EEPROM_HEATINDEX_TEMP_START, hic);
    ExtMem.put(EEPROM_HEATINDEX_TEMPF_START, hif);


}







void Display_BIN16(int var) {  // Display BIN from Var
    for (unsigned int i = 0x8000; i; i >>= 1) {
        Serial.write(var & i ? '1' : '0');
    }
    Serial.print(" ");
}

void Display_HEX(unsigned long var, unsigned char numChars) {  // Display Hex from Var
    unsigned long mask = 0x0000000F;
    mask = mask << 4 * (numChars - 1);

    for (unsigned int i = numChars; i > 0; --i) {
        Serial.print(((var & mask) >> (i - 1) * 4), HEX);
        mask = mask >> 4;
    }
    Serial.print(" ");
}


void EEPROM_READ_Power_Registers(void) {

    ExtMem.get(EEPROM_FWD_ACT_ENERGY_START, EEPROM_ForwardActiveEnergy);
    ExtMem.get(EEPROM_RWS_ACT_ENERGY_START, EEPROM_ReverseActiveEnergy);
    ExtMem.get(EEPROM_ABS_ACT_ENERGY_START, EEPROM_AbsoluteActiveEnergy);
    ExtMem.get(EEPROM_FWD_INDREACT_EENERGY_START, EEPROM_Forward_InductiveReactiveEnergy);
    ExtMem.get(EEPROM_RWS_CAPREACT_EENERGY_START, EEPROM_Reverse_CapacitiveReactiveEnergy);
    ExtMem.get(EEPROM_ABS_REACT_ENERGY_START, EEPROM_AbsoluteReactiveEnergy);
    ExtMem.get(EEPROM_MTRING_STATUS_START, EEPROM_MeteringStatus);
    ExtMem.get(EEPROM_L_LINE_CURRENTRMS_START, EEPROM_L_LineCurrentrms);
    ExtMem.get(EEPROM_VOLTAGE_RMS_START, EEPROM_Voltagerms);
    ExtMem.get(EEPROM_L_LINE_MEANACT_POWER_START, EEPROM_L_LineMeanActivePower);
    ExtMem.get(EEPROM_L_LINE_MEANREACT_POWER_START, EEPROM_L_LineMeanReactivePower);
    ExtMem.get(EEPROM_VOLTAGE_FREQUENCY_START, EEPROM_VoltageFrequency);
    ExtMem.get(EEPROM_L_LINE_PWR_FACTOR_START, EEPROM_L_LinePowerFactor);
    ExtMem.get(EEPROM_PH_ANG_VOL_L_CUR_START, EEPROM_PhaseAnglebetweenVoltageandL_LineCurrent);
    ExtMem.get(EEPROM_L_LINE_MEANAP_PWR_START, EEPROM_L_LineMeanApparentPower);
    ExtMem.get(EEPROM_N_LINE_CURRENTRMS_START, EEPROM_N_LineCurrentrms);
    ExtMem.get(EEPROM_N_LINE_MEANACT_POWER_START, EEPROM_N_LineMeanActivePower);
    ExtMem.get(EEPROM_N_LINE_MEAN_REACT_POWER_START, EEPROM_N_LineMeanReactivePower);
    ExtMem.get(EEPROM_N_LINE_PWR_FACTOR_START, EEPROM_N_LinePowerFactor);
    ExtMem.get(EEPROM_PH_ANG_VOL_N_CUR_START, EEPROM_PhaseAnglebetweenVoltageandN_LineCurrent);
    ExtMem.get(EEPROM_N_LINE_MEANAP_PWR_START, EEPROM_N_LineMeanApparentPower);

    yield();
    DBUGS.println("Forward Active Energy : "+ String(EEPROM_ForwardActiveEnergy,4)+ "KWh");
    yield();
    DBUGS.println("Reverse Active Energy : " + String(EEPROM_ReverseActiveEnergy,4) + "KWh");
    yield();
    DBUGS.println("Absolute Active Energy : " + String(EEPROM_AbsoluteActiveEnergy,4) + "KWh");
    yield();
    DBUGS.println("Forward (Inductive) Reactive Energy : " + String(EEPROM_Forward_InductiveReactiveEnergy,4) + "KVarh");
    yield();
    DBUGS.println("Reverse (Capacitive) Reactive Energy : " + String(EEPROM_Reverse_CapacitiveReactiveEnergy,4) + "KVarh");
    yield();
    DBUGS.println("Absolute Reactive Energy : " + String(EEPROM_AbsoluteReactiveEnergy,4) + "KVarh");

    yield();
    //Serial.print("Meter Status \t\t\t(EnStatus 0x46):\t0x");
    
    DBUGS.println("Metering Status : " );

    Display_HEX(EEPROM_MeteringStatus, 4);
    Display_BIN16(EEPROM_MeteringStatus);
    if (!bitRead(EEPROM_MeteringStatus, 1) && !bitRead(EEPROM_MeteringStatus, 0)) DBUGS.print("LNMode.AntiTamper ");
    if (!bitRead(EEPROM_MeteringStatus, 1) && bitRead(EEPROM_MeteringStatus, 0)) DBUGS.print("LNMode.FixedL ");
    if (bitRead(EEPROM_MeteringStatus, 1) && !bitRead(EEPROM_MeteringStatus, 0)) DBUGS.print("LNMode.LN ");
    if (bitRead(EEPROM_MeteringStatus, 1) && bitRead(EEPROM_MeteringStatus, 0)) DBUGS.print("LNMode.Flexible ");
    if (bitRead(EEPROM_MeteringStatus, 11)) DBUGS.print("Lline.AntiTamperL ");
    if (!bitRead(EEPROM_MeteringStatus, 11)) DBUGS.print("Lline.AntiTamperN ");
    if (bitRead(EEPROM_MeteringStatus, 12)) DBUGS.print("RevP.CF1ActiveReverse ");
    if (!bitRead(EEPROM_MeteringStatus, 12)) DBUGS.print("RevP.CF1ActiveForward ");
    if (bitRead(EEPROM_MeteringStatus, 13)) DBUGS.print("RevQ.CF2ReActiveReverse ");
    if (!bitRead(EEPROM_MeteringStatus, 13)) DBUGS.print("RevQ.CF2ReActiveForward ");
    if (bitRead(EEPROM_MeteringStatus, 14)) DBUGS.print("Pnoload.NoLoadActive ");
    if (!bitRead(EEPROM_MeteringStatus, 14)) DBUGS.print("Pnoload.NoLoadNotActive ");
    if (bitRead(EEPROM_MeteringStatus, 15)) DBUGS.print("Qnoload.ReactiveNoLoad ");
    if (!bitRead(EEPROM_MeteringStatus, 15)) DBUGS.print("Qnoload.NotReactiveNoLoad ");
    Serial.println();
    if (EEPROM_MeteringStatus == 0x2801) DBUGS.println(">ATM 0x46- #2801 Accumulator Populated");
    if (EEPROM_MeteringStatus == 0xC801) DBUGS.println(">ATM 0x46- #C801 Accumulator Not Running");
    if (EEPROM_MeteringStatus == 0xC800) DBUGS.println(">ATM 0x46 - #C800 Meter Status Default Value");
    if (EEPROM_MeteringStatus == 0xFFFF) DBUGS.println(">ATM 0x46 - #FFFF Failed | Fault on ATM | Reboot Needed");
    if (EEPROM_MeteringStatus == 0x0000) DBUGS.println(">ATM 0x46 - #0000 ERROR!: Possible ATM Hardware Issue\n\n");


    //String(val, decimalPlaces)
    yield();
    DBUGS.println("L Line Current rms : " + String(EEPROM_L_LineCurrentrms,4) + "A");
    yield();
    DBUGS.println("Voltage rms: " + String(EEPROM_Voltagerms,4) + "V");
    yield();
    DBUGS.println("L Line Mean Active Power : " + String(EEPROM_L_LineMeanActivePower,4) + "KW");
    yield();
    DBUGS.println("L Line Mean Reactive Power : " + String(EEPROM_L_LineMeanReactivePower,4) + "KVar");
    yield();
    DBUGS.println("Voltage Frequency: " + String(EEPROM_VoltageFrequency,4) + "Hz");
    yield();
    DBUGS.println("L Line Power Factor : " + String(EEPROM_L_LinePowerFactor,4));
    yield();
    DBUGS.println("Phase Angle between Voltage and L Line Current : " + String(EEPROM_PhaseAnglebetweenVoltageandL_LineCurrent,4) + "deg");
    yield();
    DBUGS.println("L Line Mean Apparent Power : " + String(EEPROM_L_LineMeanApparentPower,4) + "KVA");
    yield();
    DBUGS.println("N Line Current rms : " + String(EEPROM_N_LineCurrentrms,4) + "A");
    yield();
    DBUGS.println("N Line Mean Active Power : " + String(EEPROM_N_LineMeanActivePower,4) + "KW");
    yield();
    DBUGS.println("N Line Mean Reactive Power : " + String(EEPROM_N_LineMeanReactivePower,4) + "KVar");
    yield();
    DBUGS.println("N Line Power Factor : " + String(EEPROM_N_LinePowerFactor,4));
    yield();
    DBUGS.println("Phase Angle between Voltage and N Line Current: " + String(EEPROM_PhaseAnglebetweenVoltageandN_LineCurrent,4) + "deg");
    yield();
    DBUGS.println("N Line Mean Apparent Power : " + String(EEPROM_N_LineMeanApparentPower,4) + "KVA");


}


void EEPROM_Read_DHT_DATA(void) {

    ExtMem.get(EEPROM_TEMPERATURE_START, EEPROM_Temperature_Val);
    ExtMem.get(EEPROM_TEMPERATURE_F_START, EEPROM_TemperatureF_Val);
    ExtMem.get(EEPROM_HUMIDITY_START, EEPROM_Humidity_Val);
    ExtMem.get(EEPROM_HEATINDEX_TEMP_START, EEPROM_HeatIndex_Temp_Val);
    ExtMem.get(EEPROM_HEATINDEX_TEMPF_START, EEPROM_HeatIndex_TempF_Val);

    yield();
    DBUGS.println("Temperature : " + String(EEPROM_Temperature_Val, 2) + "°C");
    yield();
    DBUGS.println("Temperatur (F) : " + String(EEPROM_TemperatureF_Val, 2) + "°F");
    yield();
    DBUGS.println("Humidity : " + String(EEPROM_Humidity_Val, 2) + "%");
    yield();
    DBUGS.println("Heat Index (C) : " + String(EEPROM_HeatIndex_Temp_Val, 2) + "°C");
    yield();
    DBUGS.println("Heat Index (F) : " + String(EEPROM_HeatIndex_TempF_Val, 2) + "°F");




}



void config_save_emoncms(String server, String path, String node, String apikey, String fingerprint)
{
    Wire.begin();

    emoncms_server = server;
    emoncms_path = path;
    emoncms_node = node;
    emoncms_apikey = apikey;
    emoncms_fingerprint = fingerprint;

    //Connect to a EEPROM with ADR0 set to VCC and use the Wire1 hardware to talk to the EEPROM
    if (ExtMem.begin(EEPROM_ADDRESS, Wire) == false) //And Uno will fail to compile here
    {
        DBUGS.println("No memory detected. Freezing.");
        while (true);
    }

    // save apikey to EEPROM
    ExtMem.putString(EEPROM_EMON_API_KEY_START, emoncms_apikey); //(location, data


    // save emoncms server to EEPROM max 45 characters
    ExtMem.putString(EEPROM_EMON_SERVER_START, emoncms_server); //(location, dat


    // save emoncms server to EEPROM max 16 characters
    ExtMem.putString(EEPROM_EMON_PATH_START, emoncms_path); //(location, dat


    // save emoncms node to EEPROM max 32 characters
    ExtMem.putString(EEPROM_EMON_NODE_START, emoncms_node); //(location, 


    // save emoncms HTTPS fingerprint to EEPROM max 60 characters
    ExtMem.putString(EEPROM_EMON_FINGERPRINT_START, emoncms_fingerprint); //(location, 


   
}

void EEPROM_Write_String(uint32_t Adds, String val) {

    

    if (ExtMem.begin(EEPROM_ADDRESS, Wire) == false) //And Uno will fail to compile here
    {
        DBUGS.println("No memory detected. Freezing.");
        while (true);
    }

    // Save MQTT server max 45 characters
    ExtMem.put(Adds, val); //(location, data)



}

void config_save_mqtt(String server, String topic, String prefix, String user, String pass)
{
    
    Wire.begin();

    mqtt_server = server;
    mqtt_topic = topic;
    mqtt_feed_prefix = prefix;
    mqtt_user = user;
    mqtt_pass = pass;


    //Connect to a EEPROM with ADR0 set to VCC and use the Wire1 hardware to talk to the EEPROM
    if (ExtMem.begin(EEPROM_ADDRESS, Wire) == false) //And Uno will fail to compile here
    {
        DBUGS.println("No memory detected. Freezing.");
        while (true);
    }

    // Save MQTT server max 45 characters
    ExtMem.putString(EEPROM_MQTT_SERVER_START, mqtt_server); //(location, data)

    // Save MQTT topic max 32 characters
    ExtMem.putString(EEPROM_MQTT_TOPIC_START, mqtt_topic); //(location, data)


    // Save MQTT topic separator max 10 characters
    ExtMem.putString(EEPROM_MQTT_FEED_PREFIX_START, mqtt_feed_prefix); //(location, data


    // Save MQTT username max 32 characters
    ExtMem.putString(EEPROM_MQTT_USER_START, mqtt_user); //(location, data

    // Save MQTT pass max 64 characters
    ExtMem.putString(EEPROM_MQTT_PASS_START, mqtt_pass); //(location, data
   
}



void config_save_admin(String user, String pass)
{
  //EEPROM.begin(EEPROM_SIZE);

  www_username = user;
  www_password = pass;

  Wire.begin();
  //Connect to a EEPROM with ADR0 set to VCC and use the Wire1 hardware to talk to the EEPROM
  if (ExtMem.begin(EEPROM_ADDRESS, Wire) == false) //And Uno will fail to compile here
  {
      DBUGS.println("No memory detected. Freezing.");
      while (true);
  }

  ExtMem.put(EEPROM_WWW_USER_START, user); //(location, data)
  ExtMem.put(EEPROM_WWW_PASS_START, pass); //(location, data)



  //EEPROM.commit();
}



void config_save_wifi(String qsid, String qpass)
{

    Wire.begin();

    esid = qsid;
    epass = qpass;


    ExtMem.setMemoryType(256); // Valid types: 0, 1, 2, 4, 8, 16, 32, 64, 128, 256, 512, 1025, 2048

      //Connect to a EEPROM with ADR0 set to VCC and use the Wire1 hardware to talk to the EEPROM
    if (ExtMem.begin(EEPROM_ADDRESS, Wire) == false) //And Uno will fail to compile here
    {
        DBUGS.println("No memory detected. Freezing.");
        while (true);
    }




    DBUGS.println();
    DBUGS.println();
    DBUGS.println("Memory detected!");
    DBUGS.println();
    ExtMem.put(EEPROM_ESID_START, qsid); //(location, data)
    //String qsid;
    //ExtMem.get(EEPROM_ESID_START, qsid); //location to read, thing to put data into
    //DBUGS.println("SSID : "+ qsid);


    ExtMem.put(EEPROM_EPASS_START, qpass); //(location, data)
    //String qpass;
    //ExtMem.get(EEPROM_EPASS_START, qpass); //location to read, thing to put data into
    //DBUGS.println("PASSWORD : "+ qpass);
   // DBUGS.println();
    //DBUGS.println();


}


void EEPROM_Read_String(uint32_t Adds, String DataRead) {
    
    Wire.begin();

    if (ExtMem.begin(EEPROM_ADDRESS, Wire) == false) //And Uno will fail to compile here
    {
        DBUGS.println("No memory detected. Freezing.");
        while (true);
    }

    ExtMem.get(Adds, DataRead); //location to read, thing to put data into


//return DataRead;
}


void config_reset()
{
    ExtMem.erase(0xff);
}


String getMacAddress()
{
    uint8_t baseMac[6];

    // Get MAC address for WiFi station
    esp_read_mac(baseMac, ESP_MAC_WIFI_STA);

    char baseMacChr[18] = { 0 };
    sprintf(baseMacChr, "%02X:%02X:%02X:%02X:%02X:%02X", baseMac[0], baseMac[1], baseMac[2], baseMac[3], baseMac[4], baseMac[5]);

    String macAddress = String(baseMacChr);



    return String(baseMacChr);
}

/*
bool EEPROM_WriteString(int Addr,String Input) {
    char CharBuf[15];
    Input.toCharArray(CharBuf, 32);
    return EEPROMEX.writeBlock<char>(Addr, CharBuf, 32);

}

String Read_StringEEPROM(int Addr) {
    String OutputEEPROM;
    char Ouput[] = "";

    EEPROMEX.readBlock<char>(Addr, Ouput, 32);
    OutputEEPROM = String(Ouput);
    return OutputEEPROM;



}

*/



 //(i2c device address of eeprom, address to be written to, String to be written)

