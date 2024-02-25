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

#ifndef _EMONESP_CONFIG_H
#define _EMONESP_CONFIG_H

#include <Arduino.h>
#include <EEPROM.h>                   // Save config settings
#include <Wire.h>
#include "Typedef_Data.h"
#include "EEPROM_Defs.h"



// -------------------------------------------------------------------
// Load and save the EmonESP config.
//
// This initial implementation saves the config to the EEPROM area of flash
// -------------------------------------------------------------------

// Global config varables

// Global config varables
extern const String node_type;
extern const String node_description;
extern uint16_t node_id;
extern String node_name;
extern String node_describe;
extern char id[41];

extern String Vendor_Name;
extern String Product_Code;
extern String Revision_Number;



extern String meter_name;
extern String model;
extern String manufacturer;
extern String serial_no;
extern String hardware_rev;
extern String firmware_ver;
extern String mac_add;
extern String node;
extern uint16_t node_ID;




// Wifi Network Strings
extern String esid;
extern String epass;

extern const char* ssid; // Change this to your WiFi SSID
extern const char* password; // Change this to your WiFi password

//extern IPAddress staticIP_1;
//extern IPAddress gateway_1;
//extern IPAddress subnet_1;

extern const char* MQTT_server;
extern int mqtt_port;

// Web server authentication (leave blank for none)
extern String www_username;
extern String www_password;

// EMONCMS SERVER strings
extern String emoncms_server;
extern String emoncms_path;
extern String emoncms_node;
extern String emoncms_apikey;
extern String emoncms_fingerprint;

// MQTT Settings
extern String mqtt_server;
extern String mqtt_topic;
extern String mqtt_user;
extern String mqtt_pass;
extern String mqtt_feed_prefix;

// Calibration Settings for SM energy meter
extern String voltage_cal;
extern String ct1_cal;
extern String ct2_cal;
extern String freq_cal;
extern String gain_cal;
#ifdef SOLAR_METER
extern String svoltage_cal;
extern String sct1_cal;
extern String sct2_cal;
#endif


//================================================================================
        // Metering and Measurement CALIBRATION VARIABLE
//================================================================================
extern unsigned short     VoltageSagThreshold;                //  (u16) Voltage Sag Threshold
extern unsigned short     L_LineCalibrationGain;              //  (u16) L Line Calibration Gain
extern unsigned short     L_LineCalibrationAngle;             //  (u16) L Line Calibration Angle
extern unsigned short     N_LineCalibrationGain;              //  (u16) N Line Calibration Gain
extern unsigned short     N_LineCalibrationAngle;             //  (u16) N Line Calibration Angle
extern unsigned short     ActiveStartupPowerThreshold;        //  (u16) Active Startup Power Threshold
extern unsigned short     ActiveNoLoadPowerThreshold;         //  (u16) Active No-Load Power Threshold
extern unsigned short     ReactiveStartupPowerThreshold;      //  (u16) Reactive Startup Power Threshold
extern unsigned short     ReactiveNoLoadPowerThreshold;       //  (u16) Reactive No-Load Power Threshold
extern unsigned short     MeasurementCalibrationStart;        //  (u16) Measurement Calibration Start Command
extern unsigned short     VoltagermsGain;                     //  (u16) Voltage rms Gain
extern unsigned short     L_LineCurrentrmsGain;				//  (u16) L Line Current rms Gain
extern unsigned short     N_LineCurrentrmsGain;				//  (u16) N Line Current rms Gain
extern unsigned short     VoltageOffset;						//  (u16) Voltage Offset
extern unsigned short     L_LineCurrentOffset;				//  (u16) L Line Current Offset
extern unsigned short     N_LineCurrentOffset;                //  (u16) N Line Current Offset
extern unsigned short     L_LineActivePowerOffset;            //  (u16) L Line Active Power Offset
extern unsigned short     L_LineReactivePowerOffset;          //  (u16) L Line Reactive Power Offset
extern unsigned short     N_LineActivePowerOffset;			//  (u16) N Line Active Power Offset
extern unsigned short     N_LineReactivePowerOffset;			//  (u16) N Line Reactive Power Offset
extern unsigned short     Checksum1;                          //  (u16) Checksum 1
extern unsigned short     Checksum2;                          //  (u16) Checksum 2

extern unsigned short voltageread;


//================================================================================

//================================================================================
//  DESIGN CONFIGURATION VARIABLES
//SystemConfigurationFlag SystemConfiguration;    //  (b32)
//EventConfigurationFlag  EventConfiguration;     //  (b32)

extern unsigned short      MeteringModeConfiguration;          //  (u16) Metering Mode Configuration



//================================================================================

//================================================================================
        // Metering and Measurement VARIABLE
//================================================================================
extern float      ForwardActiveEnergy;          //  (u16) Forward Active Energy
extern float      ReverseActiveEnergy;          //  (u16) Reverse Active Energy
extern float      AbsoluteActiveEnergy;          //  (u16) Absolute Active Energy
extern float      Forward_InductiveReactiveEnergy;          //  (u16) Forward (Inductive) Reactive Energy
extern float      Reverse_CapacitiveReactiveEnergy;          //  (u16) Reverse (Capacitive) Reactive Energy
extern float      AbsoluteReactiveEnergy;          //  (u16) Absolute Reactive Energy
extern unsigned short      MeteringStatus;                     //  (u16) Metering Status
extern float      L_LineCurrentrms;          //  (u16) L Line Current rms
extern float      Voltagerms;          //  (u16) Voltage rms
extern float      L_LineMeanActivePower;          //  (u16) L Line Mean Active Power
extern float      L_LineMeanReactivePower;          //  (u16) L Line Mean Reactive Power
extern float      VoltageFrequency;          //  (u16) Voltage Frequency
extern float      L_LinePowerFactor;          //  (u16) L Line Power Factor
extern float      PhaseAnglebetweenVoltageandL_LineCurrent;          //  (u16) Phase Angle between Voltage and L Line Current
extern float      L_LineMeanApparentPower;          //  (u16) L Line Mean Apparent Power
extern float      N_LineCurrentrms;          //  (u16) N Line Current rms
extern float      N_LineMeanActivePower;          //  (u16) N Line Mean Active Power
extern float      N_LineMeanReactivePower;          //  (u16) N Line Mean Reactive Power
extern float      N_LinePowerFactor;          //  (u16) N Line Power Factor
extern float      PhaseAnglebetweenVoltageandN_LineCurrent;          //  (u16) Phase Angle between Voltage and N Line Current
extern float      N_LineMeanApparentPower;          //  (u16) N Line Mean Apparent Power



//================================================================================

extern float Hum, Temp, Temp_F, hif, hic;

//================================================================================
        // Metering and Measurement VARIABLE
//================================================================================
extern float    EEPROM_ForwardActiveEnergy;          //  EEPROM Read Value - Forward Active Energy
extern float    EEPROM_ReverseActiveEnergy;          //  EEPROM Read Value - Reverse Active Energy
extern float    EEPROM_AbsoluteActiveEnergy;          //  EEPROM Read Value - Absolute Active Energy
extern float    EEPROM_Forward_InductiveReactiveEnergy;          //  EEPROM Read Value -  (Inductive) Reactive Energy
extern float    EEPROM_Reverse_CapacitiveReactiveEnergy;          //  EEPROM Read Value -  (Capacitive) Reactive Energy
extern float    EEPROM_AbsoluteReactiveEnergy;          //  EEPROM Read Value -  Reactive Energy
extern unsigned short    EEPROM_MeteringStatus;                     //  EEPROM Read Value - Metering Status
extern float    EEPROM_L_LineCurrentrms;          //  EEPROM Read Value - L Line Current rms
extern float    EEPROM_Voltagerms;          //  EEPROM Read Value -  rms
extern float    EEPROM_L_LineMeanActivePower;          //  EEPROM Read Value -  Line Mean Active Power
extern float    EEPROM_L_LineMeanReactivePower;          //  EEPROM Read Value -  Line Mean Reactive Power
extern float    EEPROM_VoltageFrequency;          //  EEPROM Read Value -  Frequency
extern float    EEPROM_L_LinePowerFactor;          //  EEPROM Read Value -  Line Power Factor
extern float    EEPROM_PhaseAnglebetweenVoltageandL_LineCurrent;          //  EEPROM Read Value -  Angle between Voltage and L Line Current
extern float    EEPROM_L_LineMeanApparentPower;          //  EEPROM Read Value - L Line Mean Apparent Power
extern float    EEPROM_N_LineCurrentrms;          //  EEPROM Read Value -  Line Current rms
extern float    EEPROM_N_LineMeanActivePower;          //  EEPROM Read Value -  Line Mean Active Power
extern float    EEPROM_N_LineMeanReactivePower;          //  EEPROM Read Value -  Line Mean Reactive Power
extern float    EEPROM_N_LinePowerFactor;          //  EEPROM Read Value - N Line Power Factor
extern float    EEPROM_PhaseAnglebetweenVoltageandN_LineCurrent;          //  EEPROM Read Value - Phase Angle between Voltage and N Line Current
extern float    EEPROM_N_LineMeanApparentPower;          //  EEPROM Read Value - N Line Mean Apparent Power

extern float    EEPROM_Temperature_Val;          //  EEPROM Read Value - Temparature Value °C
extern float    EEPROM_TemperatureF_Val;          //  EEPROM Read Value - Temparature Value ° F
extern float    EEPROM_Humidity_Val;          //  EEPROM Read Value - Humidity Value %
extern float    EEPROM_HeatIndex_Temp_Val;          //  EEPROM Read Value - Temparature Value °C
extern float    EEPROM_HeatIndex_TempF_Val;          //  EEPROM Read Value - Temparature Value °F


//================================================================================

extern char EEPROM_Vendor_Name[30];
extern char EEPROM_Product_Code[15];
extern char EEPROM_Revision_Number[8];

extern char EEPROM_Meter_name[20];
extern char EEPROM_Model[20];
extern char EEPROM_Manufacturer[30];
extern char EEPROM_Serial_no[30];
extern char EEPROM_Hardware_rev[8];
extern char EEPROM_Firmware_ver[20];

extern char EEPROM_MAC_add[30];
extern char EEPROM_node[20];
extern uint16_t EEPROM_node_ID;

// -------------------------------------------------------------------
// Load saved settings
// -------------------------------------------------------------------
extern void config_load_settings();

extern void EEPROM_Updtate_DHT_DATA(float& Temp, float& Temp_F, float& Hum, float& hic, float& hif);
extern void EEPROM_Read_DHT_DATA(void);

extern void Save_CalConfiguration(void);
extern void Load_CalConfiguration(void);
extern void EEPROM_READ_Power_Registers(void);

extern void EEPROM_Update_ManufactureInfo(void);
extern void EEPROM_READ_ManufactureInfo(void);

extern void EEPROM_Update_Power_Registers(void);
// -------------------------------------------------------------------
// Save the EmonCMS server details
// -------------------------------------------------------------------
extern void config_save_emoncms(String server, String path, String node, String apikey, String fingerprint);

// -------------------------------------------------------------------
// Save the MQTT broker details
// -------------------------------------------------------------------
extern void config_save_mqtt(String server, String topic, String prefix, String user, String pass);

// -------------------------------------------------------------------
// Save the Calibration details
// -------------------------------------------------------------------
#ifdef SOLAR_METER
extern void config_save_cal(String voltage, String ct1, String ct2, String freq, String gain, String svoltage, String sct1, String sct2);
#else
extern void config_save_cal(String voltage, String ct1, String ct2, String freq, String gain);
#endif

// -------------------------------------------------------------------
// Save the admin/web interface details
// -------------------------------------------------------------------
extern void config_save_admin(String user, String pass);

// -------------------------------------------------------------------
// Save the Wifi details
// -------------------------------------------------------------------
extern void config_save_wifi(String qsid, String qpass);

// -------------------------------------------------------------------
// Reset the config back to defaults
// -------------------------------------------------------------------
extern void config_reset();

extern String getMacAddress();
extern void EEPROM_Write_String(uint32_t Adds, String val);


extern void EEPROM_Read_String(uint32_t Adds, String DataRead);

#endif // _EMONESP_CONFIG_H
