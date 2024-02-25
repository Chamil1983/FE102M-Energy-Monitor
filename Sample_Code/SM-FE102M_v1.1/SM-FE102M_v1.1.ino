// Libraries




#include <ArduinoJson.h>

#include <AsyncMqttClient.h>
#include <AsyncMqttClient.hpp>

#include <ssl_client.h>
#include <WiFiClientSecure.h>
#include <ezTime.h>
#include <esp_task_wdt.h>
#include "Debug.h"
#include "emonesp.h"
#include "EnergyATM90E26.h"
#include "FE102M_Defaults.h"
#include "config.h"
#include "emoncms.h"
#include "input.h"
#include "mqtt.h"
#include <DHT.h>
#include "Modbus_Serial.h"
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <driver/adc.h>
#include <EEPROM.h>
#include <MicrocontrollerID.h>
#include "Definitions.h"

#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include "SPIFFS.h"
#include <Arduino_JSON.h>
#include <Ticker.h> //https://github.com/sstaub/Ticker

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);

// Create a WebSocket object
AsyncWebSocket ws("/ws");

// Json Variable to Hold Data
JSONVar sysdata, configdata, readings;




Ticker timer;

void Modbus_Poling(void);
void Load_Power_Registers(void);
void Setup_Modbus(void);

//#include <NTPClient.h>

#include "time.h"

const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = 39600;
const int   daylightOffset_sec = 3600;

//WiFiUDP ntpUDP;
//NTPClient timeClient(ntpUDP, "europe.pool.ntp.org", time_offset, 60000);

// ######### OBJECTS #########
ATM90E26_SPI eic(05);

DHT dht(DHTPIN, DHTTYPE);

Timezone AUS;



// **************** FUNCTIONS AND ROUTINES ****************



void DisplayBIN16(int var) {  // Display BIN from Var
    for (unsigned int i = 0x8000; i; i >>= 1) {
        Serial.write(var & i ? '1' : '0');
    }
    Serial.print(" ");
}

void DisplayHEX(unsigned long var, unsigned char numChars) {  // Display Hex from Var
    unsigned long mask = 0x0000000F;
    mask = mask << 4 * (numChars - 1);

    for (unsigned int i = numChars; i > 0; --i) {
        Serial.print(((var & mask) >> (i - 1) * 4), HEX);
        mask = mask >> 4;
    }
    Serial.print(" ");
}

void DisplayRegisters() {  // Display Diagnostic Report

  // Header
    Serial.println("GTEM-1 ATM90E26 Energy Monitoring Energy Monitor - Register Status and Diagnostic Report");
    Serial.printf("ESP32 Serial ID = %04X", (uint16_t)(chipid >> 32));
    Serial.printf("%08X", (uint32_t)chipid);
    Serial.print("   Firmware Version = ");
    Serial.println(FirmwareVersion);
    Serial.println();
    Serial.println("Register Name\t\t\tVar/Address\t\tValue / Binary / Information");
    Serial.println("------------ \t\t\t-----------\t\t--------------------------------------------------------");

    // System Status
    Serial.print("System Status \t\t\t(SysStatus 0x01):\t0x");
    ReadValue = eic.GetSysStatus();
    DisplayHEX(ReadValue, 4);
    DisplayBIN16(ReadValue);
    if (bitRead(ReadValue, 1)) Serial.print("SagWarn.Enabled ");
    if (bitRead(ReadValue, 13) && bitRead(ReadValue, 12)) Serial.print("CheckSumError.CS2 ");
    if (bitRead(ReadValue, 15) && bitRead(ReadValue, 14)) Serial.print("CheckSumError.CS1 ");
    Serial.println();
    if (ReadValue == 0x0000) Serial.println(">ATM 0x01 - #0000 System Status Default Value");
    if (ReadValue == 0xFFFF) Serial.println(">ATM 0x01 - #FFFF Failed | Fault on ATM | Reboot Needed");

    // Meter Status
    yield();
    Serial.print("Meter Status \t\t\t(EnStatus 0x46):\t0x");
    ReadValue = eic.GetMeterStatus();
    DisplayHEX(ReadValue, 4);
    DisplayBIN16(ReadValue);
    if (!bitRead(ReadValue, 1) && !bitRead(ReadValue, 0)) Serial.print("LNMode.AntiTamper ");
    if (!bitRead(ReadValue, 1) && bitRead(ReadValue, 0)) Serial.print("LNMode.FixedL ");
    if (bitRead(ReadValue, 1) && !bitRead(ReadValue, 0)) Serial.print("LNMode.LN ");
    if (bitRead(ReadValue, 1) && bitRead(ReadValue, 0)) Serial.print("LNMode.Flexible ");
    if (bitRead(ReadValue, 11)) Serial.print("Lline.AntiTamperL ");
    if (!bitRead(ReadValue, 11)) Serial.print("Lline.AntiTamperN ");
    if (bitRead(ReadValue, 12)) Serial.print("RevP.CF1ActiveReverse ");
    if (!bitRead(ReadValue, 12)) Serial.print("RevP.CF1ActiveForward ");
    if (bitRead(ReadValue, 13)) Serial.print("RevQ.CF2ReActiveReverse ");
    if (!bitRead(ReadValue, 13)) Serial.print("RevQ.CF2ReActiveForward ");
    if (bitRead(ReadValue, 14)) Serial.print("Pnoload.NoLoadActive ");
    if (!bitRead(ReadValue, 14)) Serial.print("Pnoload.NoLoadNotActive ");
    if (bitRead(ReadValue, 15)) Serial.print("Qnoload.ReactiveNoLoad ");
    if (!bitRead(ReadValue, 15)) Serial.print("Qnoload.NotReactiveNoLoad ");
    Serial.println();
    if (ReadValue == 0x2801) Serial.println(">ATM 0x46- #2801 Accumulator Populated");
    if (ReadValue == 0xC801) Serial.println(">ATM 0x46- #C801 Accumulator Not Running");
    if (ReadValue == 0xC800) Serial.println(">ATM 0x46 - #C800 Meter Status Default Value");
    if (ReadValue == 0xFFFF) Serial.println(">ATM 0x46 - #FFFF Failed | Fault on ATM | Reboot Needed");
    if (ReadValue == 0x0000) Serial.println(">ATM 0x46 - #0000 ERROR!: Possible ATM Hardware Issue\n\n");

    // MMode Metering Status
    yield();
    Serial.print("MMode Status \t\t\t(MMode 0x2B):\t\t0x");
    ReadValue = eic.GetMModeStatus();
    DisplayHEX(ReadValue, 4);
    DisplayBIN16(ReadValue);
    if (!bitRead(ReadValue, 5) && !bitRead(ReadValue, 4)) Serial.print("MMode.PositiveZeroCrossing ");
    if (!bitRead(ReadValue, 5) && bitRead(ReadValue, 4)) Serial.print("MMode.NegativeZeroCrossing ");
    if (bitRead(ReadValue, 5) && !bitRead(ReadValue, 4)) Serial.print("MMode.AllZeroCrossing ");
    if (bitRead(ReadValue, 5) && bitRead(ReadValue, 4)) Serial.print("MMode.NoZeroCrossing ");
    if (bitRead(ReadValue, 10)) Serial.print("MMode.LNSel.LLine(Default) ");
    if (!bitRead(ReadValue, 10)) Serial.print("MMode.LNSel.NLine ");
    if (!bitRead(ReadValue, 12) && !bitRead(ReadValue, 11)) Serial.print("MMode.NLine.CurrentGain2 ");
    if (!bitRead(ReadValue, 12) && bitRead(ReadValue, 11)) Serial.print("MMode.NLine.CurrentGain4 ");
    if (bitRead(ReadValue, 12) && !bitRead(ReadValue, 11)) Serial.print("MMode.NLine.CurrentGain1 ");
    if (bitRead(ReadValue, 12) && bitRead(ReadValue, 11)) Serial.print("MMode.NLine.CurrentGain1 ");
    if (bitRead(ReadValue, 15)) Serial.print("MMode.CurrentChannelGain1 ");
    if (!bitRead(ReadValue, 15) && !bitRead(ReadValue, 14) && !bitRead(ReadValue, 13)) Serial.print("MMode.LGain.CurrentChannelGain4 ");
    if (!bitRead(ReadValue, 15) && !bitRead(ReadValue, 14) && bitRead(ReadValue, 13)) Serial.print("MMode.LGain.CurrentChannelGain8 ");
    if (!bitRead(ReadValue, 15) && bitRead(ReadValue, 14) && !bitRead(ReadValue, 13)) Serial.print("MMode.LGain.CurrentChannelGain16 ");
    if (!bitRead(ReadValue, 15) && bitRead(ReadValue, 14) && bitRead(ReadValue, 13)) Serial.print("MMode.LGain.CurrentChannelGain24 ");
    Serial.println();
    if (ReadValue == 0x9422) Serial.println(">ATM 0x2B - #9422 MMode Default Value");

    // ATM Read Values

    Serial.println("-----------");

    // CalStart Status
    yield();
    Serial.print("Calibraration Status \t\t(CalStart 0x20):\t0x");
    ReadValue = eic.GetCalStartStatus();
    DisplayHEX(ReadValue, 4);
    DisplayBIN16(ReadValue);
    if (ReadValue == 0x6886) Serial.print("Power-On Value. Metering Function is Disabled");
    if (ReadValue == 0x5678) Serial.print("CALIBRATION | Meter Calibration Startup Command");
    if (ReadValue == 0x8765) Serial.print("RUNNING | Normal Metering Mode");
    if (ReadValue != 0x6886 && ReadValue != 0x5678 && ReadValue != 0x8765) Serial.print(">ATM 0x20 - Metering Function is Disabled");
    Serial.println();

    yield();
    Serial.print("UGain Calibration Value\t\t(UGain 0x31):\t\t0x");
    ReadValue = eic.GetUGain();
    DisplayHEX(ReadValue, 4);
    if (bitRead(ReadValue, 15)) Serial.print("UGain Possible Value Error");
    Serial.println();

    yield();
    Serial.print("LGain Calibration Value\t\t(LGain 0x23):\t\t0x");
    ReadValue = eic.GetLGain();
    DisplayHEX(ReadValue, 4);
    Serial.println();

    yield();
    Serial.print("IGain Calibration Value\t\t(IgainL 0x32):\t\t0x");
    ReadValue = eic.GetIGain();
    DisplayHEX(ReadValue, 4);
    Serial.println();

    // Checksum 1 Status
    yield();
    Serial.print("Checksum Status \t\t(CS1 0x2C):\t\t0x");
    ReadValue = eic.GetCS1Status();
    DisplayHEX(ReadValue, 4);
    if (ReadValue != eic.GetCS1Calculated()) {  // 0xC000
        Serial.print("*ERROR: Please update _crc1 to ATM Calculated CRC: 0x");
        Serial.print(eic.GetCS1Calculated(), HEX);
    }
    Serial.println();

    // Checksum 2 Status
    yield();
    Serial.print("Checksum Status \t\t(CS2 0x3B):\t\t0x");
    ReadValue = eic.GetCS2Status();
    DisplayHEX(ReadValue, 4);
    if (ReadValue != eic.GetCS2Calculated()) {  // 0x3000
        Serial.print("*ERROR: Please update _crc2 to ATM Calculated CRC: 0x");
        Serial.print(eic.GetCS2Calculated(), HEX);
    }
    Serial.println();

    Serial.println("-----------");

    yield();
    Serial.print("Line Voltage \t\t\t(Urms 0x49):\t\t");
    Serial.print(eic.GetLineVoltage());
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
    Serial.print("Active power \t\t\t(Pmean 0x4A):\t\t");
    Serial.print(eic.GetActivePower());
    Serial.println(" W");

    yield();
    Serial.print("Reactive power \t\t\t(Qmean 0x4B):\t\t");
    Serial.print(eic.GetReactivePower());
    Serial.println(" Var");

    yield();
    Serial.print("Apparent power \t\t\t(Smean 0x4F):\t\t");
    Serial.print(eic.GetApparentPower());
    Serial.println(" VA");

    yield();
    Serial.print("Phase Angle \t\t\t(Pangle 0x4E):\t\t");
    Serial.println(eic.GetPhaseAngle());


    yield();
    Serial.print("Import Energy \t\t\t(APenergy 0x40):\t");
    Serial.println(eic.GetImportEnergy());

    yield();
    Serial.print("Export Energy \t\t\t(ANenergy 0x41):\t");
    Serial.println(eic.GetExportEnergy());

    yield();
    Serial.print("Power Factor \t\t\t(PowerF 0x4D):\t\t");
    Serial.println(eic.GetPowerFactor());

    Serial.println("-----------");

    yield();
    Serial.print("Abs Active Energy \t\t(ATenergy 0x42):\t");
    Serial.println(eic.GetAbsActiveEnergy());

    yield();
    Serial.print("Abs Reactive Energy \t\t(Rtenergy 0x45):\t");
    Serial.println(eic.GetAbsReactiveEnergy());

    yield();
    Serial.print("Abs Reactive Forward Energy \t(RPenergy 0x43):\t");
    Serial.println(eic.GetReactivefwdEnergy());

    // LSB RMS/Power Status
    yield();
    Serial.print("LSB RMS/Power \t\t\t(LSB 0x08):\t\t0x");
    ReadValue = eic.GetLSBStatus();
    DisplayHEX(ReadValue, 4);
    Serial.println(ReadValue);

    // Other GTEM Sensors

    // ESP32 ADC 12-Bit SAR (Successive Approximation Register)
    // Conversion resolution 0 - 4095 (4096)
    // You may need to calibrate as needed.

    Serial.println("-----------");

    // DCV_IN
    yield();
    ADC_Constant = 31.340;  // Adjust as needed for calibration of VDC_IN.
    ADC_Voltage = (analogRead(DCV_IN) * ADC_Constant) / 4095;
    Serial.print("DC Voltage Sensor \t\t(DCV_IN VP):\t\t");
    Serial.print(ADC_Voltage);
    if (ADC_Voltage < 5) Serial.print(" V USB Powered.  Note - Not all ATM functions will work in this mode");
    if (ADC_Voltage > 5) Serial.print(" V AC/DC Input");
    if (ADC_Voltage > 20) Serial.print(" V *WARNING: Please Check Input Voltage.  Too High!");
    Serial.println();

    // NTC
    yield();
    int Vo;
    float R1 = 10000;  // Based on 10K
    float logR2, R2, T, Tc, Tf;
    float tCal = 1.16;  // Tweak for Calibration
    float C1 = 1.009249522e-03, C2 = 2.378405444e-04, C3 = 2.019202697e-07;

    Vo = analogRead(NTC_IN);
    R2 = R1 * (4095.0 / (float)Vo - tCal);
    logR2 = log(R2);
    T = (1.0 / (C1 + C2 * logR2 + C3 * logR2 * logR2 * logR2));
    Tc = T - 273.15;
    // Tf = (Tc * 9.0) / 3.3 + 32.0;  // Fahrenheit
    Serial.print("PCB Temperature Sensor\t\t(NTC_IN VN):\t\t");
    Serial.print(Tc);
    Serial.println(" ºC");

    Serial.println("\n");
}



void ScanI2CBus() {  // I2C Bus Scanner

    byte error, address;
    int nDevices;

    Serial.println("Scanning I2C Bus for Devices ...");
    lcd.setCursor(0, 2);
    lcd.print("Scanning I2C => ");
    nDevices = 0;
    for (address = 1; address < 127; address++) {
        Wire.beginTransmission(address);
        error = Wire.endTransmission();
        if (error == 0) {
            Serial.print("I2C device found at address Decimal ");
            if (address < 16) {
                Serial.print("0");
            }
            Serial.print(address);
            Serial.print(" = Hex 0x");
            Serial.print(address, HEX);
            if (address == 80) Serial.print(" EEPROM");
            Serial.println();

            nDevices++;
        }
        else if (error == 4) {
            Serial.print("Unknown error at address Decimal ");
            if (address < 16) {
                Serial.print("0");
            }
            Serial.print(address);
            Serial.print(" = Hex 0x");
            Serial.println(address, HEX);
        }
    }
    if (nDevices == 0) {
        Serial.println("No I2C devices found. Possible Hardware Issue?");
    }
    else {
        Serial.println("I2C Bus Scan Complete");
        lcd.print("PASS");
    }
}

void ReadEEPROM() {  // Read EEPROM Test

    lcd.setCursor(0, 3);
    lcd.print("EEPROM Test => ");
    Serial.print("EEPROM Test Read Value = 0x");
    ReadValue = EEPROM.read(0);
    Serial.print(ReadValue, HEX);
    if (ReadValue == 0x00) Serial.print(" Possible EEPROM Issue?");  // Blank New EEPROM should normally return 0xFF
    if (ReadValue == 0xFF) Serial.print(" Read OK.  Possible Blank EEPROM");
    Serial.println();

    lcd.print("PASS");

}


unsigned int SEC = 0;

// **************** 15Sec Timer ****************

void IRAM_ATTR Timer1_ISR()
{

    SEC++;



}



// **************** SETUP ****************
void setup() {

    // Stabalise
    delay(250);

    MicroID.getUniqueIDString(id);
    serial_no = id;

    DBUGS.println();
    DBUGS.println();
    ;

    DBUGS.printf("Device Serial ID =%04X", (uint16_t)(chipid >> 32));
    DBUGS.printf("%08X", (uint32_t)chipid);
    DBUGS.println();
    DBUGS.println();
    DBUGS.println("Firmware: " + FirmwareVersion);
    DBUGS.printf("Free: %d\n", ESP.getFreeHeap());
    DBUGS.println("Node type: " + node_type);


    DBUGS.println("Node name: " + node_name);




    lcd.init();

    // turn on LCD backlight
    lcd.backlight();
    lcd.noCursor();
    lcd.clear();

    lcd.setCursor(0, 0);
    // print message
    lcd.print("SM-FE102M Init...");



    // Initialise UART
    Serial.begin(115200, SERIAL_8N1);  //115200
    while (!Serial);

    Serial.println("");
    Serial.println("");

    ReadStringCmd_FLASH((uint8_t*)STR_SKETCH_REV, strlen(STR_SKETCH_REV), FALSE, FALSE);
    Serial.println(SKETCH_SM_FE102M_REV);
    Serial.println("");

    ReadStringCmd_FLASH((uint8_t*)STR_REV, strlen(STR_REV), FALSE, FALSE);
    Serial.println(FirmwareVersion);
    Serial.println("");


    Serial.println("SM-FE102M Hardware Setup and Power-Up Test");


    SetOutputPin();
    PowerOnrLedTest();
    delay(1000);


    // Initialize I2C
    Wire.begin(I2C_SDA, I2C_SCL);

    // Initialize EEPROM
    EEPROM.begin(EEPROM_SIZE);

    // Hardware Tests
    //TestRGB();

    ScanI2CBus();
    delay(1000);
    ReadEEPROM();
    delay(1000);


    dht.begin();
    delay(1000);
    Read_DHT_Data();
    delay(500);


    // Initial LED on
    led_wififlash(3000, 100);


    // Initialise the WiFi
    wifi_setup();
    Setup_Webserver();
    DBUGF("After wifi_setup: %d", ESP.getFreeHeap());
    led_wififlash(50, 50);

    delay(500);
    config_load_settings();
    // Init and get the time
    //configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
    // 
    // 
waitForSync();
    // 
    //printLocalTime();
    AUS.setPosix("AEST+1:00");
    PrintTimeStamp();


#ifdef ENABLE_WDT
    DBUGS.println("Watchdog timer is enabled.");
    feedLoopWDT();
#endif

    /*Initialise ATM90E26 + SPI port */
    //eic.InitEnergyIC();
    energy_meter_setup();
    // Stabalise
    delay(250);



    Load_Reg_CalConfiguration();
    DisplayRegisters();  // Display Registers Once

    delay(500);


    EEPROM_Update_ManufactureInfo();
    EEPROM_READ_ManufactureInfo();




    Setup_Modbus();
    



    SetupTimer1();


}


// **************** LOOP ****************
void loop() {

    events();

    if (millis() > mem_info_update) {
        mem_info_update = millis() + 2000;
        uint32_t current = ESP.getFreeHeap();
        int32_t diff = (int32_t)(last_mem - current);
        if (diff != 0) {
            DBUGS.printf("Free memory %u - diff %d %d\n", current, diff, start_mem - current);
            last_mem = current;
        }
    }

    Webserver_loop();



    wifi_loop();


    



    if (SEC = 5) {
        DBUGS.println("\n\n");
        ReadPower_Data();
        EEPROM_Update_Power_Registers();
        EEPROM_Updtate_DHT_DATA(Temp, Temp_F, Hum, hic, hif);
        EEPROM_Read_DHT_DATA();
        EEPROM_READ_Power_Registers();

        SEC = 0;
    }





    Load_Power_Registers();
    // Heatbeat LED
    //digitalWrite(LEDRun_Mode, LOW);
    //delay(50);
    //digitalWrite(LEDRun_Mode, HIGH);
    Flash_LED(1, 50, 50);







    // 
    //config_save_mqtt("https://mosquitto.org/", "/t/n/Test_Lead", "12HL", "Chamil", "Eclipse_1234");
    // Loop Delay
  //delay(LoopDelay * 1000);
   PrintTimeStamp();

    Modbus_Poling();

}

/*
String getTime() {
    return timeClient.getFormattedTime();
}

void setTimeOffset() {
    timeClient.setTimeOffset(time_offset);
}

*/

void printLocalTime() {
    struct tm timeinfo;
    if (!getLocalTime(&timeinfo)) {
        DBUGS.println("Failed to obtain time");
        return;
    }


    DBUGS.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");

}


void PrintTimeStamp() {


    DBUGS.println("Time Stamp : \t" + AUS.dateTime(RFC850));
}
