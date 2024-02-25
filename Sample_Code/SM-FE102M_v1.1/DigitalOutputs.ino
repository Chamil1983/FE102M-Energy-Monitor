#include <Arduino.h>


#ifdef LEDRun_Mode
#ifndef RUN_LED_ON_STATE
#define RUN_LED_ON_STATE LOW

#ifndef RUN_LED_OFF_STATE
#define RUN_LED_OFF_STATE HIGH

#endif
#endif
#endif

#ifndef MODBUS_LED_ON_STATE
#define MODBUS_LED_ON_STATE LOW

#ifndef MODBUS_LED_OFF_STATE
#define MODBUS_LED_OFF_STATE HIGH

#endif


#endif



#ifndef WIFI_LED_ON_STATE
#define WIFI_LED_ON_STATE LOW
#endif


void SetOutputPin(void) {
    uint8_t Count = 0;
    OutputPin[0] = LEDWiFI_State;
    OutputPin[1] = LEDRun_Mode;
    OutputPin[2] = LED_Fault;
    OutputPin[3] = LEDModbus_Status;

    do {
        pinMode(OutputPin[Count], OUTPUT);
        digitalWrite(OutputPin[Count], HIGH);
    } while (++Count < sizeof(OutputPin));
}

//======================================================================
//  Function used to test Buzzer and leds
void PowerOnrLedTest(void) {
    uint8_t Count = 0;
    lcd.setCursor(0, 1);

    lcd.print("Testing LEDS => ");

    do {
        OnOff_Output(Count, OUTPUT_ON);
        delay(500);

    } while (++Count < sizeof(OutputPin));
    Count = 0;
    delay(2000);
    do {
        OnOff_Output(Count, OUTPUT_OFF);
        delay(500);
    } while (++Count < sizeof(OutputPin));

    for (int i = 0; i < 4; i++) {
        Count = 0;
        do {
            OnOff_Output(Count, OUTPUT_ON);

        } while (++Count < sizeof(OutputPin));

        delay(300);
        Count = 0;

        do {
            OnOff_Output(Count, OUTPUT_OFF);

        } while (++Count < sizeof(OutputPin));

        delay(500);


    }

    delay(2000);
    lcd.print("PASS");

}
//======================================================================

//======================================================================
//  Function used to SET or RESET digital output
//  If "OnOff" = 0 -> Output OFF
//  If "OnOff" = 1 -> Output ON
//  ----------------------------------------------
//  If "SelectOutput" = 0 -> Relay 1
//  If "SelectOutput" = 1 -> Relay 2
//  If "SelectOutput" = 2 -> Buzzer
//  If "SelectOutput" = 3 -> Led 1

void OnOff_Output(uint8_t SelectOutput, uint8_t OnOff) {
    if (OnOff != 1) {
        digitalWrite(OutputPin[SelectOutput], LOW);
        LastStateOutputPin[SelectOutput] = OUTPUT_STATE_ON;
    }
    else {
        digitalWrite(OutputPin[SelectOutput], HIGH);
        LastStateOutputPin[SelectOutput] = OUTPUT_STATE_OFF;
    }
}
//======================================================================

//======================================================================
//  Function used to TOGGLE digital output
//  If "OnOff" = 0 -> Output OFF
//  If "OnOff" = 1 -> Output ON
//  ----------------------------------------------
//  If "SelectOutput" = 0 -> Relay 1
//  If "SelectOutput" = 1 -> Relay 2
//  If "SelectOutput" = 2 -> Buzzer
//  If "SelectOutput" = 3 -> Led 1
//  If "SelectOutput" = 4 -> Led 2
//  If "SelectOutput" = 5 -> Led 3 
//  If "SelectOutput" = 6 -> Led 4
void Toggle_Output(uint8_t SelectOutput) {
    if (LastStateOutputPin[SelectOutput] == OUTPUT_STATE_ON) {
        digitalWrite(OutputPin[SelectOutput], HIGH);
        LastStateOutputPin[SelectOutput] = OUTPUT_STATE_OFF;
    }
    else if (LastStateOutputPin[SelectOutput] == OUTPUT_STATE_OFF) {
        digitalWrite(OutputPin[SelectOutput], LOW);
        LastStateOutputPin[SelectOutput] = OUTPUT_STATE_ON;
    }
}
//======================================================================

void led_wififlash(int ton, int toff) {
    digitalWrite(WIFI_LED, WIFI_LED_ON_STATE);
    delay(ton);
    digitalWrite(WIFI_LED, WIFI_LED_ON_STATE);
    delay(toff);
}

void led_runflash(int ton, int toff) {
    digitalWrite(LEDRun_Mode, RUN_LED_ON_STATE);
    delay(ton);
    digitalWrite(LEDRun_Mode, RUN_LED_OFF_STATE);
    delay(toff);
}

void Flash_LED(uint8_t SelectOutput, int ton, int toff) {
    digitalWrite(OutputPin[SelectOutput], OUTPUT_ON);
    delay(ton);
    digitalWrite(OutputPin[SelectOutput], OUTPUT_OFF);
    delay(toff);
}

void led_ModbusFlash(int ton, int toff) {
    digitalWrite(LEDModbus_Status, MODBUS_LED_ON_STATE);
    delay(ton);
    digitalWrite(LEDModbus_Status, MODBUS_LED_OFF_STATE);
    delay(toff);
}
