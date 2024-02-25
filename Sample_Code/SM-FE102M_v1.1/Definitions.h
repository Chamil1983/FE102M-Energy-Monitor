
#include <Arduino.h>
#include <LiquidCrystal_I2C.h>


// ****************  VARIABLES / DEFINES / STATIC / STRUCTURES ****************

//  Directives used to manage Output
#define LEDWiFI_State             14      //  LED_Blue     mapped on the I/O 14 (Pin 13)
#define LEDRun_Mode               12      //  LED_Green    mapped on the I/O 12 (Pin 14)
#define LED_Fault                  2      //  LED_Red       mapped on the I/O 2 (Pin 24)
#define LEDModbus_Status          15      //  LED_White    mapped on the I/O 15 (Pin 23)

#define OUTPUT_STATE_OFF           0
#define OUTPUT_STATE_ON            1
#define OUTPUT_STATE_TOGGLE        2
#define OUTPUT_STATE_TOGGLE_WAIT   3
#define OUTPUT_STATE_TOGGLE_ONLY   4
#define OUTPUT_STATE_NOTHING       255


#define SELECT_LEDBLUE              14
#define SELECT_LEDGREEN             12
#define SELECT_LEDRED               2
#define SELECT_LEDWHITE             15

#define OUTPUT_OFF                 1
#define OUTPUT_ON                  0


hw_timer_t* Timer1_Cfg = NULL;



//const int maxChar = 12; //WARNING DO NOT CHANGE THIS!

// Variables
unsigned short ReadValue;
float ADC_Voltage;
float ADC_Constant;

// Constants
const int LoopDelay = 1;  // Loop Delay in Seconds
uint64_t chipid = ESP.getEfuseMac();


// set the LCD number of columns and rows
int lcdColumns = 20;
int lcdRows = 4;

// set LCD address, number of columns and rows
LiquidCrystal_I2C lcd(0x27, lcdColumns, lcdRows);


// **************** INPUTS ****************
#define DCV_IN 36  //GPIO 36 (Analog VP / ADC 1 CH0)
#define NTC_IN 39  //GPIO 39/VN (Analog ADC 1 CH3)


// Define I2C (Expansion Port)
#define I2C_SDA 21
#define I2C_SCL 22

// EEPROM
#define EEPROM_SIZE 1

// Define Logic Types
#define TRUE  0
#define FALSE 1


#define SKETCH_SM_FE102M_REV                 "1.1"
String FirmwareVersion = "SM-FE102M 1.1V";

String HardwareRev= "SM-FE102M_hware 1.1V";

	

#define POWER_THR_OFFSET           100    //  Power Threshold offset. This parameter is used to manage leds visualization. If power measure is less then
                                          //  threshold minus offset the led green is on. Then if the power measure is more then threshold 
                                          //  minus offset the led yellow is on. Finally if the power measure is more then threshold the led red is on and buzzer is active
#define POWER_THR_DEFAULT          500    //  Power Threshold 500W
//--------------------------------------------

//--------------------------------------------
#define Mode_Input                32    //  Mode Input mapped on the I/O 32 (Pin 8)
#define Set_Input                 33    //  Set Input mapped on the I/O 33 (Pin 9)

#define LEVEL_HIGH_INPUT          0
#define LEVEL_LOW_INPUT           1
#define TOGGLE_INPUT              2

#define INPUT_STATE_IDLE          0          
#define INPUT_STATE_WAIT          1
//--------------------------------------------


//======================================================================
//  CONST STRING
const char STR_SKETCH_REV[]               PROGMEM = "SM-FE102M Sketch Rev: ";
const char STR_REV[]                      PROGMEM = "SM-FE102M Device Rev: ";
const char STR_NEGATIVE_ACTIVE_POWER[]    PROGMEM = "Active power is negative (export) and is in quadrants 2,3\n";
const char STR_POSITIVE_ACTIVE_POWER[]    PROGMEM = "Active power is positive (import) and is in quadrants 1,4\n";
const char STR_NEGATIVE_REACTIVE_POWER[]  PROGMEM = "Reactive power is negative (capacitive load) and is in quadrants 3,4\n";
const char STR_POSITIVE_REACTIVE_POWER[]  PROGMEM = "Reactive power is positive and is in quadrants 1,2\n";
const char STR_ZCD[]                      PROGMEM = "Zero Cross Detected!\n";
const char STR_ZCD_ERROR[]                PROGMEM = "Zero Cross Failure!\n";
const char STR_VOLTAGE[]                  PROGMEM = "Instant Value Vrms: ";
const char STR_CURRENT[]                  PROGMEM = "Instant Value Irms: ";
const char STR_FREQUENCY[]                PROGMEM = "Instant Value Freq: ";
const char STR_POWER_FACTOR[]             PROGMEM = "Instant Value PF:   ";
const char STR_IMPORT_ENERGY[]            PROGMEM = "Instant Value EngImp:    ";
const char STR_ACTIVE_POWER[]             PROGMEM = "Instant Value P:    ";
const char STR_EXPORT_ENERGY[]            PROGMEM = "Instant Value EngExp:    ";

#ifdef ENABLE_AVERAGE_CALC 
const char STR_AVERAGE_VOLTAGE[]          PROGMEM = "Average Value Vrms: ";
const char STR_AVERAGE_CURRENT[]          PROGMEM = "Average Value Irms: ";
const char STR_AVERAGE_FREQUENCY[]        PROGMEM = "Average Value Freq: ";
const char STR_AVERAGE_POWER_FACTOR[]     PROGMEM = "Average Value PF:   ";
const char STR_AVERAGE_IMPORT_ENERGY[]    PROGMEM = "Average Value EngImp:    ";
const char STR_AVERAGE_ACTIVE_POWER[]     PROGMEM = "Average Value P:    ";
const char STR_AVERAGE_EXPORT_ENERGY[]    PROGMEM = "Average Value EngExp:    ";
#endif

//======================================================================


static uint32_t last_mem = 0;
static uint32_t start_mem = 0;
static unsigned long mem_info_update = 0;



//======================================================================
//
uint8_t OutputPin[4];
uint8_t LastStateOutputPin[4];
uint8_t InputPin[2];

union DigitalInput {
    struct {
        uint8_t  InputStatus : 1;
        uint8_t  InputReaded : 1;
        uint8_t  InputVar : 1;
        uint8_t  LastInputStatus : 1;
        uint8_t  Free : 4;
    } Bit;
} DigitalInput[2];
//======================================================================


#define DHTPIN 4     // Digital pin connected to the DHT sensor

#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321

bool enableCors = true;

// Event timeouts
unsigned long wifiRestartTime = 0;
unsigned long mqttRestartTime = 0;
unsigned long systemRestartTime = 0;
unsigned long systemRebootTime = 0;

static const char _DUMMY_PASSWORD[] PROGMEM = "_DUMMY_PASSWORD";
#define DUMMY_PASSWORD FPSTR(_DUMMY_PASSWORD)

#define TEXTIFY(A) #A
#define ESCAPEQUOTE(A) TEXTIFY(A)
String currentfirmware = "2.6"; //ESCAPEQUOTE(BUILD_TAG);


char Ip_Add[40];







