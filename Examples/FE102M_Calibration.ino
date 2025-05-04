/*
  SM Energy FE102M Energy Monitor - Calibration Utility
  
  This utility allows for calibration of the FE102M Energy Monitor with ATM90E26 energy monitoring IC.
  
  Features:
  - Calibration routines for voltage, current, and power
  - Live measurements display for verification
  - EEPROM storage of calibration values
  - Serial interface for calibration control
  
  Hardware:
  - SM Energy FE102M Energy Monitor
  - ESP32 WROOM 32
  - ATM90E26 Energy Monitor IC
  - Current Transformer (compatible CT clamp)
  - 9-12V DC power supply
  
  Created by: Chamil Vithanage, Microcode Embedded Solutions
  License: MIT
*/

#include <SPI.h>
#include <Wire.h>
#include <DHT.h>
#include <LiquidCrystal_I2C.h>

// Pin Definitions - Based on FE102M pin mapping
#define CS_PIN          5     // GPIO5 - ATM90E26 SPI Chip Select
#define IRQ_PIN         39    // GPIO39 - ATM90E26 IRQ
#define WARNOUT_PIN     34    // GPIO34 - ATM90E26 WarnOut
#define CF1_PIN         35    // GPIO35 - ATM90E26 CF1
#define CF2_PIN         25    // GPIO25 - ATM90E26 CF2
#define ZX_PIN          32    // GPIO32 - ATM90E26 ZX

#define DHT_PIN         4     // GPIO4 - DHT22 Sensor Data Pin
#define MODE_PIN        33    // GPIO33 - MODE input Switch
#define SET_PIN         36    // GPIO36 - SET input Switch

#define WIFI_LED        14    // GPIO14 - WiFi Status LED
#define RUN_LED         12    // GPIO12 - Run Mode LED
#define FAULT_LED       2     // GPIO2 - Fault Status LED
#define MODBUS_LED      15    // GPIO15 - MODBUS Status LED

// EEPROM address (24C256)
#define EEPROM_ADDR 0x50

// EEPROM addresses for calibration values
#define EEPROM_UGAIN_ADDR 0
#define EEPROM_IGAIN_ADDR 2
#define EEPROM_LGAIN_ADDR 4
#define EEPROM_CRC1_ADDR  6
#define EEPROM_CRC2_ADDR  8
#define EEPROM_MAGIC_ADDR 10  // Magic number to confirm calibration data exists

// Default calibration values
#define DEFAULT_UGAIN 0xF709
#define DEFAULT_IGAIN 0x7DFB
#define DEFAULT_LGAIN 0x1D39
#define DEFAULT_CRC1  0xAE70
#define DEFAULT_CRC2  0x3D1D
#define CALIBRATION_MAGIC 0xABCD  // Magic number to confirm calibration exists

// DHT Sensor setup
#define DHTTYPE DHT22
DHT dht(DHT_PIN, DHTTYPE);

// I2C LCD setup (20x4)
LiquidCrystal_I2C lcd(0x27, 20, 4);

// ATM90E26 Register Addresses
// System Registers
#define SoftReset    0x00  // Software Reset
#define SysStatus    0x01  // System Status
#define FuncEn       0x02  // Function Enable
#define SagTh        0x03  // Voltage Sag Threshold
#define MMode        0x04  // Metering Mode Configuration
#define CSOne        0x05  // Checksum 1
#define CSTwo        0x06  // Checksum 2
#define Gain         0x07  // Measurement Gain

// Calibration Registers
#define Ugain        0x08  // Voltage RMS Gain
#define IgainL       0x09  // L Line Current Gain
#define Uoffset      0x0A  // Voltage Offset
#define IoffsetL     0x0B  // L Line Current Offset
#define PoffsetL     0x0C  // L Line Active Power Offset
#define QoffsetL     0x0D  // L Line Reactive Power Offset
#define Lgain        0x0E  // L Line Calibration Gain

// Energy Registers
#define APenergy     0x40  // Forward Active Energy
#define ANenergy     0x41  // Reverse Active Energy
#define RPenergy     0x42  // Forward Reactive Energy
#define RNenergy     0x43  // Reverse Reactive Energy
#define ATenergy     0x44  // Absolute Active Energy
#define RTenergy     0x45  // Absolute Reactive Energy

// Measurement Registers
#define Irms         0x48  // Current RMS
#define Urms         0x49  // Voltage RMS
#define Pmean        0x4A  // Mean Active Power
#define Qmean        0x4B  // Mean Reactive Power
#define Smean        0x4C  // Mean Apparent Power
#define PowerF       0x4D  // Power Factor
#define PAngle       0x4E  // Phase Angle
#define Freq         0x4F  // Line Frequency

// Configuration Registers
#define CalStart     0x20  // Calibration Start Command
#define PLconstH     0x21  // High Word of PL_Constant
#define PLconstL     0x22  // Low Word of PL_Constant
#define PStartTh     0x27  // Active Startup Power Threshold
#define QStartTh     0x28  // Reactive Startup Power Threshold
#define PNolTh       0x29  // Active No-Load Power Threshold
#define QNolTh       0x2A  // Reactive No-Load Power Threshold
#define AdjStart     0x30  // Measurement Calibration Start Command
#define LSB          0x32  // LSB Value

// Calibration values
unsigned short ugain = DEFAULT_UGAIN;
unsigned short igain = DEFAULT_IGAIN;
unsigned short lgain = DEFAULT_LGAIN;
unsigned short crc1 = DEFAULT_CRC1;
unsigned short crc2 = DEFAULT_CRC2;

// Reference values for calibration
float referenceVoltage = 230.0;  // Default reference voltage
float referenceCurrent = 1.0;    // Default reference current
float referencePower = 230.0;    // Default reference power (resistive load)

// Flag for interrupt handling
volatile bool dataReady = false;

// Timer for display updates
unsigned long lastUpdateTime = 0;
const unsigned long updateInterval = 1000; // 1 second update interval

// Calibration mode
enum CalibrationMode {
  VOLTAGE_CAL,
  CURRENT_CAL,
  POWER_CAL,
  VERIFICATION,
  SAVE_CAL
};

CalibrationMode currentMode = VOLTAGE_CAL;

// Function prototypes
void initATM90E26();
unsigned short CommEnergyIC(unsigned char RW, unsigned char address, unsigned short val);
float GetLineVoltage();
float GetLineCurrent();
float GetActivePower();
float GetFrequency();
float GetPowerFactor();
void saveCalibrationToEEPROM();
bool loadCalibrationFromEEPROM();
void writeEEPROM(int deviceAddress, unsigned int eeAddress, byte data);
byte readEEPROM(int deviceAddress, unsigned int eeAddress);
void writeWord(int deviceAddress, unsigned int eeAddress, unsigned short data);
unsigned short readWord(int deviceAddress, unsigned int eeAddress);
void printCalibrationMenu();
void handleModeButton();
void handleSetButton();
void printMeasurements();
void calibrateVoltage();
void calibrateCurrent();
void calibratePower();
void verifyCalibration();
void updateDisplay();

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  Serial.println("\n\nSM Energy FE102M Energy Monitor - Calibration Utility");
  
  // Initialize GPIO pins
  pinMode(CS_PIN, OUTPUT);
  pinMode(IRQ_PIN, INPUT);
  pinMode(WARNOUT_PIN, INPUT);
  pinMode(CF1_PIN, INPUT);
  pinMode(CF2_PIN, INPUT);
  pinMode(ZX_PIN, INPUT);
  
  pinMode(MODE_PIN, INPUT_PULLUP);  // Use internal pullup
  pinMode(SET_PIN, INPUT_PULLUP);   // Use internal pullup
  
  pinMode(WIFI_LED, OUTPUT);
  pinMode(RUN_LED, OUTPUT);
  pinMode(FAULT_LED, OUTPUT);
  pinMode(MODBUS_LED, OUTPUT);
  
  // Set default LED states
  digitalWrite(WIFI_LED, LOW);
  digitalWrite(RUN_LED, HIGH);  // Turn on RUN LED to indicate the device is running
  digitalWrite(FAULT_LED, LOW);
  digitalWrite(MODBUS_LED, LOW);
  
  // Initialize SPI for ATM90E26 communication
  SPI.begin();
  
  // Initialize DHT22 sensor
  dht.begin();
  
  // Initialize I2C and LCD
  Wire.begin();
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("FE102M Energy Monitor");
  lcd.setCursor(0, 1);
  lcd.print("Calibration Utility");
  lcd.setCursor(0, 2);
  lcd.print("Initializing...");
  
  // Try to load calibration values from EEPROM
  if (loadCalibrationFromEEPROM()) {
    Serial.println("Loaded calibration values from EEPROM");
    lcd.setCursor(0, 3);
    lcd.print("Found saved values");
  } else {
    Serial.println("Using default calibration values");
    lcd.setCursor(0, 3);
    lcd.print("Using defaults");
  }
  
  // Initialize the ATM90E26
  delay(1000);  // Give user time to read LCD
  initATM90E26();
  
  // Attach interrupt for data ready
  attachInterrupt(digitalPinToInterrupt(IRQ_PIN), dataReadyISR, FALLING);
  
  // Print the calibration menu
  printCalibrationMenu();
  
  // Display initial mode
  updateDisplay();
}

void loop() {
  // Check if data is ready to be read
  if (dataReady) {
    dataReady = false;
    printMeasurements();
  }
  
  // Update display and readings at regular intervals
  if (millis() - lastUpdateTime >= updateInterval) {
    lastUpdateTime = millis();
    updateDisplay();
  }
  
  // Check for button presses
  if (digitalRead(MODE_PIN) == LOW) {
    delay(50);  // Debounce
    if (digitalRead(MODE_PIN) == LOW) {
      handleModeButton();
      while (digitalRead(MODE_PIN) == LOW) {
        // Wait for button release
        delay(10);
      }
    }
  }
  
  if (digitalRead(SET_PIN) == LOW) {
    delay(50);  // Debounce
    if (digitalRead(SET_PIN) == LOW) {
      handleSetButton();
      while (digitalRead(SET_PIN) == LOW) {
        // Wait for button release
        delay(10);
      }
    }
  }
  
  // Check for serial commands
  if (Serial.available()) {
    char cmd = Serial.read();
    
    switch (cmd) {
      case '1':  // Voltage calibration
        currentMode = VOLTAGE_CAL;
        Serial.println("Switched to Voltage Calibration mode");
        break;
        
      case '2':  // Current calibration
        currentMode = CURRENT_CAL;
        Serial.println("Switched to Current Calibration mode");
        break;
        
      case '3':  // Power calibration
        currentMode = POWER_CAL;
        Serial.println("Switched to Power Calibration mode");
        break;
        
      case '4':  // Verification
        currentMode = VERIFICATION;
        Serial.println("Switched to Verification mode");
        break;
        
      case '5':  // Save calibration
        currentMode = SAVE_CAL;
        Serial.println("Switched to Save Calibration mode");
        break;
        
      case 'v':  // Set reference voltage
      case 'V':
        Serial.print("Enter reference voltage (V): ");
        while (!Serial.available()) {
          // Wait for input
        }
        referenceVoltage = Serial.parseFloat();
        Serial.println(referenceVoltage);
        break;
        
      case 'c':  // Set reference current
      case 'C':
        Serial.print("Enter reference current (A): ");
        while (!Serial.available()) {
          // Wait for input
        }
        referenceCurrent = Serial.parseFloat();
        Serial.println(referenceCurrent);
        break;
        
      case 'p':  // Set reference power
      case 'P':
        Serial.print("Enter reference power (W): ");
        while (!Serial.available()) {
          // Wait for input
        }
        referencePower = Serial.parseFloat();
        Serial.println(referencePower);
        break;
        
      case 's':  // Apply calibration setting for current mode
      case 'S':
        switch (currentMode) {
          case VOLTAGE_CAL:
            calibrateVoltage();
            break;
          case CURRENT_CAL:
            calibrateCurrent();
            break;
          case POWER_CAL:
            calibratePower();
            break;
          case SAVE_CAL:
            saveCalibrationToEEPROM();
            break;
          default:
            Serial.println("No calibration action for current mode");
            break;
        }
        break;
        
      case 'r':  // Reset to defaults
      case 'R':
        Serial.println("Resetting to default calibration values");
        ugain = DEFAULT_UGAIN;
        igain = DEFAULT_IGAIN;
        lgain = DEFAULT_LGAIN;
        crc1 = DEFAULT_CRC1;
        crc2 = DEFAULT_CRC2;
        initATM90E26();
        break;
        
      case 'h':  // Help
      case 'H':
      case '?':
        printCalibrationMenu();
        break;
        
      case '\n':
      case '\r':
        // Ignore newlines
        break;
        
      default:
        Serial.println("Unknown command");
        break;
    }
    
    // Clear any remaining characters in the input buffer
    while (Serial.available()) {
      Serial.read();
    }
    
    updateDisplay();
  }
}

// ISR triggered when IRQ pin goes low
void dataReadyISR() {
  dataReady = true;
}

// Initialize ATM90E26 energy monitor with current calibration values
void initATM90E26() {
  // Reset the ATM90E26
  CommEnergyIC(0, SoftReset, 0x789A);  // Software reset
  delay(100);
  
  // Configure system
  CommEnergyIC(0, FuncEn, 0x0030);  // Voltage sag irq=1, report on warnout pin=1, energy dir change irq=0
  CommEnergyIC(0, SagTh, 0x17DD);   // Voltage sag threshold
  
  // Set metering calibration values
  CommEnergyIC(0, CalStart, 0x5678);  // Metering calibration startup command. Register 21 to 2B need to be set
  
  // Configure
  CommEnergyIC(0, MMode, 0x9422);  // Metering Mode Configuration. All defaults. See pg 31 of datasheet.
  
  CommEnergyIC(0, PLconstH, 0x05CD);  // PL Constant MSB
  CommEnergyIC(0, PLconstL, 0xBB1C);  // PL Constant LSB
  
  CommEnergyIC(0, Lgain, lgain);      // Line calibration gain
  CommEnergyIC(0, Lphi, 0x0000);      // Line calibration angle
  CommEnergyIC(0, PStartTh, 0x08BD);  // Active Startup Power Threshold
  CommEnergyIC(0, PNolTh, 0x0000);    // Active No-Load Power Threshold
  CommEnergyIC(0, QStartTh, 0x0AEC);  // Reactive Startup Power Threshold
  CommEnergyIC(0, QNolTh, 0x0000);    // Reactive No-Load Power Threshold
  CommEnergyIC(0, CSOne, crc1);       // Write CSOne, as self calculated
  
  // Set measurement calibration values
  CommEnergyIC(0, AdjStart, 0x5678);  // Measurement calibration startup command, registers 31-3A
  CommEnergyIC(0, Ugain, ugain);      // Voltage rms gain
  CommEnergyIC(0, IgainL, igain);     // L line current gain
  CommEnergyIC(0, Uoffset, 0x0000);   // Voltage offset
  CommEnergyIC(0, IoffsetL, 0x0000);  // L line current offset
  CommEnergyIC(0, PoffsetL, 0x0000);  // L line active power offset
  CommEnergyIC(0, QoffsetL, 0x0000);  // L line reactive power offset
  CommEnergyIC(0, CSTwo, crc2);       // Write CSTwo, as self calculated
  
  CommEnergyIC(0, CalStart, 0x8765);  // Checks correctness of 21-2B registers and starts normal metering if ok
  CommEnergyIC(0, AdjStart, 0x8765);  // Checks correctness of 31-3A registers and starts normal measurement if ok
  
  // Check system status and update CRC values if needed
  unsigned short systemStatus = CommEnergyIC(1, SysStatus, 0xFFFF);
  Serial.println("");
  Serial.print("System Status: 0x");
  Serial.println(systemStatus, HEX);
  
  // If checksum errors, get the correct values from the IC
  if (systemStatus & 0xC000) {  // Checksum 1 Error
    crc1 = CommEnergyIC(1, CSOne, 0x0000);
    Serial.print("Checksum 1 Error! Updated CRC1 to: 0x");
    Serial.println(crc1, HEX);
    CommEnergyIC(0, CSOne, crc1);
  }
  
  if (systemStatus & 0x3000) {  // Checksum 2 Error
    crc2 = CommEnergyIC(1, CSTwo, 0x0000);
    Serial.print("Checksum 2 Error! Updated CRC2 to: 0x");
    Serial.println(crc2, HEX);
    CommEnergyIC(0, CSTwo, crc2);
  }
  
  // Initialize again with the correct checksums if there were errors
  if ((systemStatus & 0xC000) || (systemStatus & 0x3000)) {
    delay(100);
    CommEnergyIC(0, CalStart, 0x8765);  // Restart metering with correct checksums
    CommEnergyIC(0, AdjStart, 0x8765);  // Restart measurement with correct checksums
    
    // Check status again
    systemStatus = CommEnergyIC(1, SysStatus, 0xFFFF);
    Serial.print("Updated System Status: 0x");
    Serial.println(systemStatus, HEX);
  }
  
  // Turn on fault LED if still errors
  if ((systemStatus & 0xC000) || (systemStatus & 0x3000)) {
    digitalWrite(FAULT_LED, HIGH);
  } else {
    digitalWrite(FAULT_LED, LOW);
  }
}

// Communication with ATM90E26 IC
unsigned short CommEnergyIC(unsigned char RW, unsigned char address, unsigned short val) {
  unsigned char* data = (unsigned char*)&val;
  unsigned short output;
  
  // SPI interface rate is 200 to 160k bps. It Will need to be slowed down for EnergyIC
  SPISettings settings(200000, MSBFIRST, SPI_MODE3);
  
  // Switch MSB and LSB of value
  output = (val >> 8) | (val << 8);
  val = output;
  
  // Set read write flag
  address |= RW << 7;
  
  // Transmit and receive data
  SPI.beginTransaction(settings);
  
  digitalWrite(CS_PIN, LOW);
  delayMicroseconds(10);
  SPI.transfer(address);
  
  // Must wait 4 us for data to become valid
  delayMicroseconds(4);
  
  // Read data
  // Do for each byte in transfer
  if (RW) {
    for (byte i = 0; i < 2; i++) {
      *data = SPI.transfer(0x00);
      data++;
    }
  } else {
    for (byte i = 0; i < 2; i++) {
      SPI.transfer(*data);  // Write all the bytes
      data++;
    }
  }
  
  digitalWrite(CS_PIN, HIGH);
  delayMicroseconds(10);
  
  SPI.endTransaction();
  
  output = (val >> 8) | (val << 8);  // Reverse MSB and LSB
  return output;
}

// Get line voltage
float GetLineVoltage() {
  unsigned short voltage = CommEnergyIC(1, Urms, 0xFFFF);
  return (float)voltage / 100;
}

// Get line current
float GetLineCurrent() {
  unsigned short current = CommEnergyIC(1, Irms, 0xFFFF);
  return (float)current / 1000;
}

// Get active power
float GetActivePower() {
  short int apower = (short int)CommEnergyIC(1, Pmean, 0xFFFF);  // Complement, MSB is signed bit
  return (float)apower;
}

// Get frequency
float GetFrequency() {
  unsigned short freq = CommEnergyIC(1, Freq, 0xFFFF);
  return (float)freq / 100;
}

// Get power factor
float GetPowerFactor() {
  short int pf = (short int)CommEnergyIC(1, PowerF, 0xFFFF);  // MSB is signed bit
  
  // If negative
  if (pf & 0x8000) {
    pf = (pf & 0x7FFF) * -1;
  }
  
  return (float)pf / 1000;
}

// Save calibration to EEPROM
void saveCalibrationToEEPROM() {
  Serial.println("Saving calibration to EEPROM...");
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Saving Calibration");
  lcd.setCursor(0, 1);
  lcd.print("to EEPROM...");
  
  // Write calibration values
  writeWord(EEPROM_ADDR, EEPROM_UGAIN_ADDR, ugain);
  writeWord(EEPROM_ADDR, EEPROM_IGAIN_ADDR, igain);
  writeWord(EEPROM_ADDR, EEPROM_LGAIN_ADDR, lgain);
  writeWord(EEPROM_ADDR, EEPROM_CRC1_ADDR, crc1);
  writeWord(EEPROM_ADDR, EEPROM_CRC2_ADDR, crc2);
  
  // Write magic number to indicate valid calibration data
  writeWord(EEPROM_ADDR, EEPROM_MAGIC_ADDR, CALIBRATION_MAGIC);
  
  delay(500);
  
  // Verify written values
  unsigned short testUgain = readWord(EEPROM_ADDR, EEPROM_UGAIN_ADDR);
  unsigned short testIgain = readWord(EEPROM_ADDR, EEPROM_IGAIN_ADDR);
  unsigned short testLgain = readWord(EEPROM_ADDR, EEPROM_LGAIN_ADDR);
  unsigned short testMagic = readWord(EEPROM_ADDR, EEPROM_MAGIC_ADDR);
  
  if (testUgain == ugain && testIgain == igain && testLgain == lgain && testMagic == CALIBRATION_MAGIC) {
    Serial.println("Calibration saved successfully!");
    lcd.setCursor(0, 2);
    lcd.print("Save successful!");
  } else {
    Serial.println("Error: Calibration save failed!");
    lcd.setCursor(0, 2);
    lcd.print("Save FAILED!");
  }
  
  // Print saved values
  Serial.println("Saved calibration values:");
  Serial.print("UGain: 0x");
  Serial.println(ugain, HEX);
  Serial.print("IGain: 0x");
  Serial.println(igain, HEX);
  Serial.print("LGain: 0x");
  Serial.println(lgain, HEX);
  Serial.print("CRC1: 0x");
  Serial.println(crc1, HEX);
  Serial.print("CRC2: 0x");
  Serial.println(crc2, HEX);
  
  delay(2000);
  updateDisplay();
}

// Load calibration from EEPROM
bool loadCalibrationFromEEPROM() {
  // Check if valid calibration data exists by reading magic number
  unsigned short magic = readWord(EEPROM_ADDR, EEPROM_MAGIC_ADDR);
  
  if (magic != CALIBRATION_MAGIC) {
    Serial.println("No valid calibration data found in EEPROM");
    return false;
  }
  
  // Read calibration values
  ugain = readWord(EEPROM_ADDR, EEPROM_UGAIN_ADDR);
  igain = readWord(EEPROM_ADDR, EEPROM_IGAIN_ADDR);
  lgain = readWord(EEPROM_ADDR, EEPROM_LGAIN_ADDR);
  crc1 = readWord(EEPROM_ADDR, EEPROM_CRC1_ADDR);
  crc2 = readWord(EEPROM_ADDR, EEPROM_CRC2_ADDR);
  
  // Print loaded values
  Serial.println("Loaded calibration values from EEPROM:");
  Serial.print("UGain: 0x");
  Serial.println(ugain, HEX);
  Serial.print("IGain: 0x");
  Serial.println(igain, HEX);
  Serial.print("LGain: 0x");
  Serial.println(lgain, HEX);
  Serial.print("CRC1: 0x");
  Serial.println(crc1, HEX);
  Serial.print("CRC2: 0x");
  Serial.println(crc2, HEX);
  
  return true;
}

// Write a byte to EEPROM
void writeEEPROM(int deviceAddress, unsigned int eeAddress, byte data) {
  Wire.beginTransmission(deviceAddress);
  Wire.write((int)(eeAddress >> 8));   // MSB
  Wire.write((int)(eeAddress & 0xFF)); // LSB
  Wire.write(data);
  Wire.endTransmission();
  
  delay(5); // EEPROM write cycle time
}

// Read a byte from EEPROM
byte readEEPROM(int deviceAddress, unsigned int eeAddress) {
  byte rdata = 0xFF;
  
  Wire.beginTransmission(deviceAddress);
  Wire.write((int)(eeAddress >> 8));   // MSB
  Wire.write((int)(eeAddress & 0xFF)); // LSB
  Wire.endTransmission();
  
  Wire.requestFrom(deviceAddress, 1);
  
  if (Wire.available()) {
    rdata = Wire.read();
  }
  
  return rdata;
}

// Write a 16-bit word to EEPROM
void writeWord(int deviceAddress, unsigned int eeAddress, unsigned short data) {
  writeEEPROM(deviceAddress, eeAddress, (byte)(data >> 8));     // MSB
  writeEEPROM(deviceAddress, eeAddress + 1, (byte)(data & 0xFF)); // LSB
}

// Read a 16-bit word from EEPROM
unsigned short readWord(int deviceAddress, unsigned int eeAddress) {
  unsigned short rdata = 0;
  
  rdata = readEEPROM(deviceAddress, eeAddress) << 8; // MSB
  rdata |= readEEPROM(deviceAddress, eeAddress + 1); // LSB
  
  return rdata;
}

// Print the calibration menu
void printCalibrationMenu() {
  Serial.println("\n------ SM Energy FE102M Calibration Menu ------");
  Serial.println("1 - Voltage Calibration Mode");
  Serial.println("2 - Current Calibration Mode");
  Serial.println("3 - Power Calibration Mode");
  Serial.println("4 - Verification Mode");
  Serial.println("5 - Save Calibration to EEPROM");
  Serial.println("");
  Serial.println("v - Set Reference Voltage");
  Serial.println("c - Set Reference Current");
  Serial.println("p - Set Reference Power");
  Serial.println("s - Apply Calibration for Current Mode");
  Serial.println("r - Reset to Default Calibration");
  Serial.println("h - Help (this menu)");
  Serial.println("");
  Serial.println("Current reference values:");
  Serial.print("Voltage: ");
  Serial.print(referenceVoltage);
  Serial.println(" V");
  Serial.print("Current: ");
  Serial.print(referenceCurrent);
  Serial.println(" A");
  Serial.print("Power: ");
  Serial.print(referencePower);
  Serial.println(" W");
  Serial.println("----------------------------------------------");
}

// Handle MODE button press
void handleModeButton() {
  // Change the calibration mode
  currentMode = (CalibrationMode)((currentMode + 1) % 5);
  
  // Update display
  updateDisplay();
  
  // Print the current mode to serial
  Serial.print("Changed to mode: ");
  switch (currentMode) {
    case VOLTAGE_CAL:
      Serial.println("Voltage Calibration");
      break;
    case CURRENT_CAL:
      Serial.println("Current Calibration");
      break;
    case POWER_CAL:
      Serial.println("Power Calibration");
      break;
    case VERIFICATION:
      Serial.println("Verification");
      break;
    case SAVE_CAL:
      Serial.println("Save Calibration");
      break;
  }
}

// Handle SET button press
void handleSetButton() {
  // Perform action based on current mode
  switch (currentMode) {
    case VOLTAGE_CAL:
      calibrateVoltage();
      break;
    case CURRENT_CAL:
      calibrateCurrent();
      break;
    case POWER_CAL:
      calibratePower();
      break;
    case VERIFICATION:
      verifyCalibration();
      break;
    case SAVE_CAL:
      saveCalibrationToEEPROM();
      break;
  }
}

// Print current measurements to serial and LCD
void printMeasurements() {
  // Read energy parameters
  float voltage = GetLineVoltage();
  float current = GetLineCurrent();
  float power = GetActivePower();
  float frequency = GetFrequency();
  float pf = GetPowerFactor();
  
  // Print to serial
  Serial.println("\n--- Current Measurements ---");
  Serial.print("Voltage: ");
  Serial.print(voltage, 2);
  Serial.println(" V");
  
  Serial.print("Current: ");
  Serial.print(current, 3);
  Serial.println(" A");
  
  Serial.print("Active Power: ");
  Serial.print(power, 2);
  Serial.println(" W");
  
  Serial.print("Frequency: ");
  Serial.print(frequency, 2);
  Serial.println(" Hz");
  
  Serial.print("Power Factor: ");
  Serial.println(pf, 3);
  
  // Print calibration values
  Serial.println("\n--- Current Calibration Values ---");
  Serial.print("UGain: 0x");
  Serial.println(ugain, HEX);
  Serial.print("IGain: 0x");
  Serial.println(igain, HEX);
  Serial.print("LGain: 0x");
  Serial.println(lgain, HEX);
  Serial.print("CRC1: 0x");
  Serial.println(crc1, HEX);
  Serial.print("CRC2: 0x");
  Serial.println(crc2, HEX);
  
  Serial.println("-------------------------------");
}

// Calibrate voltage
void calibrateVoltage() {
  // Read the current voltage
  float measuredVoltage = GetLineVoltage();
  
  // Calculate new gain value based on reference voltage
  unsigned short newUgain;
  
  // If measured is non-zero, calculate proper gain adjustment
  if (measuredVoltage > 0) {
    newUgain = (unsigned short)((float)ugain * referenceVoltage / measuredVoltage);
  } else {
    Serial.println("Error: Measured voltage is zero or negative!");
    return;
  }
  
  // Limit to valid range
  if (newUgain < 0x0000) newUgain = 0x0000;
  if (newUgain > 0xFFFF) newUgain = 0xFFFF;
  
  // Print calibration information
  Serial.println("\n--- Voltage Calibration ---");
  Serial.print("Reference Voltage: ");
  Serial.print(referenceVoltage, 2);
  Serial.println(" V");
  Serial.print("Measured Voltage: ");
  Serial.print(measuredVoltage, 2);
  Serial.println(" V");
  Serial.print("Old UGain: 0x");
  Serial.println(ugain, HEX);
  Serial.print("New UGain: 0x");
  Serial.println(newUgain, HEX);
  
  // Update LCD
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Voltage Calibration");
  lcd.setCursor(0, 1);
  lcd.print("Ref: ");
  lcd.print(referenceVoltage, 1);
  lcd.print("V Meas: ");
  lcd.print(measuredVoltage, 1);
  lcd.print("V");
  lcd.setCursor(0, 2);
  lcd.print("Old: 0x");
  lcd.print(ugain, HEX);
  lcd.setCursor(0, 3);
  lcd.print("New: 0x");
  lcd.print(newUgain, HEX);
  
  // Ask for confirmation
  Serial.println("\nApply new voltage calibration? (y/n)");
  
  // Wait for user input
  while (!Serial.available()) {
    // Check for SET button press as alternative confirmation
    if (digitalRead(SET_PIN) == LOW) {
      delay(50);  // Debounce
      if (digitalRead(SET_PIN) == LOW) {
        while (digitalRead(SET_PIN) == LOW) {
          // Wait for button release
          delay(10);
        }
        Serial.println("Confirmed with SET button");
        break;
      }
    }
    delay(10);
  }
  
  char response = 'n';
  if (Serial.available()) {
    response = Serial.read();
  } else {
    response = 'y';  // SET button was pressed
  }
  
  // Clear any remaining characters in the input buffer
  while (Serial.available()) {
    Serial.read();
  }
  
  if (response == 'y' || response == 'Y') {
    Serial.println("Applying new voltage calibration...");
    
    // Save the new gain value
    ugain = newUgain;
    
    // Apply the new calibration
    CommEnergyIC(0, Ugain, ugain);
    
    // System status may need update after changing calibration
    unsigned short systemStatus = CommEnergyIC(1, SysStatus, 0xFFFF);
    if (systemStatus & 0x3000) {  // Checksum 2 Error
      crc2 = CommEnergyIC(1, CSTwo, 0x0000);
      Serial.print("Updated CRC2 to: 0x");
      Serial.println(crc2, HEX);
      CommEnergyIC(0, CSTwo, crc2);
    }
    
    Serial.println("Voltage calibration complete!");
  } else {
    Serial.println("Voltage calibration cancelled.");
  }
  
  delay(2000);  // Give time to read the display
  updateDisplay();
}

// Calibrate current
void calibrateCurrent() {
  // Read the current
  float measuredCurrent = GetLineCurrent();
  
  // Calculate new gain value based on reference current
  unsigned short newIgain;
  
  // If measured is non-zero, calculate proper gain adjustment
  if (measuredCurrent > 0) {
    newIgain = (unsigned short)((float)igain * referenceCurrent / measuredCurrent);
  } else {
    Serial.println("Error: Measured current is zero or negative!");
    return;
  }
  
  // Limit to valid range
  if (newIgain < 0x0000) newIgain = 0x0000;
  if (newIgain > 0xFFFF) newIgain = 0xFFFF;
  
  // Print calibration information
  Serial.println("\n--- Current Calibration ---");
  Serial.print("Reference Current: ");
  Serial.print(referenceCurrent, 3);
  Serial.println(" A");
  Serial.print("Measured Current: ");
  Serial.print(measuredCurrent, 3);
  Serial.println(" A");
  Serial.print("Old IGain: 0x");
  Serial.println(igain, HEX);
  Serial.print("New IGain: 0x");
  Serial.println(newIgain, HEX);
  
  // Update LCD
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Current Calibration");
  lcd.setCursor(0, 1);
  lcd.print("Ref: ");
  lcd.print(referenceCurrent, 3);
  lcd.print("A Meas: ");
  lcd.print(measuredCurrent, 3);
  lcd.print("A");
  lcd.setCursor(0, 2);
  lcd.print("Old: 0x");
  lcd.print(igain, HEX);
  lcd.setCursor(0, 3);
  lcd.print("New: 0x");
  lcd.print(newIgain, HEX);
  
  // Ask for confirmation
  Serial.println("\nApply new current calibration? (y/n)");
  
  // Wait for user input
  while (!Serial.available()) {
    // Check for SET button press as alternative confirmation
    if (digitalRead(SET_PIN) == LOW) {
      delay(50);  // Debounce
      if (digitalRead(SET_PIN) == LOW) {
        while (digitalRead(SET_PIN) == LOW) {
          // Wait for button release
          delay(10);
        }
        Serial.println("Confirmed with SET button");
        break;
      }
    }
    delay(10);
  }
  
  char response = 'n';
  if (Serial.available()) {
    response = Serial.read();
  } else {
    response = 'y';  // SET button was pressed
  }
  
  // Clear any remaining characters in the input buffer
  while (Serial.available()) {
    Serial.read();
  }
  
  if (response == 'y' || response == 'Y') {
    Serial.println("Applying new current calibration...");
    
    // Save the new gain value
    igain = newIgain;
    
    // Apply the new calibration
    CommEnergyIC(0, IgainL, igain);
    
    // System status may need update after changing calibration
    unsigned short systemStatus = CommEnergyIC(1, SysStatus, 0xFFFF);
    if (systemStatus & 0x3000) {  // Checksum 2 Error
      crc2 = CommEnergyIC(1, CSTwo, 0x0000);
      Serial.print("Updated CRC2 to: 0x");
      Serial.println(crc2, HEX);
      CommEnergyIC(0, CSTwo, crc2);
    }
    
    Serial.println("Current calibration complete!");
  } else {
    Serial.println("Current calibration cancelled.");
  }
  
  delay(2000);  // Give time to read the display
  updateDisplay();
}

// Calibrate power
void calibratePower() {
  // Read the power
  float measuredPower = GetActivePower();
  
  // Calculate new gain value based on reference power
  unsigned short newLgain;
  
  // If measured is non-zero, calculate proper gain adjustment
  if (measuredPower != 0) {
    newLgain = (unsigned short)((float)lgain * referencePower / measuredPower);
  } else {
    Serial.println("Error: Measured power is zero!");
    return;
  }
  
  // Limit to valid range
  if (newLgain < 0x0000) newLgain = 0x0000;
  if (newLgain > 0xFFFF) newLgain = 0xFFFF;
  
  // Print calibration information
  Serial.println("\n--- Power Calibration ---");
  Serial.print("Reference Power: ");
  Serial.print(referencePower, 2);
  Serial.println(" W");
  Serial.print("Measured Power: ");
  Serial.print(measuredPower, 2);
  Serial.println(" W");
  Serial.print("Old LGain: 0x");
  Serial.println(lgain, HEX);
  Serial.print("New LGain: 0x");
  Serial.println(newLgain, HEX);
  
  // Update LCD
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Power Calibration");
  lcd.setCursor(0, 1);
  lcd.print("Ref: ");
  lcd.print(referencePower, 1);
  lcd.print("W Meas: ");
  lcd.print(measuredPower, 1);
  lcd.print("W");
  lcd.setCursor(0, 2);
  lcd.print("Old: 0x");
  lcd.print(lgain, HEX);
  lcd.setCursor(0, 3);
  lcd.print("New: 0x");
  lcd.print(newLgain, HEX);
  
  // Ask for confirmation
  Serial.println("\nApply new power calibration? (y/n)");
  
  // Wait for user input
  while (!Serial.available()) {
    // Check for SET button press as alternative confirmation
    if (digitalRead(SET_PIN) == LOW) {
      delay(50);  // Debounce
      if (digitalRead(SET_PIN) == LOW) {
        while (digitalRead(SET_PIN) == LOW) {
          // Wait for button release
          delay(10);
        }
        Serial.println("Confirmed with SET button");
        break;
      }
    }
    delay(10);
  }
  
  char response = 'n';
  if (Serial.available()) {
    response = Serial.read();
  } else {
    response = 'y';  // SET button was pressed
  }
  
  // Clear any remaining characters in the input buffer
  while (Serial.available()) {
    Serial.read();
  }
  
  if (response == 'y' || response == 'Y') {
    Serial.println("Applying new power calibration...");
    
    // Save the new gain value
    lgain = newLgain;
    
    // Apply the new calibration
    CommEnergyIC(0, Lgain, lgain);
    
    // System status may need update after changing calibration
    unsigned short systemStatus = CommEnergyIC(1, SysStatus, 0xFFFF);
    if (systemStatus & 0xC000) {  // Checksum 1 Error
      crc1 = CommEnergyIC(1, CSOne, 0x0000);
      Serial.print("Updated CRC1 to: 0x");
      Serial.println(crc1, HEX);
      CommEnergyIC(0, CSOne, crc1);
    }
    
    Serial.println("Power calibration complete!");
  } else {
    Serial.println("Power calibration cancelled.");
  }
  
  delay(2000);  // Give time to read the display
  updateDisplay();
}

// Verify calibration with current measurements
void verifyCalibration() {
  // Read all parameters
  float voltage = GetLineVoltage();
  float current = GetLineCurrent();
  float power = GetActivePower();
  float frequency = GetFrequency();
  float pf = GetPowerFactor();
  
  // Calculate expected power (for simple resistive loads)
  float expectedPower = voltage * current;
  float powerError = abs(power - expectedPower) / expectedPower * 100.0;
  
  // Display verification results
  Serial.println("\n--- Calibration Verification ---");
  Serial.print("Voltage: ");
  Serial.print(voltage, 2);
  Serial.print(" V (Reference: ");
  Serial.print(referenceVoltage, 2);
  Serial.println(" V)");
  
  Serial.print("Current: ");
  Serial.print(current, 3);
  Serial.print(" A (Reference: ");
  Serial.print(referenceCurrent, 3);
  Serial.println(" A)");
  
  Serial.print("Power: ");
  Serial.print(power, 2);
  Serial.print(" W (Reference: ");
  Serial.print(referencePower, 2);
  Serial.println(" W)");
  
  Serial.print("Frequency: ");
  Serial.print(frequency, 2);
  Serial.println(" Hz");
  
  Serial.print("Power Factor: ");
  Serial.println(pf, 3);
  
  Serial.print("Calculated Power (V*I): ");
  Serial.print(expectedPower, 2);
  Serial.println(" W");
  
  Serial.print("Power Error: ");
  Serial.print(powerError, 2);
  Serial.println(" %");
  
  // Update LCD with verification info
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Verification");
  lcd.setCursor(0, 1);
  lcd.print("V:");
  lcd.print(voltage, 1);
  lcd.print("V  I:");
  lcd.print(current, 3);
  lcd.print("A");
  lcd.setCursor(0, 2);
  lcd.print("P:");
  lcd.print(power, 1);
  lcd.print("W  PF:");
  lcd.print(pf, 2);
  lcd.setCursor(0, 3);
  lcd.print("Error: ");
  lcd.print(powerError, 1);
  lcd.print("%");
  
  // Show additional information in serial
  Serial.println("\nCalibration values:");
  Serial.print("UGain: 0x");
  Serial.println(ugain, HEX);
  Serial.print("IGain: 0x");
  Serial.println(igain, HEX);
  Serial.print("LGain: 0x");
  Serial.println(lgain, HEX);
  
  delay(5000);  // Display verification for 5 seconds
  updateDisplay();
}

// Update the LCD display based on current mode
void updateDisplay() {
  // Read current values
  float voltage = GetLineVoltage();
  float current = GetLineCurrent();
  float power = GetActivePower();
  float frequency = GetFrequency();
  float pf = GetPowerFactor();
  
  // Update LCD based on current mode
  lcd.clear();
  
  // Line 1 - Mode
  lcd.setCursor(0, 0);
  lcd.print("Mode: ");
  switch (currentMode) {
    case VOLTAGE_CAL:
      lcd.print("Voltage Cal");
      break;
    case CURRENT_CAL:
      lcd.print("Current Cal");
      break;
    case POWER_CAL:
      lcd.print("Power Cal");
      break;
    case VERIFICATION:
      lcd.print("Verification");
      break;
    case SAVE_CAL:
      lcd.print("Save Calibration");
      break;
  }
  
  // Line 2 - Measurements part 1
  lcd.setCursor(0, 1);
  lcd.print("V:");
  lcd.print(voltage, 1);
  lcd.print("V  I:");
  lcd.print(current, 3);
  lcd.print("A");
  
  // Line 3 - Measurements part 2
  lcd.setCursor(0, 2);
  lcd.print("P:");
  lcd.print(power, 1);
  lcd.print("W  F:");
  lcd.print(frequency, 1);
  lcd.print("Hz");
  
  // Line 4 - Current gain values based on mode
  lcd.setCursor(0, 3);
  switch (currentMode) {
    case VOLTAGE_CAL:
      lcd.print("UGain: 0x");
      lcd.print(ugain, HEX);
      break;
    case CURRENT_CAL:
      lcd.print("IGain: 0x");
      lcd.print(igain, HEX);
      break;
    case POWER_CAL:
      lcd.print("LGain: 0x");
      lcd.print(lgain, HEX);
      break;
    case VERIFICATION:
      lcd.print("PF:");
      lcd.print(pf, 2);
      break;
    case SAVE_CAL:
      lcd.print("Press SET to save");
      break;
  }
}
