/*
 * FE102M Energy Monitor - MODBUS Example
 * 
 * This example demonstrates how to use the FE102M Energy Monitor
 * with MODBUS RTU communication protocol.
 * 
 * Hardware:
 * - SM Energy FE102M Energy Monitor
 * - Current Transformer (compatible CT clamp)
 * - 9-12V DC power supply
 * - RS485 connection to MODBUS master
 * 
 * Created by: Chamil Vithanage, Microcode Embedded Solutions
 * License: MIT
 */

 #include <FE102M.h>
 #include <ModbusRTU.h>
 
 // Create FE102M instance
 FE102M energyMonitor;
 
 // Create ModbusRTU instance
 ModbusRTU modbus;
 
 // MODBUS settings
 #define SLAVE_ID      1
 #define BAUDRATE      9600
 #define MAX485_DE_PIN 13    // GPIO13 - MAX485 DE/RE pin for MODBUS
 #define MAX485_RO_PIN 16    // GPIO16 - MAX485 RO pin
 #define MAX485_DI_PIN 17    // GPIO17 - MAX485 DI pin
 
 // Modbus register addresses
 // Input Registers (read-only)
 #define REG_VOLTAGE             0
 #define REG_LINE_CURRENT        2
 #define REG_N_LINE_CURRENT      4
 #define REG_ACTIVE_POWER        6
 #define REG_REACTIVE_POWER      8
 #define REG_APPARENT_POWER      10
 #define REG_POWER_FACTOR        12
 #define REG_FREQUENCY           14
 #define REG_PHASE_ANGLE         16
 #define REG_IMPORT_ENERGY       18
 #define REG_EXPORT_ENERGY       20
 #define REG_TEMPERATURE         22
 #define REG_HUMIDITY            24
 
 // Holding Registers (read/write)
 #define REG_CALIBRATION_START   1000
 #define REG_L_GAIN              1001
 #define REG_U_GAIN              1002
 #define REG_I_GAIN_L            1003
 #define REG_I_GAIN_N            1004
 
 // Timer for regular updates
 unsigned long lastUpdateTime = 0;
 const unsigned long updateInterval = 1000; // 1 second update interval
 
 // Pre-transmission callback for RS485
 void preTransmission() {
   digitalWrite(MAX485_DE_PIN, HIGH);
   delayMicroseconds(50);
 }
 
 // Post-transmission callback for RS485
 void postTransmission() {
   delayMicroseconds(50);
   digitalWrite(MAX485_DE_PIN, LOW);
 }
 
 void setup() {
   // Initialize serial communication
   Serial.begin(115200);
   Serial.println("FE102M Energy Monitor - MODBUS Example");
   
   // Initialize the energy monitor
   energyMonitor.begin();
   
   // Set operating mode to MODBUS_MODE
   energyMonitor.setOperatingMode(MODBUS_MODE);
   
   // Initialize MODBUS RTU
   Serial2.begin(BAUDRATE, SERIAL_8N1, MAX485_RO_PIN, MAX485_DI_PIN);
   
   // Configure ModbusRTU
   modbus.begin(&Serial2, MAX485_DE_PIN);
   modbus.setBaudrate(BAUDRATE);
   modbus.slave(SLAVE_ID);
   
   // Set callbacks for RS485 flow control
   modbus.preTransmission(preTransmission);
   modbus.postTransmission(postTransmission);
   
   // Setup MODBUS registers
   setupModbusRegisters();
   
   Serial.println("MODBUS RTU initialized as slave ID " + String(SLAVE_ID));
   Serial.println("Initialization complete!");
 }
 
 void loop() {
   // Check for button presses
   if (digitalRead(MODE_PIN) == LOW) {
     delay(50);  // Debounce
     if (digitalRead(MODE_PIN) == LOW) {
       energyMonitor.handleModeButton();
       while(digitalRead(MODE_PIN) == LOW) {
         // Wait for button release
         delay(10);
       }
     }
   }
   
   if (digitalRead(SET_PIN) == LOW) {
     delay(50);  // Debounce
     if (digitalRead(SET_PIN) == LOW) {
       energyMonitor.handleSetButton();
       while(digitalRead(SET_PIN) == LOW) {
         // Wait for button release
         delay(10);
       }
     }
   }
   
   // Update readings at regular intervals
   if (millis() - lastUpdateTime >= updateInterval) {
     lastUpdateTime = millis();
     
     // Read energy and environmental data
     energyMonitor.readEnergyData();
     energyMonitor.readEnvironmentalData();
     
     // Update the display
     energyMonitor.updateDisplay();
     
     // Update MODBUS registers with current values
     updateModbusRegisters();
     
     // Flash MODBUS LED to indicate activity
     digitalWrite(MODBUS_LED, HIGH);
     delay(50);
     digitalWrite(MODBUS_LED, LOW);
   }
   
   // Process MODBUS messages
   modbus.task();
   yield();
 }
 
 // Setup MODBUS registers
 void setupModbusRegisters() {
   // Add Input Registers (read-only)
   // Note: Each 32-bit float value requires two 16-bit registers
   modbus.addIreg(REG_VOLTAGE, 0, 2);           // Voltage (float)
   modbus.addIreg(REG_LINE_CURRENT, 0, 2);      // Line Current (float)
   modbus.addIreg(REG_N_LINE_CURRENT, 0, 2);    // N Line Current (float)
   modbus.addIreg(REG_ACTIVE_POWER, 0, 2);      // Active Power (float)
   modbus.addIreg(REG_REACTIVE_POWER, 0, 2);    // Reactive Power (float)
   modbus.addIreg(REG_APPARENT_POWER, 0, 2);    // Apparent Power (float)
   modbus.addIreg(REG_POWER_FACTOR, 0, 2);      // Power Factor (float)
   modbus.addIreg(REG_FREQUENCY, 0, 2);         // Frequency (float)
   modbus.addIreg(REG_PHASE_ANGLE, 0, 2);       // Phase Angle (float)
   modbus.addIreg(REG_IMPORT_ENERGY, 0, 2);     // Import Energy (float)
   modbus.addIreg(REG_EXPORT_ENERGY, 0, 2);     // Export Energy (float)
   modbus.addIreg(REG_TEMPERATURE, 0, 2);       // Temperature (float)
   modbus.addIreg(REG_HUMIDITY, 0, 2);          // Humidity (float)
   
   // Add Holding Registers (read/write)
   modbus.addHreg(REG_CALIBRATION_START, 0);    // Calibration Start Command
   modbus.addHreg(REG_L_GAIN, 0);               // L Line Calibration Gain
   modbus.addHreg(REG_U_GAIN, 0);               // Voltage Calibration Gain
   modbus.addHreg(REG_I_GAIN_L, 0);             // L Line Current Gain
   modbus.addHreg(REG_I_GAIN_N, 0);             // N Line Current Gain
 }
 
 // Update MODBUS registers with current readings
 void updateModbusRegisters() {
   // Helper function to store float values in two 16-bit registers
   auto storeFloat = [](float value, uint16_t address) {
     uint32_t rawValue;
     memcpy(&rawValue, &value, 4);
     
     uint16_t lowWord = rawValue & 0xFFFF;
     uint16_t highWord = (rawValue >> 16) & 0xFFFF;
     
     modbus.Ireg(address, lowWord);
     modbus.Ireg(address + 1, highWord);
   };
   
   // Update Input Registers with current values
   storeFloat(energyMonitor.getLineVoltage(), REG_VOLTAGE);
   storeFloat(energyMonitor.getLineCurrent(), REG_LINE_CURRENT);
   storeFloat(energyMonitor.getNLineCurrent(), REG_N_LINE_CURRENT);
   storeFloat(energyMonitor.getActivePower(), REG_ACTIVE_POWER);
   storeFloat(energyMonitor.getReactivePower(), REG_REACTIVE_POWER);
   storeFloat(energyMonitor.getApparentPower(), REG_APPARENT_POWER);
   storeFloat(energyMonitor.getPowerFactor(), REG_POWER_FACTOR);
   storeFloat(energyMonitor.getFrequency(), REG_FREQUENCY);
   storeFloat(energyMonitor.getPhaseAngle(), REG_PHASE_ANGLE);
   storeFloat(energyMonitor.getImportEnergy(), REG_IMPORT_ENERGY);
   storeFloat(energyMonitor.getExportEnergy(), REG_EXPORT_ENERGY);
   storeFloat(energyMonitor.getTemperature(), REG_TEMPERATURE);
   storeFloat(energyMonitor.getHumidity(), REG_HUMIDITY);
 }