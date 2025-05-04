/*
 * FE102M Energy Monitor - Basic Example
 * 
 * This example demonstrates how to use the FE102M Energy Monitor
 * library to read and display energy measurements.
 * 
 * Hardware:
 * - SM Energy FE102M Energy Monitor
 * - Current Transformer (compatible CT clamp)
 * - 9-12V DC power supply
 * 
 * Created by: Chamil Vithanage, Microcode Embedded Solutions
 * License: MIT
 */

 #include <FE102M.h>

 // Create FE102M instance
 FE102M energyMonitor;
 
 // Timer for regular updates
 unsigned long lastUpdateTime = 0;
 const unsigned long updateInterval = 1000; // 1 second update interval
 
 // Timer for data logging
 unsigned long lastLogTime = 0;
 const unsigned long logInterval = 60000; // 1 minute logging interval
 
 void setup() {
   // Initialize serial communication
   Serial.begin(115200);
   Serial.println("FE102M Energy Monitor - Basic Example");
   
   // Initialize the energy monitor
   energyMonitor.begin();
   
   // Set operating mode to STANDALONE
   energyMonitor.setOperatingMode(STANDALONE);
   
   // Optionally, enable WiFi or MODBUS
   // energyMonitor.setupWiFi("YourSSID", "YourPassword");
   // energyMonitor.setupModbus(1, 9600);
   
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
     
     // Print data to serial
     printSerialData();
   }
   
   // Log data at specified interval
   if (millis() - lastLogTime >= logInterval) {
     lastLogTime = millis();
     
     // Save energy data to EEPROM for persistence
     energyMonitor.saveToEEPROM();
     
     Serial.println("Energy data logged to EEPROM");
   }
 }
 
 // Print energy data to serial monitor
 void printSerialData() {
   Serial.println("------ Energy Monitor Data ------");
   
   Serial.print("Voltage: ");
   Serial.print(energyMonitor.getLineVoltage(), 1);
   Serial.println(" V");
   
   Serial.print("L Line Current: ");
   Serial.print(energyMonitor.getLineCurrent(), 3);
   Serial.println(" A");
   
   Serial.print("N Line Current: ");
   Serial.print(energyMonitor.getNLineCurrent(), 3);
   Serial.println(" A");
   
   Serial.print("Active Power: ");
   Serial.print(energyMonitor.getActivePower(), 1);
   Serial.println(" W");
   
   Serial.print("Reactive Power: ");
   Serial.print(energyMonitor.getReactivePower(), 1);
   Serial.println(" VAR");
   
   Serial.print("Apparent Power: ");
   Serial.print(energyMonitor.getApparentPower(), 1);
   Serial.println(" VA");
   
   Serial.print("Power Factor: ");
   Serial.println(energyMonitor.getPowerFactor(), 3);
   
   Serial.print("Phase Angle: ");
   Serial.print(energyMonitor.getPhaseAngle(), 1);
   Serial.println(" degrees");
   
   Serial.print("Frequency: ");
   Serial.print(energyMonitor.getFrequency(), 2);
   Serial.println(" Hz");
   
   Serial.print("Import Energy: ");
   Serial.print(energyMonitor.getImportEnergy(), 3);
   Serial.println(" kWh");
   
   Serial.print("Export Energy: ");
   Serial.print(energyMonitor.getExportEnergy(), 3);
   Serial.println(" kWh");
   
   Serial.print("Temperature: ");
   Serial.print(energyMonitor.getTemperature(), 1);
   Serial.println(" °C");
   
   Serial.print("Humidity: ");
   Serial.print(energyMonitor.getHumidity(), 1);
   Serial.println(" %");
   
   Serial.print("Heat Index: ");
   Serial.print(energyMonitor.getHeatIndex(), 1);
   Serial.println(" °C");
   
   Serial.print("Operating Mode: ");
   switch(energyMonitor.getOperatingMode()) {
     case STANDALONE:
       Serial.println("Standalone");
       break;
     case WIFI_MODE:
       Serial.println("WiFi");
       break;
     case MODBUS_MODE:
       Serial.println("MODBUS");
       break;
     case DUAL_MODE:
       Serial.println("Dual (WiFi+MODBUS)");
       break;
   }
   
   Serial.println("--------------------------------");
 }