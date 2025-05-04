/*
 * FE102M Energy Monitor - WiFi Example
 * 
 * This example demonstrates how to use the FE102M Energy Monitor
 * with WiFi connectivity and a web server for remote monitoring.
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
 #include <WiFi.h>
 #include <AsyncTCP.h>
 #include <ESPAsyncWebServer.h>
 #include <SPIFFS.h>
 #include <ArduinoJson.h>
 
 // Create FE102M instance
 FE102M energyMonitor;
 
 // WiFi credentials
 const char* ssid = "YourWiFiSSID";
 const char* password = "YourWiFiPassword";
 
 // Web server
 AsyncWebServer server(80);
 
 // Timer for regular updates
 unsigned long lastUpdateTime = 0;
 const unsigned long updateInterval = 1000; // 1 second update interval
 
 // Timer for data logging
 unsigned long lastLogTime = 0;
 const unsigned long logInterval = 60000; // 1 minute logging interval
 
 void setup() {
   // Initialize serial communication
   Serial.begin(115200);
   Serial.println("FE102M Energy Monitor - WiFi Example");
   
   // Initialize SPIFFS for web server files
   if(!SPIFFS.begin(true)) {
     Serial.println("An error occurred while mounting SPIFFS");
     return;
   }
   
   // Initialize the energy monitor
   energyMonitor.begin();
   
   // Set operating mode to WiFi_MODE
   energyMonitor.setOperatingMode(WIFI_MODE);
   
   // Connect to WiFi
   setupWiFi();
   
   // Setup web server
   setupWebServer();
   
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
   }
   
   // Log data at specified interval
   if (millis() - lastLogTime >= logInterval) {
     lastLogTime = millis();
     
     // Save energy data to EEPROM for persistence
     energyMonitor.saveToEEPROM();
     
     Serial.println("Energy data logged to EEPROM");
   }
   
   // Maintain WiFi connection
   if(WiFi.status() != WL_CONNECTED) {
     digitalWrite(WIFI_LED, LOW); // Turn off WiFi LED
     reconnectWiFi();
   } else {
     digitalWrite(WIFI_LED, HIGH); // Turn on WiFi LED
   }
 }
 
 // Connect to WiFi
 void setupWiFi() {
   WiFi.mode(WIFI_STA);
   WiFi.begin(ssid, password);
   
   Serial.println("Connecting to WiFi...");
   
   int attempts = 0;
   while (WiFi.status() != WL_CONNECTED && attempts < 20) {
     digitalWrite(WIFI_LED, !digitalRead(WIFI_LED)); // Toggle LED
     delay(500);
     Serial.print(".");
     attempts++;
   }
   
   if(WiFi.status() == WL_CONNECTED) {
     Serial.println("\nWiFi connected");
     Serial.print("IP address: ");
     Serial.println(WiFi.localIP());
     digitalWrite(WIFI_LED, HIGH); // Turn on WiFi LED
   } else {
     Serial.println("\nWiFi connection failed");
     Serial.println("Starting Access Point mode...");
     startAccessPoint();
   }
 }
 
 // Start Access Point if WiFi connection fails
 void startAccessPoint() {
   WiFi.disconnect();
   delay(100);
   
   // Create a unique AP name with the chip ID
   String ap_ssid = "FE102M_" + String((uint32_t)ESP.getEfuseMac());
   
   WiFi.softAP(ap_ssid.c_str(), ""); // No password for simplicity
   IPAddress apIP = WiFi.softAPIP();
   
   Serial.print("Access Point started. SSID: ");
   Serial.println(ap_ssid);
   Serial.print("AP IP address: ");
   Serial.println(apIP);
   
   // Blink WiFi LED to indicate AP mode
   for (int i = 0; i < 5; i++) {
     digitalWrite(WIFI_LED, HIGH);
     delay(100);
     digitalWrite(WIFI_LED, LOW);
     delay(100);
   }
 }
 
 // Reconnect to WiFi if connection is lost
 void reconnectWiFi() {
   if (WiFi.status() != WL_CONNECTED) {
     Serial.println("Reconnecting to WiFi...");
     WiFi.disconnect();
     WiFi.begin(ssid, password);
     
     int attempts = 0;
     while (WiFi.status() != WL_CONNECTED && attempts < 10) {
       delay(500);
       Serial.print(".");
       attempts++;
     }
     
     if(WiFi.status() == WL_CONNECTED) {
       Serial.println("\nWiFi reconnected");
       Serial.print("IP address: ");
       Serial.println(WiFi.localIP());
       digitalWrite(WIFI_LED, HIGH);
     } else {
       Serial.println("\nWiFi reconnection failed");
       digitalWrite(WIFI_LED, LOW);
     }
   }
 }
 
 // Setup web server
 void setupWebServer() {
   // Serve static files from SPIFFS
   server.serveStatic("/", SPIFFS, "/").setDefaultFile("index.html");
   
   // API endpoint for current readings
   server.on("/api/readings", HTTP_GET, [](AsyncWebServerRequest *request) {
     // Create JSON document
     DynamicJsonDocument doc(1024);
     
     // Add current readings
     doc["voltage"] = energyMonitor.getLineVoltage();
     doc["lineCurrent"] = energyMonitor.getLineCurrent();
     doc["nLineCurrent"] = energyMonitor.getNLineCurrent();
     doc["activePower"] = energyMonitor.getActivePower();
     doc["reactivePower"] = energyMonitor.getReactivePower();
     doc["apparentPower"] = energyMonitor.getApparentPower();
     doc["powerFactor"] = energyMonitor.getPowerFactor();
     doc["frequency"] = energyMonitor.getFrequency();
     doc["phaseAngle"] = energyMonitor.getPhaseAngle();
     doc["importEnergy"] = energyMonitor.getImportEnergy();
     doc["exportEnergy"] = energyMonitor.getExportEnergy();
     doc["temperature"] = energyMonitor.getTemperature();
     doc["humidity"] = energyMonitor.getHumidity();
     doc["heatIndex"] = energyMonitor.getHeatIndex();
     doc["timestamp"] = millis();
     
     // Convert to string
     String jsonResponse;
     serializeJson(doc, jsonResponse);
     
     // Send response
     request->send(200, "application/json", jsonResponse);
   });
   
   // API endpoint for device status
   server.on("/api/status", HTTP_GET, [](AsyncWebServerRequest *request) {
     // Create JSON document
     DynamicJsonDocument doc(512);
     
     // Add device status information
     doc["deviceName"] = "FE102M Energy Monitor";
     
     switch(energyMonitor.getOperatingMode()) {
       case STANDALONE:
         doc["operatingMode"] = "Standalone";
         break;
       case WIFI_MODE:
         doc["operatingMode"] = "WiFi";
         break;
       case MODBUS_MODE:
         doc["operatingMode"] = "MODBUS";
         break;
       case DUAL_MODE:
         doc["operatingMode"] = "Dual (WiFi+MODBUS)";
         break;
     }
     
     doc["ipAddress"] = WiFi.localIP().toString();
     doc["macAddress"] = WiFi.macAddress();
     doc["rssi"] = WiFi.RSSI();
     doc["uptime"] = millis() / 1000;
     doc["systemStatus"] = String(energyMonitor.getSysStatus(), HEX);
     doc["meterStatus"] = String(energyMonitor.getMeterStatus(), HEX);
     
     // Convert to string
     String jsonResponse;
     serializeJson(doc, jsonResponse);
     
     // Send response
     request->send(200, "application/json", jsonResponse);
   });
   
   // API endpoint for calibration
   server.on("/api/calibration", HTTP_POST, [](AsyncWebServerRequest *request) {
     // In a real implementation, handle calibration updates here
     request->send(200, "text/plain", "Calibration settings updated");
   });
   
   // Start the server
   server.begin();
   Serial.println("Web server started");
 }