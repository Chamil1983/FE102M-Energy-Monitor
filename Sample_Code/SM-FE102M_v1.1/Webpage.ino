
#ifdef ESP8266
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#elif defined(ESP32)
#include <Arduino.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#else
#error "Board not found"
#endif

#include <WiFiType.h>
#include <FS.h>

#define wifi_is_client_configured()   (WiFi.SSID() != "")

// Wifi mode
#define wifi_mode_is_sta()            (WIFI_STA == (WiFi.getMode() & WIFI_STA))
#define wifi_mode_is_sta_only()       (WIFI_STA == WiFi.getMode())
#define wifi_mode_is_ap()             (WIFI_AP == (WiFi.getMode() & WIFI_AP))

// Performing a scan enables STA so we end up in AP+STA mode so treat AP+STA with no
// ssid set as AP only
#define wifi_mode_is_ap_only()        ((WIFI_AP == WiFi.getMode()) || \
                                       (WIFI_AP_STA == WiFi.getMode() && !wifi_is_client_configured()))

// Timer variables
unsigned long lastTime = 0;
unsigned long timerDelay = 5000;





// Get Sensor Readings and return JSON object
String getSystemInfo() {
    sysdata["vendorname"] = Vendor_Name;
    sysdata["productcode"] = Product_Code;
    sysdata["rev_no"] = Revision_Number;

    sysdata["meter_name"] = meter_name;
    sysdata["model"] = model;
    sysdata["manufacturer"] = manufacturer;
    sysdata["serial_no"] = serial_no;
    sysdata["hardware_rev"] = hardware_rev;
    sysdata["firmware_ver"] = firmware_ver;
    sysdata["mac_add"] = mac_add;
    sysdata["node"] = node;
    sysdata["node_ID"] = String(node_ID);
    sysdata["ssid"] = String(ssid);
    sysdata["ipaddress"] = ipaddress;


    String jsonString = JSON.stringify(sysdata);
    return jsonString;
}

// Get Sensor Readings and return JSON object
String getConfigInfo() {
    configdata["vsagthe"] = String(VoltageSagThreshold);
    configdata["Llinegain"] = String(L_LineCalibrationGain);
    configdata["Llineang"] = String(L_LineCalibrationAngle);
    configdata["Nlinegain"] = String(N_LineCalibrationGain);
    configdata["Nlineang"] = String(N_LineCalibrationAngle);
    configdata["actstartpowthr"] = String(ActiveStartupPowerThreshold);
    configdata["actnoldpwrthr"] = String(ActiveNoLoadPowerThreshold);
    configdata["reactstartpwrthr"] = String(ReactiveStartupPowerThreshold);
    configdata["reactnoldpwrthr"] = String(ReactiveNoLoadPowerThreshold);
    configdata["vrmsgain"] = String(VoltagermsGain);
    configdata["Llinecurrmsgain"] = String(L_LineCurrentrmsGain);
    configdata["Nlinecurrmsgain"] = String(N_LineCurrentrmsGain);
    configdata["voffset"] = String(VoltageOffset);
    configdata["Llinecuroffset"] = String(L_LineCurrentOffset);
    configdata["Nlinecuroffset"] = String(N_LineCurrentOffset);
    configdata["Llineactpwtoffset"] = String(L_LineActivePowerOffset);
    configdata["Llinereactpwroffset"] = String(L_LineReactivePowerOffset);
    configdata["Nlineactpwroffset"] = String(N_LineActivePowerOffset);
    configdata["Nlinereactpwroffset"] = String(N_LineReactivePowerOffset);

    configdata["crc1"] = String(Checksum1);
    configdata["crc2"] = String(Checksum2);



    String jsonString = JSON.stringify(configdata);
    return jsonString;
}


// Get Sensor Readings and return JSON object
String getReadings() {
    readings["vrms"] = String(Voltagerms);
    readings["Lcurrentrms"] = String(L_LineCurrentrms);
    readings["Ncurrentrms"] = String(N_LineCurrentrms);
    readings["frequency"] = String(VoltageFrequency);
    readings["Lpwrfactor"] = String(L_LinePowerFactor);
    readings["phangVandL"] = String(PhaseAnglebetweenVoltageandL_LineCurrent);
    readings["Npwrfactor"] = String(N_LinePowerFactor);
    readings["phangVandN"] = String(PhaseAnglebetweenVoltageandN_LineCurrent);
    readings["Lactpwr"] = String(L_LineMeanActivePower);
    readings["Lreactpwr"] = String(L_LineMeanReactivePower);
    readings["Lapptpwr"] = String(L_LineMeanApparentPower);
    readings["Nactpwr"] = String(N_LineMeanActivePower);
    readings["Nreactpwr"] = String(N_LineMeanReactivePower);
    readings["Napptpwr"] = String(N_LineMeanApparentPower);
    readings["fwrdactengy"] = String(ForwardActiveEnergy);
    readings["rvsactengy"] = String(ReverseActiveEnergy);
    readings["absactengy"] = String(AbsoluteActiveEnergy);
    readings["fwrdreactengy"] = String(Forward_InductiveReactiveEnergy);
    readings["rvsreactengy"] = String(Reverse_CapacitiveReactiveEnergy);
    readings["absreactengy"] = String(AbsoluteReactiveEnergy);


    String jsonString = JSON.stringify(readings);
    return jsonString;
}

// Initialize SPIFFS
void initSPIFFS() {

    SPIFFS.begin(); // mount the fs

    if (!SPIFFS.begin(true)) {
        Serial.println("An error has occurred while mounting SPIFFS");
    }
    Serial.println("SPIFFS mounted successfully");
}


void notifyClients(String meterdata) {
    ws.textAll(meterdata);

}




void handleWebSocketMessage(void* arg, uint8_t* data, size_t len) {
    AwsFrameInfo* info = (AwsFrameInfo*)arg;
    if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
        data[len] = 0;
        String message = (char*)data;
        //Check if the message is "getReadings"
        if (strcmp((char*)data, "getmeterInfo") == 0) {
            //if it is, send current sensor readings
            String meterdata = getSystemInfo();
            Serial.print(meterdata);
            notifyClients(meterdata);
        }

        if (strcmp((char*)data, "getmeterconfig") == 0) {
            //if it is, send current sensor readings

            String configdata = getConfigInfo();
            Serial.print(configdata);
            notifyClients(configdata);
        }

        if (strcmp((char*)data, "getreadings") == 0) {
            //if it is, send current sensor readings

            String meterreading = getReadings();
            Serial.print(meterreading);
            notifyClients(meterreading);
        }


              // try to decipher the JSON string received
        StaticJsonDocument<200> doc;                    // create a JSON container
        DeserializationError error = deserializeJson(doc, data);
        if (error) {
            Serial.print(F("deserializeJson() failed: "));
            Serial.println(error.f_str());
            return;
        }
        else {
            // JSON string was received correctly, so information can be retrieved:
            const int PARAM_INPUT_1 = doc["vsagthetxt"];
            const int PARAM_INPUT_2 = doc["Llinegaintxt"];
            const int PARAM_INPUT_3 = doc["Llineangtxt"];
            const int PARAM_INPUT_4 = doc["Nlinegaintxt"];
            const int PARAM_INPUT_5 = doc["Nlineangtxt"];

            const int PARAM_INPUT_6 = doc["actstartpowthrtxt"];
            const int PARAM_INPUT_7 = doc["actnoldpwrthrtxt"];
            const int PARAM_INPUT_8 = doc["reactstartpwrthrtxt"];
            const int PARAM_INPUT_9 = doc["reactnoldpwrthrtxt"];
            const int PARAM_INPUT_10 = doc["vrmsgaintxt"];
            const int PARAM_INPUT_11 = doc["Llinecurrmsgaintxt"];
            const int PARAM_INPUT_12 = doc["Nlinecurrmsgaintxt"];
            const int PARAM_INPUT_13 = doc["voffsettxt"];
            const int PARAM_INPUT_14 = doc["Llinecuroffsettxt"];
            const int PARAM_INPUT_15 = doc["Nlinecuroffsettxt"];
            const int PARAM_INPUT_16 = doc["Llineactpwtoffsettxt"];
            const int PARAM_INPUT_17 = doc["Llinereactpwroffsettxt"];
            const int PARAM_INPUT_18 = doc["Nlineactpwroffsettxt"];
            const int PARAM_INPUT_19 = doc["Nlinereactpwroffsettxt"];
            const int PARAM_INPUT_20 = doc["crc1txt"];
            const int PARAM_INPUT_21 = doc["crc1tx2"];

            
            VoltageSagThreshold = PARAM_INPUT_1;
            L_LineCalibrationGain = PARAM_INPUT_2;
            L_LineCalibrationAngle = PARAM_INPUT_3;
            N_LineCalibrationGain = PARAM_INPUT_4;
            N_LineCalibrationAngle = PARAM_INPUT_5;
            ActiveStartupPowerThreshold = PARAM_INPUT_6;
            ActiveNoLoadPowerThreshold = PARAM_INPUT_7;
            ReactiveStartupPowerThreshold = PARAM_INPUT_8;
            ReactiveNoLoadPowerThreshold = PARAM_INPUT_9;
            VoltagermsGain = PARAM_INPUT_10;
            L_LineCurrentrmsGain = PARAM_INPUT_11;
            N_LineCurrentrmsGain = PARAM_INPUT_12;
            VoltageOffset = PARAM_INPUT_13;
            L_LineCurrentOffset = PARAM_INPUT_14;
            N_LineCurrentOffset = PARAM_INPUT_15;
            L_LineActivePowerOffset = PARAM_INPUT_16;
            L_LineReactivePowerOffset = PARAM_INPUT_17;
            N_LineActivePowerOffset = PARAM_INPUT_18;
            N_LineReactivePowerOffset = PARAM_INPUT_19;
            Checksum1 = PARAM_INPUT_20;
            Checksum2 = PARAM_INPUT_21;

            
                        
            Serial.println("vsagthetxt: " + String(PARAM_INPUT_1));
            Serial.println("Llinegaintxt: " + String(PARAM_INPUT_2));
            Serial.println("Llineangtxt: " + String(PARAM_INPUT_3));
            Serial.println("Nlinegaintxt: " + String(PARAM_INPUT_4));
            Serial.println("Nlineangtxt: " + String(PARAM_INPUT_5));
            Serial.println("actstartpowthrtxt: " + String(PARAM_INPUT_6));
            Serial.println("actnoldpwrthrtxt: " + String(PARAM_INPUT_7));
            Serial.println("reactstartpwrthrtxt: " + String(PARAM_INPUT_8));
            Serial.println("reactnoldpwrthrtxt: " + String(PARAM_INPUT_9));
            Serial.println("vrmsgaintxt: " + String(PARAM_INPUT_10));
            Serial.println("Llinecurrmsgaintxt: " + String(PARAM_INPUT_11));
            Serial.println("Nlinecurrmsgaintxt: " + String(PARAM_INPUT_12));
            Serial.println("voffsettxt: " + String(PARAM_INPUT_13));
            Serial.println("Llinecuroffsettxt: " + String(PARAM_INPUT_14));
            Serial.println("Nlinecuroffsettxt: " + String(PARAM_INPUT_15));
            Serial.println("Llineactpwtoffsettxt: " + String(PARAM_INPUT_16));
            Serial.println("Llinereactpwroffsettxt: " + String(PARAM_INPUT_17));
            Serial.println("Nlineactpwroffsettxt: " + String(PARAM_INPUT_18));
            Serial.println("Nlinereactpwroffsettxt: " + String(PARAM_INPUT_19));

            Serial.println("crc1txt: " + String(PARAM_INPUT_20));
            Serial.println("crc1tx2: " + String(PARAM_INPUT_21));

            Save_CalConfiguration();

        }
        Serial.println("");

        systemRestartTime = millis() + 1000;
        
    }

    
    
}


void onEvent(AsyncWebSocket* server, AsyncWebSocketClient* client, AwsEventType type, void* arg, uint8_t* data, size_t len) {
    switch (type) {

    case WS_EVT_CONNECT:
        Serial.printf("WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
        break;
    
    case WS_EVT_DISCONNECT:
        Serial.printf("WebSocket client #%u disconnected\n", client->id());
        break;
    case WS_EVT_DATA:
        handleWebSocketMessage(arg, data, len);
        break;
    
    case WS_EVT_PONG:
    case WS_EVT_ERROR:
        break;
    }
}


void initWebSocket() {
    ws.onEvent(onEvent);
    server.addHandler(&ws);
}


void Setup_Webserver(void) {


    initSPIFFS();
    initWebSocket();
    Load_Reg_CalConfiguration();



    // Setup the static files

    server.serveStatic("/", SPIFFS, "/")
        .setDefaultFile("index.html");



    // Web Server Root URL

    server.on("/", HTTP_GET, [](AsyncWebServerRequest* request) {
        request->send(SPIFFS, "/index.html", "text/html");
        });


    server.on("/config", HTTP_GET, [](AsyncWebServerRequest* request) {
        request->send(SPIFFS, "/configurations.html", "text/html");
        });


    server.on("/readdata", HTTP_GET, [](AsyncWebServerRequest* request) {
        request->send(SPIFFS, "/readings.html", "text/html");
        });

    server.serveStatic("/", SPIFFS, "/");


    // Start server
    server.begin();


}


void Webserver_loop(void) {



    if ((millis() - lastTime) > timerDelay) {
        String meterdata = getSystemInfo();
        String configdata = getConfigInfo();
        String meterreading = getReadings();
        Serial.print(meterdata);
        notifyClients(meterdata);

        Serial.print(configdata);
        notifyClients(configdata);

        Serial.print(meterreading);
        notifyClients(meterreading);

        lastTime = millis();

    }

    ws.cleanupClients();

    // Do we need to restart the system?
    if (systemRestartTime > 0 && millis() > systemRestartTime) {
        systemRestartTime = 0;
        eic.InitCallibrate();
    }


}

void notFound(AsyncWebServerRequest* request) {
    request->send(404, "text/plain", "Not found");
}

void getconfig() {



    server.onNotFound(notFound);
    server.begin();
}

//==============================================================================================/






