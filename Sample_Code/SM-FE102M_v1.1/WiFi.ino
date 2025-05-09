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
#include "config.h"
#include <WiFi.h>
#include <WiFiMulti.h>
#include <ESPmDNS.h>
#include <DNSServer.h>
#include "time.h"






#define wifi_is_client_configured()   (WiFi.SSID() != "")

    // Wifi mode
#define wifi_mode_is_sta()            (WIFI_STA == (WiFi.getMode() & WIFI_STA))
#define wifi_mode_is_sta_only()       (WIFI_STA == WiFi.getMode())
#define wifi_mode_is_ap()             (WIFI_AP == (WiFi.getMode() & WIFI_AP))

// Performing a scan enables STA so we end up in AP+STA mode so treat AP+STA with no
// ssid set as AP only
#define wifi_mode_is_ap_only()        ((WIFI_AP == WiFi.getMode()) || \
                                       (WIFI_AP_STA == WiFi.getMode() && !wifi_is_client_configured()))




DNSServer dnsServer;                  // Create class DNS server, captive portal re-direct
const byte DNS_PORT = 53;

// Access Point SSID, password & IP address. SSID will be softAP_ssid + chipID to make SSID unique
const char* softAP_ssid = "SM-FE102M";
const char* softAP_password = "";
IPAddress apIP(192, 168, 4, 1);
IPAddress netMsk(255, 255, 255, 0);
int apClients = 0;

bool startAPonWifiDisconnect = true;

// hostname for mDNS. Should work at least on windows. Try http://emonesp.local
const char* esp_hostname = "smenergy";

// Last discovered WiFi access points
String st;
String rssi;


// Wifi Network Strings
String connected_network = "";
String ipaddress = "";

int client_disconnects = 0;
bool client_retry = false;
unsigned long client_retry_time = 0;

#ifdef WIFI_LED


#ifndef WIFI_LED_ON_STATE
#define WIFI_LED_ON_STATE LOW
#endif

//the time the LED actually stays on
#ifndef WIFI_LED_ON_TIME
#define WIFI_LED_ON_TIME 50
#endif


//times the LED is off...
#ifndef WIFI_LED_AP_TIME
#define WIFI_LED_AP_TIME 2000
#endif

#ifndef WIFI_LED_AP_CONNECTED_TIME
#define WIFI_LED_AP_CONNECTED_TIME 1000
#endif

#ifndef WIFI_LED_STA_CONNECTING_TIME
#define WIFI_LED_STA_CONNECTING_TIME 500
#endif

#ifndef WIFI_LED_STA_CONNECTED_TIME
#define WIFI_LED_STA_CONNECTED_TIME 4000
#endif



#ifndef WIFI_LED_AP_TIME
#define WIFI_LED_AP_TIME 1000
#endif

#ifndef WIFI_LED_AP_CONNECTED_TIME
#define WIFI_LED_AP_CONNECTED_TIME 100
#endif

#ifndef WIFI_LED_STA_CONNECTING_TIME
#define WIFI_LED_STA_CONNECTING_TIME 500
#endif

int wifiLedState = !WIFI_LED_ON_STATE;
unsigned long wifiLedTimeOut = millis();
#endif

#ifndef WIFI_BUTTON
#define WIFI_BUTTON 0
#endif

#ifndef WIFI_BUTTON_AP_TIMEOUT
#define WIFI_BUTTON_AP_TIMEOUT              (5 * 1000)
#endif

#ifndef WIFI_BUTTON_FACTORY_RESET_TIMEOUT
#define WIFI_BUTTON_FACTORY_RESET_TIMEOUT   (10 * 1000)
#endif

#ifndef WIFI_CLIENT_RETRY_TIMEOUT
#define WIFI_CLIENT_RETRY_TIMEOUT (5 * 60 * 1000)
#endif

int wifiButtonState = HIGH;
unsigned long wifiButtonTimeOut = millis();
bool apMessage = false;

// -------------------------------------------------------------------
// Start Access Point
// Access point is used for wifi network selection
// -------------------------------------------------------------------
void
startAP() {
    DBUGS.println("Starting AP");

    if (wifi_mode_is_sta()) {
        WiFi.disconnect(true);
    }

    WiFi.enableAP(true);

    WiFi.softAPConfig(apIP, apIP, netMsk);
    // Create Unique SSID e.g "emonESP_XXXXXX"

    String softAP_ssid_ID =
#ifdef ESP32
        String(softAP_ssid) + "_" + String((uint32_t)ESP.getEfuseMac());
#else
        String(softAP_ssid) + "_" + String(ESP.getChipId());
#endif

    int channel = (random(3) * 5) + 1;
    WiFi.softAP(node_name.c_str(), softAP_password, channel);

    // Setup the DNS server redirecting all the domains to the apIP
    dnsServer.setErrorReplyCode(DNSReplyCode::NoError);
    dnsServer.start(DNS_PORT, "*", apIP);

    IPAddress myIP = WiFi.softAPIP();
    char tmpStr[40];
    sprintf(tmpStr, "%d.%d.%d.%d", myIP[0], myIP[1], myIP[2], myIP[3]);
    DBUGS.print(F("AP IP Address: "));
    DBUGS.println(tmpStr);
    ipaddress = tmpStr;

    apClients = 0;
}

// -------------------------------------------------------------------
// Start Client, attempt to connect to Wifi network
// -------------------------------------------------------------------
void
startClient()
{
    WiFi.disconnect();
    delay(10);
    DBUGS.print(F("Connecting to SSID: "));
    DBUGS.println(ssid);
    //DEBUG.print(F(" PSK:"));
    //DEBUG.println(epass.c_str());

    client_disconnects = 0;

    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        digitalWrite(WIFI_LED, HIGH);
        delay(250);
        digitalWrite(FAULT_LED, LOW);
        delay(250);
    }

    WiFi.hostname(node_name.c_str());
    digitalWrite(FAULT_LED, HIGH);
    digitalWrite(WIFI_LED, LOW);
    Serial.println("");
    Serial.println("WiFi connected");
    WiFi.enableSTA(true);





}

static void wifi_start()
{
    // 1) If no network configured start up access point
    DBUGVAR(ssid);
    if (ssid == 0 || ssid == "SM-FE102M(WiFi)")
    {
        startAP();
    }
    // 2) else try and connect to the configured network
    else
    {
        startClient();
    }
}

#ifdef ESP32

void WiFiEvent(WiFiEvent_t event)
{

#ifdef WIFI_LED
    wifiLedState = WIFI_LED_ON_STATE;
    digitalWrite(WIFI_LED, wifiLedState);
#endif

    switch (event) {
    case SYSTEM_EVENT_WIFI_READY:
        DBUGS.println("WiFi interface ready");
        break;
    case SYSTEM_EVENT_SCAN_DONE:
        DBUGS.println("Completed scan for access points");
        break;
    case SYSTEM_EVENT_STA_START:
        DBUGS.println("WiFi client started");
        break;
    case SYSTEM_EVENT_STA_STOP:
        DBUGS.println("WiFi clients stopped");
        break;
    case SYSTEM_EVENT_STA_CONNECTED:
        DBUGS.print("Connected to SSID: ");
        DBUGS.println(ssid);
        client_disconnects = 0;
        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
        DBUGS.println("Disconnected from WiFi access point");
        break;
    case SYSTEM_EVENT_STA_AUTHMODE_CHANGE:
        DBUGS.println("Authentication mode of access point has changed");
        break;
    case SYSTEM_EVENT_STA_GOT_IP: {
        IPAddress myAddress = WiFi.localIP();
        char tmpStr[40];
        sprintf(tmpStr, "%d.%d.%d.%d", myAddress[0], myAddress[1], myAddress[2], myAddress[3]);
        ipaddress = tmpStr;
        DBUGS.print("SM-FE102M(WiFi) IP: ");
        DBUGS.println(tmpStr);

        // Copy the connected network and ipaddress to global strings for use in status request
        connected_network = ssid;

        // Clear any error state
        client_disconnects = 0;
        client_retry = false;
    }
                                break;
    case SYSTEM_EVENT_STA_LOST_IP:
        DBUGS.println("Lost IP address and IP address is reset to 0");
        break;
    case SYSTEM_EVENT_STA_WPS_ER_SUCCESS:
        DBUGS.println("WiFi Protected Setup (WPS): succeeded in enrollee mode");
        break;
    case SYSTEM_EVENT_STA_WPS_ER_FAILED:
        DBUGS.println("WiFi Protected Setup (WPS): failed in enrollee mode");
        break;
    case SYSTEM_EVENT_STA_WPS_ER_TIMEOUT:
        DBUGS.println("WiFi Protected Setup (WPS): timeout in enrollee mode");
        break;
    case SYSTEM_EVENT_STA_WPS_ER_PIN:
        DBUGS.println("WiFi Protected Setup (WPS): pin code in enrollee mode");
        break;
    case SYSTEM_EVENT_AP_START:
        DBUGS.println("WiFi access point started");
        break;
    case SYSTEM_EVENT_AP_STOP:
        DBUGS.println("WiFi access point stopped");
        break;
    case SYSTEM_EVENT_AP_STACONNECTED:
        DBUGS.println("Client connected");
        apClients++;
        break;
    case SYSTEM_EVENT_AP_STADISCONNECTED:
        DBUGS.println("Client disconnected");
        apClients--;
        break;
    case SYSTEM_EVENT_AP_STAIPASSIGNED:
        DBUGS.println("Assigned IP address to client");
        break;
    case SYSTEM_EVENT_AP_PROBEREQRECVED:
        DBUGS.println("Received probe request");
        break;
    default:
        break;
    }
}

#else //ESP8266
void wifi_onStationModeGotIP(const WiFiEventStationModeGotIP& event)
{
    IPAddress myAddress = WiFi.localIP();
    char tmpStr[40];
    sprintf(tmpStr, "%d.%d.%d.%d", myAddress[0], myAddress[1], myAddress[2], myAddress[3]);
    ipaddress = tmpStr;
    DBUGS.print("Connected, IP: ");
    DBUGS.println(tmpStr);

    // Copy the connected network and ipaddress to global strings for use in status request
    connected_network = esid;

    // Clear any error state
    client_disconnects = 0;
    client_retry = false;
}

void wifi_onStationModeDisconnected(const WiFiEventStationModeDisconnected& event)
{
    DBUGF("WiFi disconnected: %s",
        WIFI_DISCONNECT_REASON_UNSPECIFIED == event.reason ? "WIFI_DISCONNECT_REASON_UNSPECIFIED" :
        WIFI_DISCONNECT_REASON_AUTH_EXPIRE == event.reason ? "WIFI_DISCONNECT_REASON_AUTH_EXPIRE" :
        WIFI_DISCONNECT_REASON_AUTH_LEAVE == event.reason ? "WIFI_DISCONNECT_REASON_AUTH_LEAVE" :
        WIFI_DISCONNECT_REASON_ASSOC_EXPIRE == event.reason ? "WIFI_DISCONNECT_REASON_ASSOC_EXPIRE" :
        WIFI_DISCONNECT_REASON_ASSOC_TOOMANY == event.reason ? "WIFI_DISCONNECT_REASON_ASSOC_TOOMANY" :
        WIFI_DISCONNECT_REASON_NOT_AUTHED == event.reason ? "WIFI_DISCONNECT_REASON_NOT_AUTHED" :
        WIFI_DISCONNECT_REASON_NOT_ASSOCED == event.reason ? "WIFI_DISCONNECT_REASON_NOT_ASSOCED" :
        WIFI_DISCONNECT_REASON_ASSOC_LEAVE == event.reason ? "WIFI_DISCONNECT_REASON_ASSOC_LEAVE" :
        WIFI_DISCONNECT_REASON_ASSOC_NOT_AUTHED == event.reason ? "WIFI_DISCONNECT_REASON_ASSOC_NOT_AUTHED" :
        WIFI_DISCONNECT_REASON_DISASSOC_PWRCAP_BAD == event.reason ? "WIFI_DISCONNECT_REASON_DISASSOC_PWRCAP_BAD" :
        WIFI_DISCONNECT_REASON_DISASSOC_SUPCHAN_BAD == event.reason ? "WIFI_DISCONNECT_REASON_DISASSOC_SUPCHAN_BAD" :
        WIFI_DISCONNECT_REASON_IE_INVALID == event.reason ? "WIFI_DISCONNECT_REASON_IE_INVALID" :
        WIFI_DISCONNECT_REASON_MIC_FAILURE == event.reason ? "WIFI_DISCONNECT_REASON_MIC_FAILURE" :
        WIFI_DISCONNECT_REASON_4WAY_HANDSHAKE_TIMEOUT == event.reason ? "WIFI_DISCONNECT_REASON_4WAY_HANDSHAKE_TIMEOUT" :
        WIFI_DISCONNECT_REASON_GROUP_KEY_UPDATE_TIMEOUT == event.reason ? "WIFI_DISCONNECT_REASON_GROUP_KEY_UPDATE_TIMEOUT" :
        WIFI_DISCONNECT_REASON_IE_IN_4WAY_DIFFERS == event.reason ? "WIFI_DISCONNECT_REASON_IE_IN_4WAY_DIFFERS" :
        WIFI_DISCONNECT_REASON_GROUP_CIPHER_INVALID == event.reason ? "WIFI_DISCONNECT_REASON_GROUP_CIPHER_INVALID" :
        WIFI_DISCONNECT_REASON_PAIRWISE_CIPHER_INVALID == event.reason ? "WIFI_DISCONNECT_REASON_PAIRWISE_CIPHER_INVALID" :
        WIFI_DISCONNECT_REASON_AKMP_INVALID == event.reason ? "WIFI_DISCONNECT_REASON_AKMP_INVALID" :
        WIFI_DISCONNECT_REASON_UNSUPP_RSN_IE_VERSION == event.reason ? "WIFI_DISCONNECT_REASON_UNSUPP_RSN_IE_VERSION" :
        WIFI_DISCONNECT_REASON_INVALID_RSN_IE_CAP == event.reason ? "WIFI_DISCONNECT_REASON_INVALID_RSN_IE_CAP" :
        WIFI_DISCONNECT_REASON_802_1X_AUTH_FAILED == event.reason ? "WIFI_DISCONNECT_REASON_802_1X_AUTH_FAILED" :
        WIFI_DISCONNECT_REASON_CIPHER_SUITE_REJECTED == event.reason ? "WIFI_DISCONNECT_REASON_CIPHER_SUITE_REJECTED" :
        WIFI_DISCONNECT_REASON_BEACON_TIMEOUT == event.reason ? "WIFI_DISCONNECT_REASON_BEACON_TIMEOUT" :
        WIFI_DISCONNECT_REASON_NO_AP_FOUND == event.reason ? "WIFI_DISCONNECT_REASON_NO_AP_FOUND" :
        WIFI_DISCONNECT_REASON_AUTH_FAIL == event.reason ? "WIFI_DISCONNECT_REASON_AUTH_FAIL" :
        WIFI_DISCONNECT_REASON_ASSOC_FAIL == event.reason ? "WIFI_DISCONNECT_REASON_ASSOC_FAIL" :
        WIFI_DISCONNECT_REASON_HANDSHAKE_TIMEOUT == event.reason ? "WIFI_DISCONNECT_REASON_HANDSHAKE_TIMEOUT" :
        "UNKNOWN");

    client_disconnects++;
    MDNS.end();
}

#endif


void setup_wifi() {
    int k = 0;
    WiFi.disconnect();
    delay(10);
    // We start by connecting to a WiFi network
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(ssid);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {

        digitalWrite(WIFI_LED, HIGH);
        delay(250);
        Serial.println("Connecting to WiFi...");
        digitalWrite(FAULT_LED, LOW);
        delay(250);
    }

    //WiFi.config(staticIP_10, gateway_10, subnet_10);


    digitalWrite(FAULT_LED, HIGH);
    digitalWrite(WIFI_LED, LOW);
    Serial.println("");
    Serial.println("WiFi connected");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());

    IPAddress myIP = WiFi.softAPIP();
    sprintf(Ip_Add, "%d.%d.%d.%d", myIP[0], myIP[1], myIP[2], myIP[3]);


    ipaddress = Ip_Add;
}



void
wifi_setup() {
#ifdef WIFI_LED
    pinMode(WIFI_LED, OUTPUT);
    digitalWrite(WIFI_LED, wifiLedState);
#endif

    randomSeed(analogRead(0));


#ifdef ESP32
    WiFi.onEvent(WiFiEvent);

#else
    static auto _onStationModeConnected = WiFi.onStationModeConnected([](const WiFiEventStationModeConnected& event) {
        DBUGF("Connected to %s", event.ssid.c_str());
        });
    static auto _onStationModeGotIP = WiFi.onStationModeGotIP(wifi_onStationModeGotIP);
    static auto _onStationModeDisconnected = WiFi.onStationModeDisconnected(wifi_onStationModeDisconnected);
    static auto _onSoftAPModeStationConnected = WiFi.onSoftAPModeStationConnected([](const WiFiEventSoftAPModeStationConnected& event) {
        apClients++;
        });
    static auto _onSoftAPModeStationDisconnected = WiFi.onSoftAPModeStationDisconnected([](const WiFiEventSoftAPModeStationDisconnected& event) {
        apClients--;
        });
#endif

    wifi_start();
    //startClient();

    if (MDNS.begin(esp_hostname)) {
        Serial.println(esp_hostname);
        MDNS.addService("http", "tcp", 80);
    }

    client_retry_time = millis();

}

void wifi_loop()
{

    Profile_Start(wifi_loop);

    bool isClient = wifi_mode_is_sta();
    bool isClientOnly = wifi_mode_is_sta_only();
    bool isAp = wifi_mode_is_ap();
    bool isApOnly = wifi_mode_is_ap_only();

    // flash the LED according to what state wifi is in
    // if AP mode & disconnected - blink every 2 seconds
    // if AP mode & someone is connected - blink fast
    // if Client mode - slow blink every 4 seconds

#ifdef WIFI_LED
    if ((isApOnly || !WiFi.isConnected()) && millis() > wifiLedTimeOut) {
        wifiLedState = !wifiLedState;
        digitalWrite(WIFI_LED, wifiLedState);

        if (wifiLedState) {
            wifiLedTimeOut = millis() + WIFI_LED_ON_TIME;
        }
        else {
            int ledTime = isApOnly ? (0 == apClients ? WIFI_LED_AP_TIME : WIFI_LED_AP_CONNECTED_TIME) : WIFI_LED_STA_CONNECTING_TIME;
            wifiLedTimeOut = millis() + ledTime;
        }
    }
    if ((isClientOnly || WiFi.isConnected()) && millis() > wifiLedTimeOut) {
        wifiLedState = !wifiLedState;
        digitalWrite(WIFI_LED, wifiLedState);

        if (wifiLedState) {
            wifiLedTimeOut = millis() + WIFI_LED_ON_TIME;
        }
        else {
            int ledTime = WIFI_LED_STA_CONNECTED_TIME;
            wifiLedTimeOut = millis() + ledTime;
        }
    }
#endif

#if defined(WIFI_LED) && WIFI_BUTTON == WIFI_LED
    digitalWrite(WIFI_BUTTON, HIGH);
    pinMode(WIFI_BUTTON, INPUT_PULLUP);
#endif

    // Pressing the WIFI_BUTTON for 5 seconds will turn on AP mode, 10 seconds will factory reset
    int button = digitalRead(WIFI_BUTTON);

#if defined(WIFI_LED) && WIFI_BUTTON == WIFI_LED
    pinMode(WIFI_BUTTON, OUTPUT);
    digitalWrite(WIFI_LED, wifiLedState);
#endif

    //DBUGF("%lu %d %d", millis() - wifiButtonTimeOut, button, wifiButtonState);
    if (wifiButtonState != button)
    {
        wifiButtonState = button;
        if (LOW == button) {
            DBUGS.println("Button pressed");
            wifiButtonTimeOut = millis();
            apMessage = false;
        }
        else {
            DBUGS.println("Button released");
            if (millis() > wifiButtonTimeOut + WIFI_BUTTON_AP_TIMEOUT) {
                wifi_turn_on_ap();
            }
        }
    }
    /*
        if (LOW == wifiButtonState && millis() > wifiButtonTimeOut + WIFI_BUTTON_FACTORY_RESET_TIMEOUT)
        {
            DBUGS.println("Factory reset");
            delay(1000);

            config_reset();

    #ifdef ESP32
            WiFi.disconnect(false, true);
            delay(50);
            esp_restart();
    #else
            WiFi.disconnect();
            ESP.eraseConfig();
            delay(50);
            ESP.reset();
    #endif
        }
        else if (false == apMessage && LOW == wifiButtonState && millis() > wifiButtonTimeOut + WIFI_BUTTON_AP_TIMEOUT)
        {
            DBUGS.println("Access point");
            apMessage = true;
        }


    */


    // Manage state while connecting
    if (startAPonWifiDisconnect) {
        while (wifi_mode_is_sta_only() && !wifi_mode_is_ap_only() && !WiFi.isConnected())
        {
            client_disconnects++; //set to 0 when connection to AP is made

            // If we have failed to connect 3 times, turn on the AP
            if (client_disconnects > 2) {
                DBUGS.println("Start AP if WiFi can not reconnect to AP");
                startAP();
                client_retry = true;
                //client_retry_time = millis() + WIFI_CLIENT_RETRY_TIMEOUT;
                client_disconnects = 0;
            }
            else {
                // wait 10 seconds and retry
#ifdef ENABLE_WDT
        // so watchdog (hard coded to 5 seconds) is not triggered by delay
                if (WIFI_CLIENT_DISCONNECT_RETRY >= 5000) {
                    int disconnect_retry;
                    int dr_div;
                    int i = 0;
                    dr_div = WIFI_CLIENT_DISCONNECT_RETRY / 1000;
                    disconnect_retry = WIFI_CLIENT_DISCONNECT_RETRY / dr_div;
                    DBUGS.print("disconnect retry time: ");
                    DBUGS.println(disconnect_retry);
                    while (i < dr_div) {
                        delay(disconnect_retry);
                        feedLoopWDT();
                        i++;
                    }
                }
                else {
#endif
                    delay(WIFI_CLIENT_DISCONNECT_RETRY);
#ifdef ENABLE_WDT
                }
#endif
                wifi_restart();
            }
#ifdef ENABLE_WDT
            feedLoopWDT();
#endif
        }
    }

    // Remain in AP mode if no one is connected for 5 Minutes before resetting
    if (isApOnly && 0 == apClients && client_retry && ((millis() - client_retry_time) >= WIFI_CLIENT_RETRY_TIMEOUT)) {
        DBUGS.println("Try to connect to client again - resetting");
        wifi_turn_off_ap();
        delay(50);
#ifdef ESP32
        esp_restart();
#else
        ESP.reset();
#endif
    }

    Profile_End(wifi_loop, 5);
}


void wifi_scan() {
    int n = WiFi.scanNetworks();
    DBUGS.print(n);
    DBUGS.println(" networks found");
    st = "";
    rssi = "";
    for (int i = 0; i < n; ++i) {
        st += "\"" + WiFi.SSID(i) + "\"";
        rssi += "\"" + String(WiFi.RSSI(i)) + "\"";
        if (i < n - 1)
            st += ",";
        if (i < n - 1)
            rssi += ",";
    }
}


void wifi_restart() {
    wifi_disconnect();
    wifi_start();
}

void wifi_disconnect() {
    wifi_turn_off_ap();
    if (wifi_mode_is_sta()) {
        WiFi.disconnect(true);
    }
}

void wifi_turn_off_ap()
{
    if (wifi_mode_is_ap())
    {
        WiFi.softAPdisconnect(true);
        dnsServer.stop();
    }
}

void wifi_turn_on_ap()
{
    DBUGF("wifi_turn_on_ap %d", WiFi.getMode());
    if (!wifi_mode_is_ap()) {
        startAP();
    }
}

bool wifi_client_connected()
{
    return WiFi.isConnected() && (WIFI_STA == (WiFi.getMode() & WIFI_STA));
}
