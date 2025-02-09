/****************************************************************************
 *
 * Copyright (c) 2015, 2016 Gus Grubba. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file main.cpp
 * ESP8266 Wifi AP, MavLink UART/UDP Bridge
 *
 * @author Gus Grubba <mavlink@grubba.com>
 */

#include "mavesp8266.h"
#include "mavesp8266_parameters.h"
#include "mavesp8266_gcs.h"
#include "mavesp8266_vehicle.h"
#include "mavesp8266_httpd.h"
#include "mavesp8266_component.h"
#include "mavesp8266_power_mgmt.h"

#include <ESP8266mDNS.h>

#define GPIO02  2

//---------------------------------------------------------------------------------
//-- HTTP Update Status
class MavESP8266UpdateImp : public MavESP8266Update {
public:
    MavESP8266UpdateImp ()
        : _isUpdating(false)
        , _rebootAt(0)
    {

    }
    void clearScheduledReboot ()
    {
        _rebootAt = 0;
    }
    void scheduleReboot (uint32_t delayMs)
    {
        _rebootAt = millis() + delayMs;
    }
    void updateStarted  ()
    {
        _isUpdating = true;
    }
    void updateCompleted()
    {
        //-- TODO
    }
    void updateError    ()
    {
        //-- TODO
    }
    bool isRebootPending() { return _rebootAt != 0; }
    bool isUpdating     () { return _isUpdating; }
    bool shouldReboot   () { return isRebootPending() && millis() > _rebootAt; }
private:
    bool _isUpdating;
    uint32_t _rebootAt;
};



//-- Singletons
IPAddress               localIP;
IPAddress               subnetMask;
MavESP8266Component     Component;
MavESP8266Parameters    Parameters;
MavESP8266GCS           GCS;
MavESP8266Vehicle       Vehicle;
MavESP8266Httpd         updateServer;
MavESP8266UpdateImp     updateStatus;
MavESP8266Log           Logger;
MavESP8266PowerMgmt     Power;

//---------------------------------------------------------------------------------
//-- Accessors
class MavESP8266WorldImp : public MavESP8266World {
public:
    MavESP8266Parameters*   getParameters   () { return &Parameters;    }
    MavESP8266Component*    getComponent    () { return &Component;     }
    MavESP8266Vehicle*      getVehicle      () { return &Vehicle;       }
    MavESP8266GCS*          getGCS          () { return &GCS;           }
    MavESP8266Log*          getLogger       () { return &Logger;        }
    MavESP8266PowerMgmt*    getPowerMgmt    () { return &Power;         }
};

MavESP8266WorldImp      World;

MavESP8266World* getWorld()
{
    return &World;
}

//---------------------------------------------------------------------------------
//-- Wait for a DHCPD client
void wait_for_client() {
    DEBUG_LOG("Waiting for a client...\n");
#ifdef ENABLE_DEBUG
    int wcount = 0;
#endif
    uint8 client_count = wifi_softap_get_station_num();
    while (!client_count) {
#ifdef ENABLE_DEBUG
        Serial1.print(".");
        if(++wcount > 80) {
            wcount = 0;
            Serial1.println();
        }
#endif
        delay(1000);
        client_count = wifi_softap_get_station_num();
    }
    DEBUG_LOG("Got %d client(s)\n", client_count);
}

//---------------------------------------------------------------------------------
//-- Reset all parameters whenever the reset gpio pin is active
void reset_interrupt(){
    Parameters.resetToDefaults();
    Parameters.saveAllToEeprom();
    ESP.reset();
}

//---------------------------------------------------------------------------------
//-- Set things up
void setup() {
    delay(1000);
    Parameters.begin();
#ifdef ENABLE_DEBUG
    //   We only use it for non debug because GPIO02 is used as a serial
    //   pin (TX) when debugging.
    Serial1.begin(115200);
#else
    //-- Initialized GPIO02 (Used for "Reset To Factory")
    // This seems to cause a reset loop at boot time with the Adafruit HUZZAH
    // ESP8266 so it is now disabled.
    //
    // pinMode(GPIO02, INPUT_PULLUP);
    // attachInterrupt(digitalPinToInterrupt(GPIO02), reset_interrupt, FALLING);
#endif
    Logger.begin(2048);

#ifdef FC_POWER_CONTROL_PIN
    //-- Initialize power control pin (if there is one)
    Power.setControlPinIsActiveHigh(FC_POWER_CONTROL_PIN_ACTIVE_STATE == HIGH);
    Power.setControlPinIndex(FC_POWER_CONTROL_PIN);
    Power.setPulseLengthMsec(FC_POWER_CONTROL_PIN_PULSE_LENGTH_MSEC);
#endif
#ifdef FC_POWER_QUERY_PIN
    //-- Initialize power query pin (if there is one)
    Power.setQueryPinIndex(FC_POWER_QUERY_PIN);
#endif
    Power.begin();

    DEBUG_LOG("\nConfiguring access point...\n");
    DEBUG_LOG("Free Sketch Space: %u\n", ESP.getFreeSketchSpace());

    WiFi.disconnect(true);

    if(Parameters.getWifiMode() == WIFI_MODE_STA){
        //-- Connect to an existing network
        WiFi.mode(WIFI_STA);
        WiFi.config(Parameters.getWifiStaIP(), Parameters.getWifiStaGateway(), Parameters.getWifiStaSubnet(), 0U, 0U);
        WiFi.begin(Parameters.getWifiStaSsid(), Parameters.getWifiStaPassword());

        //-- Wait a minute to connect
        for(int i = 0; i < 120 && WiFi.status() != WL_CONNECTED; i++) {
            #ifdef ENABLE_DEBUG
            Serial.print(".");
            #endif
            delay(500);
        }
        if(WiFi.status() == WL_CONNECTED) {
            localIP = WiFi.localIP();
            subnetMask = WiFi.subnetMask();
            WiFi.setAutoReconnect(true);
        } else {
            //-- Fall back to AP mode if no connection could be established
            WiFi.disconnect(true);
            Parameters.setWifiMode(WIFI_MODE_AP);
        }
    }

    if(Parameters.getWifiMode() == WIFI_MODE_AP){
        //-- Start AP
        WiFi.mode(WIFI_AP);
        WiFi.encryptionType(AUTH_WPA2_PSK);
        WiFi.softAP(Parameters.getWifiSsid(), Parameters.getWifiPassword(), Parameters.getWifiChannel());
        localIP = WiFi.softAPIP();
        subnetMask = IPAddress(255, 255, 255, 0);
        wait_for_client();
    }

    //-- Boost power to Max
    WiFi.setOutputPower(20.5);

    //-- MDNS
    char mdnsName[256];
    sprintf(mdnsName, "MavEsp8266-%d",localIP[3]);
    MDNS.begin(mdnsName);
    MDNS.addService("http", "tcp", 80);

    //-- Initialize Comm Links
    DEBUG_LOG("Start WiFi Bridge\n");
    DEBUG_LOG("Local IP: %s\n", localIP.toString().c_str());

    Parameters.setLocalIPAddress(localIP);

    //-- Start broadcasting first and switch to the GCS IP once we get it.
    IPAddress gcs_ip(localIP);
    for (int i = 0; i < 4; i++) {
        gcs_ip[i] = (gcs_ip[i] & subnetMask[i]) | (0xFF & ~subnetMask[i]);
    }
    GCS.begin(&Vehicle, gcs_ip);

    //-- Assume that the last byte of the IP address is the MAVLink system ID
    Vehicle.begin(&GCS, localIP[3]);

    //-- Initialize Update Server
    updateServer.begin(&updateStatus);
}

//---------------------------------------------------------------------------------
//-- Main Loop
void loop() {
    if(!updateStatus.isUpdating()) {
        if (Component.inRawMode()) {
            GCS.readMessageRaw();
            delay(0);
            Vehicle.readMessageRaw();

        } else {
            GCS.readMessage();
            delay(0);
            Vehicle.readMessage();
        }
        Power.loop();
    }
    if (updateStatus.shouldReboot()) {
        updateStatus.clearScheduledReboot();
        ESP.restart();
    }
    updateServer.checkUpdates();
}
