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
#include "led_manager.h"

#include <ESP8266mDNS.h>

#define GPIO02 2
//---------------------------------------------------------------------------------
//-- HTTP Update Status
class MavESP8266UpdateImp : public MavESP8266Update
{
public:
    MavESP8266UpdateImp()
        : _isUpdating(false)
    {
    }
    void updateStarted()
    {
        _isUpdating = true;
    }
    void updateCompleted()
    {
        //-- TODO
    }
    void updateError()
    {
        //-- TODO
    }
    bool isUpdating() { return _isUpdating; }

private:
    bool _isUpdating;
};

//-- Singletons
LEDManager ledManager;
IPAddress localIP;
MavESP8266Component Component;
MavESP8266Parameters Parameters;
MavESP8266GCS GCS(ledManager);
MavESP8266Vehicle Vehicle(ledManager);
MavESP8266Httpd updateServer;
MavESP8266UpdateImp updateStatus;
MavESP8266Log Logger;

//---------------------------------------------------------------------------------
//-- Accessors
class MavESP8266WorldImp : public MavESP8266World
{
public:
    MavESP8266Parameters *getParameters() { return &Parameters; }
    MavESP8266Component *getComponent() { return &Component; }
    MavESP8266Vehicle *getVehicle() { return &Vehicle; }
    MavESP8266GCS *getGCS() { return &GCS; }
    MavESP8266Log *getLogger() { return &Logger; }
};

MavESP8266WorldImp World;

MavESP8266World *getWorld()
{
    return &World;
}

//---------------------------------------------------------------------------------
//-- Wait for a DHCPD client
void wait_for_client()
{
    DEBUG_LOG("Waiting for a client...\n");

    uint8 client_count = wifi_softap_get_station_num();
    while (!client_count)
    {
        ledManager.blinkLED();
        delay(200);
        client_count = wifi_softap_get_station_num();
    }
    ledManager.setLED(ledManager.wifi, ledManager.on);
    ledManager.setLED(ledManager.gcs, ledManager.blink);
    ledManager.setLED(ledManager.air, ledManager.blink);
    DEBUG_LOG("Got %d client(s)\n", client_count);
}

void check_wifi_connected()
{
    if (Parameters.getWifiMode() == WIFI_MODE_AP && !wifi_softap_get_station_num())
    {
        ledManager.setLED(ledManager.wifi, ledManager.blink);
    }
    else if (Parameters.getWifiMode() == WIFI_MODE_AP)
    {
        ledManager.setLED(ledManager.wifi, ledManager.on);
    }

    if (Parameters.getWifiMode() == WIFI_MODE_STA && WiFi.status() != WL_CONNECTED)
    {
        ledManager.setLED(ledManager.wifi, ledManager.doubleBlink);
    }
    else if (Parameters.getWifiMode() == WIFI_MODE_STA && WiFi.status() == WL_CONNECTED)
    {
        ledManager.setLED(ledManager.wifi, ledManager.on);
    }
}

//---------------------------------------------------------------------------------
//-- Reset all parameters whenever the reset gpio pin is active within first 5s of boot
IRAM_ATTR void reset_interrupt()
{
    if (millis() < 5000)
    {
        Parameters.resetToDefaults();
        Parameters.saveAllToEeprom();
        ESP.reset();
    }
    else
    {
        detachInterrupt(GPIO02);
    }
}

void setup_station()
{
    //-- Connect to an existing network
    ledManager.setLED(ledManager.wifi, ledManager.doubleBlink); // Double blink while searching for station
    WiFi.setSleepMode(WIFI_NONE_SLEEP);                         // Aparrently it can go to sleep in station mode
    WiFi.mode(WIFI_STA);
    WiFi.config(Parameters.getWifiStaIP(), Parameters.getWifiStaGateway(), Parameters.getWifiStaSubnet(), 0U, 0U);
    WiFi.begin(Parameters.getWifiStaSsid(), Parameters.getWifiStaPassword());
}

void setup_AP()
{
    ledManager.setLED(ledManager.wifi, ledManager.blink);
    WiFi.mode(WIFI_AP);
    WiFi.encryptionType(AUTH_WPA2_PSK);
    WiFi.softAP(Parameters.getWifiSsid(), Parameters.getWifiPassword(), Parameters.getWifiChannel());
    localIP = WiFi.softAPIP();
}

void setup_wifi()
{
    //-- Boost power to Max
    WiFi.setOutputPower(20.5);
    //-- MDNS
    char mdsnName[256];
    sprintf(mdsnName, "MavEsp8266-%d", localIP[3]);
    MDNS.begin(mdsnName);
    MDNS.addService("http", "tcp", 80);
    //-- Initialize Comm Links
    DEBUG_LOG("Start WiFi Bridge\n");
    DEBUG_LOG("Local IP: %s\n", localIP.toString().c_str());

    Parameters.setLocalIPAddress(localIP);
    IPAddress gcs_ip(localIP);
    gcs_ip[3] = Parameters.getTargetAddress();
    GCS.begin(&Vehicle, gcs_ip);
    //-- Initialize Update Server
    updateServer.begin(&updateStatus);
}

bool connect_wifi()
{
    if (Parameters.getWifiMode() == WIFI_MODE_STA)
    {
        if (WiFi.status() == WL_CONNECTED)
        {
            ledManager.setLED(ledManager.wifi, ledManager.on);
            ledManager.setLED(ledManager.gcs, ledManager.blink);
            localIP = WiFi.localIP();
            WiFi.setAutoReconnect(true);
            setup_wifi();
            return false;
        }
        else if (millis() > 60000)
        {
            //-- Fall back to AP mode if no connection could be established
            WiFi.disconnect(true);
            Parameters.setWifiMode(WIFI_MODE_AP);
            setup_AP();
        }
    }

    if (Parameters.getWifiMode() == WIFI_MODE_AP)
    {
        DEBUG_LOG("Waiting for a client...\n");

        uint8 client_count = wifi_softap_get_station_num();

        if (client_count)
        {
            ledManager.setLED(ledManager.wifi, ledManager.on);
            ledManager.setLED(ledManager.gcs, ledManager.blink);
            DEBUG_LOG("Got %d client(s)\n", client_count);
            setup_wifi();
            return false;
        }
    }
    return true;
}

//---------------------------------------------------------------------------------
//-- Set things up
void setup()
{
    delay(1000);
    Parameters.begin();
    // set up pins for LEDs
    pinMode(12, OUTPUT);
    pinMode(4, OUTPUT);
    pinMode(5, OUTPUT);

    //-- Initialize GPIO02 (Used for "Reset To Factory")
    pinMode(GPIO02, INPUT_PULLUP);
    attachInterrupt(GPIO02, reset_interrupt, FALLING);

    Logger.begin(2048);

    DEBUG_LOG("\nConfiguring access point...\n");
    DEBUG_LOG("Free Sketch Space: %u\n", ESP.getFreeSketchSpace());

    WiFi.disconnect(true);

    // Set LED state when reboot
    ledManager.setLED(ledManager.air, ledManager.off);
    ledManager.setLED(ledManager.air, ledManager.blink);
    ledManager.setLED(ledManager.gcs, ledManager.blink);

    if (Parameters.getWifiMode() == WIFI_MODE_STA)
    {
        setup_station();
    }

    if (Parameters.getWifiMode() == WIFI_MODE_AP)
    {
        setup_AP();
    }

    Vehicle.begin(&GCS);
}

//---------------------------------------------------------------------------------
//-- Main Loop
bool firstConnection = true; // Flag to see if this is the first time connecting to wifi

void loop()
{
    if (firstConnection)
    {
        firstConnection = connect_wifi();
        if (Component.inRawMode())
        {
            Vehicle.readMessageRaw();
        }
        else
        {
            Vehicle.readMessage();
        }
    }
    else
    {
        check_wifi_connected();
        if (Component.inRawMode())
        {
            GCS.readMessageRaw();
            delay(0);
            Vehicle.readMessageRaw();
        }
        else
        {
            GCS.readMessage();
            delay(0);
            Vehicle.readMessage();
        }
        updateServer.checkUpdates();
    }
    ledManager.blinkLED();
    ledManager.doubleBlinkLED();
}
