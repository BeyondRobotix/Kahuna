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
 * @file mavesp8266_gcs.cpp
 * ESP8266 Wifi AP, MavLink UART/UDP Bridge
 *
 * @author Gus Grubba <mavlink@grubba.com>
 */

#include "mavesp8266.h"
#include "mavesp8266_gcs.h"
#include "mavesp8266_parameters.h"
#include "mavesp8266_component.h"
#include "led_manager.h"

//---------------------------------------------------------------------------------
MavESP8266GCS::MavESP8266GCS(LEDManager &ledManager)
    : _udp_port(DEFAULT_UDP_HPORT), _ledManager(ledManager)
{
    _recv_chan = MAVLINK_COMM_1;
    _send_chan = MAVLINK_COMM_0;
    memset(&_message, 0, sizeof(_message));

    // Set up the client list
    for (int i = 0; i < MAX_GCS_CLIENTS; i++)
    {
        _gcs_clients[i].active = false;
        _gcs_clients[i].heard_from = false;
        memset(&_gcs_clients[i].status, 0, sizeof(linkStatus));
    }
}

//---------------------------------------------------------------------------------
//-- Initialize
void MavESP8266GCS::begin(MavESP8266Bridge *forwardTo, IPAddress gcsIP)
{
    MavESP8266Bridge::begin(forwardTo);
    _gcs_broadcast_ip = gcsIP;
    //-- Init variables that shouldn't change unless we reboot
    _udp_port = getWorld()->getParameters()->getWifiUdpHport();
    //-- Start UDP
    _udp.begin(getWorld()->getParameters()->getWifiUdpCport());
}

void MavESP8266GCS::readMessage()
{
    //-- Prune stale clients
    _pruneClients();

    //-- Read UDP
    if (_readMessage())
    {
        //-- If we have a message, forward it
        _forwardTo->sendMessage(&_message);
        memset(&_message, 0, sizeof(_message));
    }

    //-- Update radio status (1Hz)
    if (_isGCSHeard() && (millis() - _last_status_time > 1000))
    {
        delay(0);
        _sendRadioStatus();
        _last_status_time = millis();
    }
}

void MavESP8266GCS::readMessageRaw()
{
    int udp_count = _udp.parsePacket();
    if (udp_count > 0)
    {
        char buf[1024];
        int len = _udp.read(buf, sizeof(buf));

        if (len > 0)
        {
            //-- Update client list
            _addOrUpdateClient(_udp.remoteIP(), _udp.remotePort());

            if (buf[0] == 0x30 && buf[1] == 0x20)
            {
                // reboot command, switch out of raw mode soon
                getWorld()->getComponent()->resetRawMode();
            }

            _forwardTo->sendMessageRaw((uint8_t *)buf, len);
        }
    }
}

int MavESP8266GCS::sendMessage(mavlink_message_t *message, int count)
{
    // Outer loop iterates through the batch of messages from the vehicle
    for (int i = 0; i < count; i++)
    {
        // Buffer to hold the currently serialized MAVLink packet
        char buf[300];
        // Serialize the i-th message from the array
        unsigned len = mavlink_msg_to_send_buffer((uint8_t *)buf, &message[i]);

        for (int j = 0; j < MAX_GCS_CLIENTS; j++)
        {
            if (_gcs_clients[j].active)
            {
                _udp.beginPacket(_gcs_clients[j].ip, _gcs_clients[j].port);
                _udp.write((uint8_t *)(void *)buf, len);
                _udp.endPacket();
            }
        }

        // Also send to the broadcast address to allow new clients to connect
        _udp.beginPacket(_gcs_broadcast_ip, _udp_port);
        _udp.write((uint8_t *)(void *)buf, len);
        _udp.endPacket();
    }

    _status.packets_sent += count;

    // Return the number of merssages processed
    return count;
}

int MavESP8266GCS::sendMessage(mavlink_message_t *message)
{
    return sendMessage(message, 1);
}

int MavESP8266GCS::sendMessageRaw(uint8_t *buffer, int len)
{
    int sentCount = 0;
    //-- Send to all active unicast clients
    for (int i = 0; i < MAX_GCS_CLIENTS; i++)
    {
        if (_gcs_clients[i].active)
        {
            _udp.beginPacket(_gcs_clients[i].ip, _gcs_clients[i].port);
            _udp.write(buffer, len);
            _udp.endPacket();
            sentCount++;
        }
    }

    //-- Also send to the broadcast address
    _udp.beginPacket(_gcs_broadcast_ip, _udp_port);
    _udp.write(buffer, len);
    _udp.endPacket();

    return sentCount;
}

bool MavESP8266GCS::_readMessage()
{
    bool msgReceived = false;
    int udp_count = _udp.parsePacket();
    if (udp_count > 0)
    {
        //-- Get the sender's IP and port
        IPAddress gcs_ip = _udp.remoteIP();
        uint16_t gcs_port = _udp.remotePort();

        while (udp_count--)
        {
            int result = _udp.read();
            if (result >= 0)
            {
                // Parsing
                msgReceived = mavlink_frame_char_buffer(&_rxmsg,
                                                        &_rxstatus,
                                                        result,
                                                        &_message,
                                                        &_mav_status);
                if (msgReceived)
                {
                    //-- Get the client for this message
                    GCSClient *client = _addOrUpdateClient(gcs_ip, gcs_port);
                    _ledManager.setLED(_ledManager.wifi, _ledManager.on);

                    if (client)
                    {
                        client->status.packets_received++;

                        //-- Handle first contact with this specific client
                        if (!client->heard_from)
                        {
                            if (_message.msgid == MAVLINK_MSG_ID_HEARTBEAT)
                            {
                                client->heard_from = true;
                                client->seq_expected = _message.seq + 1;
                                //-- Set global sys/comp ID from first GCS that connects
                                // TODO handle this better
                                if (!_heard_from)
                                {
                                    _heard_from = true;
                                    _ledManager.setLED(_ledManager.gcs, _ledManager.on);
                                    _system_id = _message.sysid;
                                    _component_id = _message.compid;
                                }
                            }
                        }
                        else
                        {
                            //-- Check for packet loss from this specific client
                            uint16_t seq_received = (uint16_t)_message.seq;
                            uint16_t packet_lost_count = 0;
                            if (seq_received < client->seq_expected)
                            {
                                packet_lost_count = (seq_received + 255) - client->seq_expected;
                            }
                            else
                            {
                                packet_lost_count = seq_received - client->seq_expected;
                            }
                            client->seq_expected = _message.seq + 1;
                            client->status.packets_lost += packet_lost_count;
                        }
                    }

                    if (_message.msgid == MAVLINK_MSG_ID_HEARTBEAT)
                    {
                        _last_heartbeat = millis();
                    }

                    if (msgReceived == MAVLINK_FRAMING_BAD_CRC ||
                        msgReceived == MAVLINK_FRAMING_BAD_SIGNATURE)
                    {
                        break;
                    }

                    if (getWorld()->getComponent()->handleMessage(this, &_message))
                    {
                        memset(&_message, 0, sizeof(_message));
                        msgReceived = false;
                        continue;
                    }
                    break;
                }
            }
        }
    }
    if (!msgReceived)
    {
        //-- Checks if ALL GCSs have timed out
        if (_heard_from && !_isGCSHeard())
        {
            _ledManager.setLED(_ledManager.gcs, _ledManager.blink);
            _heard_from = false;
            getWorld()->getLogger()->log("All GCS clients timed out.\n");
        }
    }
    return msgReceived;
}

void MavESP8266GCS::_sendRadioStatus()
{
    linkStatus *st = _forwardTo->getStatus();
    uint8_t rssi = 0;

    if (wifi_get_opmode() == STATION_MODE)
    {
        rssi = (uint8_t)wifi_station_get_rssi();
    }

    //-- Aggregate packet loss from all active GCS clients
    uint32_t total_gcs_packets_received = 0;
    uint32_t total_gcs_packets_lost = 0;
    for (int i = 0; i < MAX_GCS_CLIENTS; i++)
    {
        if (_gcs_clients[i].active)
        {
            total_gcs_packets_received += _gcs_clients[i].status.packets_received;
            total_gcs_packets_lost += _gcs_clients[i].status.packets_lost;
        }
    }

    uint8_t lostVehicleMessages = 100;
    if (st->packets_received > 0)
    {
        lostVehicleMessages = (st->packets_lost * 100) / st->packets_received;
    }

    uint8_t lostGcsMessages = 100;
    if (total_gcs_packets_received > 0)
    {
        lostGcsMessages = (total_gcs_packets_lost * 100) / total_gcs_packets_received;
    }

    //-- Build message
    mavlink_message_t msg;
    mavlink_msg_radio_status_pack_chan(
        _forwardTo->systemID(),
        MAV_COMP_ID_UDP_BRIDGE,
        _forwardTo->_recv_chan,
        &msg,
        rssi,
        0,
        st->queue_status,
        0,
        lostVehicleMessages,
        lostGcsMessages,
        0);

    sendMessage(&msg);
    _status.radio_status_sent++;
}

//---------------------------------------------------------------------------------
//-- Send UDP Single Message
void MavESP8266GCS::_sendSingleUdpMessage(mavlink_message_t *msg)
{
    // Translate message to buffer
    char buf[300];
    unsigned len = mavlink_msg_to_send_buffer((uint8_t *)buf, msg);
    // Send it
    _udp.beginPacket(_gcs_broadcast_ip, _udp_port);
    size_t sent = _udp.write((uint8_t *)(void *)buf, len);
    _udp.endPacket();
    //-- Fibble attempt at not losing data until we get access to the socket TX buffer
    //   status before we try to send.
    if (sent != len)
    {
        delay(1);
        _udp.beginPacket(_gcs_broadcast_ip, _udp_port);
        _udp.write((uint8_t *)(void *)&buf[sent], len - sent);
        _udp.endPacket();
    }
    _status.packets_sent++;
}

//---------------------------------------------------------------------------------
//-- See if we have at least one active GCS client
bool MavESP8266GCS::_isGCSHeard()
{
    for (int i = 0; i < MAX_GCS_CLIENTS; i++)
    {
        if (_gcs_clients[i].active)
        {
            return true;
        }
    }
    return false;
}

//---------------------------------------------------------------------------------
//-- Prune clients that have been silent for too long
void MavESP8266GCS::_pruneClients()
{
    for (int i = 0; i < MAX_GCS_CLIENTS; i++)
    {
        if (_gcs_clients[i].active && (millis() - _gcs_clients[i].last_heartbeat > HEARTBEAT_TIMEOUT))
        {
            getWorld()->getLogger()->log("GCS client timed out: %s\n", _gcs_clients[i].ip.toString().c_str());
            _gcs_clients[i].active = false;
        }
    }
}

//---------------------------------------------------------------------------------
//-- Add a new client or update an existing one
GCSClient *MavESP8266GCS::_addOrUpdateClient(IPAddress ip, uint16_t port)
{
    //-- First, check if we already have this client
    for (int i = 0; i < MAX_GCS_CLIENTS; i++)
    {
        if (_gcs_clients[i].active && _gcs_clients[i].ip == ip && _gcs_clients[i].port == port)
        {
            _gcs_clients[i].last_heartbeat = millis();
            return &_gcs_clients[i]; // Return pointer to existing client
        }
    }

    //-- If not, find an empty slot to add it
    for (int i = 0; i < MAX_GCS_CLIENTS; i++)
    {
        if (!_gcs_clients[i].active)
        {
            _gcs_clients[i].active = true;
            _gcs_clients[i].ip = ip;
            _gcs_clients[i].port = port;
            _gcs_clients[i].last_heartbeat = millis();
            //-- Reset status for new client
            _gcs_clients[i].heard_from = false;
            memset(&_gcs_clients[i].status, 0, sizeof(linkStatus));
            getWorld()->getLogger()->log("New GCS client connected: %s\n", ip.toString().c_str());
            return &_gcs_clients[i]; // Return pointer to new client
        }
    }

    getWorld()->getLogger()->log("GCS client list is full. Could not add: %s\n", ip.toString().c_str());
    return NULL; // Return NULL if list is full
}