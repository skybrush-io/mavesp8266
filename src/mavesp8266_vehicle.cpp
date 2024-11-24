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
 * @file mavesp8266_vehicle.cpp
 * ESP8266 Wifi AP, MavLink UART/UDP Bridge
 *
 * @author Gus Grubba <mavlink@grubba.com>
 */

#include "mavesp8266.h"
#include "mavesp8266_vehicle.h"
#include "mavesp8266_parameters.h"
#include "mavesp8266_component.h"

//---------------------------------------------------------------------------------
MavESP8266Vehicle::MavESP8266Vehicle()
{
    _recv_chan = MAVLINK_COMM_0;
    _send_chan = MAVLINK_COMM_1;
    _last_fake_heartbeat_sent_at = millis();
    memset(&_last_heartbeat_msg, 0, sizeof(_last_heartbeat_msg));
}

//---------------------------------------------------------------------------------
//-- Initialize
void
MavESP8266Vehicle::begin(MavESP8266Bridge* forwardTo, uint8_t system_id, uint8_t component_id)
{
    _system_id = system_id;
    _component_id = component_id;

    MavESP8266Bridge::begin(forwardTo);
    //-- Start UART connected to UAS
    Serial.begin(getWorld()->getParameters()->getUartBaudRate());
    //-- Swap to TXD2/RXD2 (GPIO015/GPIO013) For ESP12 Only
#ifdef ENABLE_DEBUG
#ifdef ARDUINO_ESP8266_ESP12
    Serial.swap();
#endif
#endif
    // raise serial buffer size (default is 256)
    Serial.setRxBufferSize(4096);
}


//---------------------------------------------------------------------------------
//-- Read MavLink message from UAS
void
MavESP8266Vehicle::readMessage()
{
    if (_readMessage()) {
        _forwardTo->sendMessage(&_msg);
    }
    //-- Update radio status (1Hz)
    if(_heard_from && (millis() - _last_status_time > 1000)) {
        delay(0);
        _last_status_time = millis();
    }
#ifdef SIMULATE_HEARTBEATS_WHEN_POWERED_OFF
    //-- Generate fake heartbeats so the GCS can sense the presence of the
    //-- drone even when the FC is powered down (1Hz)
    if(!_heard_from && (millis() - _last_fake_heartbeat_sent_at > 1000)) {
        delay(0);
        _generateFakeHeartbeat();
        _last_fake_heartbeat_sent_at = millis();
    }
#endif
}

void
MavESP8266Vehicle::readMessageRaw() {
    static uint8_t buf[1024];
    int buf_index = 0;

    while(Serial.available() && buf_index < 300)
    {
        int result = Serial.read();
        if (result >= 0)
        {
            buf[buf_index] = result;
            buf_index++;
        }
    }

    if (buf_index > 0) {
        _forwardTo->sendMessageRaw(buf, buf_index);
    }
}

//---------------------------------------------------------------------------------
//-- Send MavLink message to UAS
int
MavESP8266Vehicle::sendMessage(mavlink_message_t* message) {
    // Translate message to buffer
    char buf[300];
    unsigned len = mavlink_msg_to_send_buffer((uint8_t*)buf, message);
    // Send it
    while (Serial.availableForWrite() < 32) {
        // don't spin in the send loop, wait for 25% of the FIFO to be free
        delay(1);
    }
    Serial.write((uint8_t*)(void*)buf, len);
    _status.packets_sent++;
    return 1;
}

int
MavESP8266Vehicle::sendMessageRaw(uint8_t *buffer, int len) {
    Serial.write(buffer, len);
    //Serial.flush();
    return len;
}

//---------------------------------------------------------------------------------
//-- We have some special status to capture when asked for
linkStatus*
MavESP8266Vehicle::getStatus()
{
    _status.queue_status = 0;
    return &_status;
}

//---------------------------------------------------------------------------------
//-- Returns whether the UAS is likely to be armed
bool
MavESP8266Vehicle::isArmed()
{
    return _armed;
}

//---------------------------------------------------------------------------------
//-- Read MavLink message from UAS
bool
MavESP8266Vehicle::_readMessage()
{
    uint8_t msgReceived = MAVLINK_FRAMING_INCOMPLETE;
    int16_t avail = Serial.available();
    if (avail <= 0 && _non_mavlink_len != 0 && _rxstatus.parse_state <= MAVLINK_PARSE_STATE_IDLE) {
        // flush out the non-mavlink buffer when there is nothing pending. This
        // allows us to gather non-mavlink msgs into a single write
        _forwardTo->sendMessageRaw(_non_mavlink_buffer, _non_mavlink_len);
        _non_mavlink_len = 0;
    }
    while (avail--)
    {
        int result = Serial.read();
        if (result >= 0)
        {
            // Parsing
            uint8_t last_parse_error = _rxstatus.parse_error;
            msgReceived = mavlink_frame_char_buffer(&_rxmsg,
                                                    &_rxstatus,
                                                    result,
                                                    &_msg,
                                                    &_mav_status);
            handle_non_mavlink(result, msgReceived);
            if (last_parse_error != _rxstatus.parse_error) {
                _status.parse_errors++;
            }
            if (msgReceived != MAVLINK_FRAMING_INCOMPLETE) {
                _status.packets_received++;
                //-- Is this the first packet we got?
                if(!_heard_from) {
                    if(_msg.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
                        _heard_from     = true;
                        _component_id   = _msg.compid;
                        _system_id      = _msg.sysid;
                        _seq_expected   = _msg.seq + 1;
                        _last_heartbeat = millis();
                        _last_heartbeat_msg = _msg;
                    }
                } else {
                    if(_msg.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
                        _last_heartbeat = millis();
                        _last_heartbeat_msg = _msg;
                    }
                    _checkLinkErrors(&_msg);
                }

                //-- Update the status of the 'armed' flag if we have a heartbeat
                if (_msg.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
                    uint8_t system_status = mavlink_msg_heartbeat_get_system_status(&_msg);
                    uint8_t base_mode = mavlink_msg_heartbeat_get_base_mode(&_msg);
                    _armed = (
                        (
                            system_status == MAV_STATE_ACTIVE &&
                            (base_mode & MAV_MODE_FLAG_SAFETY_ARMED) != 0
                        ) ||
                        system_status == MAV_STATE_CRITICAL ||
                        system_status == MAV_STATE_EMERGENCY
                    );
                }

                if (msgReceived == MAVLINK_FRAMING_BAD_CRC) {
                    // we don't process messages locally with bad CRC,
                    // but we do forward them, so when new messages
                    // are added we can bridge them
                    break;
                }

#ifdef MAVLINK_FRAMING_BAD_SIGNATURE
                if (msgReceived == MAVLINK_FRAMING_BAD_SIGNATURE) {
                    break;
                }
#endif
                
                //-- Check for message we might be interested
                if(getWorld()->getComponent()->handleMessage(this, &_msg)){
                    //-- Eat message (don't send it to GCS)
                    msgReceived = false;
                    continue;
                }

                break;
            }
        }
    }
    if(!msgReceived) {
        if(_heard_from && (millis() - _last_heartbeat) > HEARTBEAT_TIMEOUT) {
            _heard_from = false;
            _last_fake_heartbeat_sent_at = _last_heartbeat;
            getWorld()->getLogger()->log("Heartbeat timeout from Vehicle\n");
        }
    }
    return msgReceived;
}

//---------------------------------------------------------------------------------
//-- Generate fake heartbeat message on behalf of the vehicle when the FC is off
void
MavESP8266Vehicle::_generateFakeHeartbeat()
{
    mavlink_message_t msg;
	mavlink_status_t *status;
    char* buf;
	uint8_t header[10];

    if (_system_id == 0)
    {
        // We haven't seen a heartbeat from the vehicle yet and we haven't been
        // provided an "assumed" system ID from the outside so we don't know
        // what the system ID should be in the generated packet.
        return;
    }

    // This is a horrible hack here. We cannot use mavlink_msg_heartbeat_pack_chan()
    // for some reason to prepare a "proper" MAVLink message, because if we do
    // then the log downloads suddenly start to slow down. I have no idea why.
    // Luckily the heartbeat packet is simple enough so we can construct it
    // manually.
    buf = _MAV_PAYLOAD_NON_CONST(&msg);

    // Order of encoded fields is:
    // 1) Custom mode
    buf[0] = 0; buf[1] = 0; buf[2] = 0; buf[3] = 0;
    // 2) type
    buf[4] = mavlink_msg_heartbeat_get_type(&_last_heartbeat_msg);
    // 3) autopilot
    buf[5] = mavlink_msg_heartbeat_get_autopilot(&_last_heartbeat_msg);
    // 4) base mode
    buf[6] = 0;
    // 5) system status
    buf[7] = MAV_STATE_FLIGHT_TERMINATION;
    // 6) MAVLink version (yes, 3 is correct)
    buf[8] = 3;

    // Prepare message header
    msg.magic = 0xfd;
    msg.incompat_flags = 0;
    msg.compat_flags = 0;
    msg.sysid = _system_id > 0 ? _system_id : 1;
    msg.compid = _component_id > 0 ? _component_id : 1;
    msg.msgid = MAVLINK_MSG_ID_HEARTBEAT;
    msg.len = MAVLINK_MSG_ID_HEARTBEAT_LEN;

    // Handle sequence number
    status = mavlink_get_channel_status(this->_send_chan);
    msg.seq = status->current_tx_seq;
	status->current_tx_seq = status->current_tx_seq + 1;

	// form the header as a byte array for the crc
	header[0] = msg.magic;
	header[1] = msg.len;
    header[2] = msg.incompat_flags;
    header[3] = msg.compat_flags;
    header[4] = msg.seq;
    header[5] = msg.sysid;
    header[6] = msg.compid;
    header[7] = msg.msgid & 0xFF;
    header[8] = (msg.msgid >> 8) & 0xFF;
    header[9] = (msg.msgid >> 16) & 0xFF;

    // Calculate the CRC
	uint16_t checksum = crc_calculate(&header[1], 9);
	crc_accumulate_buffer(&checksum, buf, msg.len);
	crc_accumulate(MAVLINK_MSG_ID_HEARTBEAT_CRC, &checksum);
    buf[9] = (uint8_t)(checksum & 0xFF);
    buf[10] = (uint8_t)(checksum >> 8);
	msg.checksum = checksum;

    // Now we can send the message
    _forwardTo->sendMessage(&msg);
}
