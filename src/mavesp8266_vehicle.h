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
 * @file mavesp8266_vehicle.h
 * ESP8266 Wifi AP, MavLink UART/UDP Bridge
 *
 * @author Gus Grubba <mavlink@grubba.com>
 */

#ifndef MAVESP8266_VEHICLE_H
#define MAVESP8266_VEHICLE_H

#include "mavesp8266.h"

class MavESP8266Vehicle : public MavESP8266Bridge {
public:
    MavESP8266Vehicle();

    void    begin           (MavESP8266Bridge* forwardTo, IPAddress ownIP, uint8_t system_id = 0, uint8_t component_id = 0);
    IPAddress getIPAddress          () { return _ip; }
    void    readMessage     ();
    void    readMessageRaw  ();
    int     sendMessage     (mavlink_message_t* message);
    int     sendMessageRaw   (uint8_t *buffer, int len);
    linkStatus* getStatus   ();
    bool    isArmed         ();

private:
    void    _generateFakeHeartbeat();
    bool    _readMessage    ();
    void    _send_pending();

private:
    IPAddress               _ip;
    bool                    _armed;
    unsigned long           _queue_time;
    mavlink_message_t       _msg;
    mavlink_message_t       _last_heartbeat_msg;
    uint32_t                _last_fake_heartbeat_sent_at;
};

#endif
