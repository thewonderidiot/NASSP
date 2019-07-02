/***************************************************************************
  This file is part of Project Apollo - NASSP
  Copyright 2004-2005

  Project Apollo is free software; you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation; either version 2 of the License, or
  (at your option) any later version.

  Project Apollo is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Project Apollo; if not, write to the Free Software
  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

  See http://nassp.sourceforge.net/license/ for more details.

  **************************************************************************/

#if !defined(_PA_AGCBRIDGE_H)
#define _PA_AGCBRIDGE_H

#include <fstream>
#include <cstdint>
#include "ftd2xx.h"

#define SLIP_END     0xC0
#define SLIP_ESC     0xDB
#define SLIP_ESC_END 0xDC
#define SLIP_ESC_ESC 0xDD

#define MON_READ_MSG_SIZE 3
#define MON_DATA_MSG_SIZE 5
#define MON_DATA_FLAG 0x80

#define MON_GROUP_ERASABLE 0x00
#define MON_GROUP_CHANNELS 0x02
#define MON_GROUP_CONTROL  0x20
#define MON_GROUP_MON_CHAN 0x22
#define MON_GROUP_DSKY     0x23
#define MON_GROUP_NASSP    0x26

#define MON_CONTROL_START   0x0000
#define MON_CONTROL_STOP    0x0001
#define MON_CONTROL_PROCEED 0x0003
#define MON_CONTROL_NHALGA  0x0040

#define MON_DSKY_BUTTON  0x0009
#define MON_DSKY_PROCEED 0x000A
#define MON_DSKY_STATUS  0x000B

#define MON_NASSP_PIPAX  0x0004
#define MON_NASSP_THRUST 0x0007
#define MON_NASSP_ALTM   0x0008

#define MON_STOP_T12 0x0001

class ApolloGuidance;

class MonitorMessage {
public:
    MonitorMessage(uint8_t g, uint16_t a) {
        group = g;
        address = a;
        has_data = false;
    }

    MonitorMessage(uint8_t g, uint16_t a, uint16_t d) {
        group = g;
        address = a;
        has_data = true;
        data = d;
    }

    uint8_t group;
    uint16_t address;
    boolean has_data;
    uint16_t data;
};

class AGCBridge {
public:
    AGCBridge(char *serial, ApolloGuidance *guidance);
    ~AGCBridge();
    void send_message(MonitorMessage &msg);
    void service(double simt);

    uint16_t channels[4];
    uint16_t chan12;
    uint16_t chan13;
    boolean halted;

private:
    void read_messages();
    void handle_message(MonitorMessage &msg);
    boolean unslip_message(uint8_t *buf, uint16_t length, MonitorMessage *msg, uint16_t *bytes_used);
    uint8_t slip(uint8_t *slipped, uint8_t *buf, uint8_t length);

    ApolloGuidance *agc;
    FT_HANDLE mon_handle;
    std::ofstream mon_log;
    uint8_t read_buf[4096 + 2*MON_READ_MSG_SIZE + 2];
    uint16_t read_buf_len;
    boolean dsky_flash;
    double dsky_flash_t;
};

#endif // _PA_AGCBRIDGE_H