/***************************************************************************
  This file is part of Project Apollo - NASSP
  Copyright 2004-2005 Mark Grant, Rodrigo R. M. B. Maia

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
#include "Orbitersdk.h"
#include "stdio.h"
#include "math.h"
#include "soundlib.h"
#include "ioChannels.h"
#include "apolloguidance.h"
#include "AGCBridge.h"

AGCBridge::AGCBridge(char *serial, ApolloGuidance *guidance) {
    FT_STATUS status;

    agc = guidance;
    read_buf_len = 0;
    halted = false;
    for (uint8_t i = 0; i < 4; i++) {
        channels[i] = 077777;
    }
    chan12 = 0;
    chan13 = 0;

    mon_log.open("monitor.log", std::ios::out | std::ios::trunc);

    status = FT_OpenEx(serial, FT_OPEN_BY_SERIAL_NUMBER, &mon_handle);
    if (status != FT_OK)
    {
        mon_log << "Failed to open monitor, RC=" << status << std::endl;
        return;
    }

    mon_log << "Monitor connected!" << std::endl;

    FT_SetTimeouts(mon_handle, 1, 1);
    FT_SetBitMode(mon_handle, 0xFF, 0x00);
    FT_SetBitMode(mon_handle, 0xFF, 0x40);
    FT_SetUSBParameters(mon_handle, 128, 128);
    FT_SetLatencyTimer(mon_handle, 2);
    FT_SetFlowControl(mon_handle, FT_FLOW_RTS_CTS, 0, 0);
    FT_Purge(mon_handle, FT_PURGE_RX);
    FT_Purge(mon_handle, FT_PURGE_TX);
}

AGCBridge::~AGCBridge() {
    send_message(MonitorMessage(MON_GROUP_NASSP, 0, 0));
    send_message(MonitorMessage(MON_GROUP_NASSP, 1, 0));
    send_message(MonitorMessage(MON_GROUP_NASSP, 2, 0));
    send_message(MonitorMessage(MON_GROUP_NASSP, 3, 0));
    Sleep(50);
    FT_Close(mon_handle);
    mon_log << "Monitor disconnected." << std::endl;
    mon_log.close();
}

void AGCBridge::service(double simt) {
    if (dsky_flash && ((simt - dsky_flash_t) >= 0.32)) {
        dsky_flash = false;
        dsky_flash_t = simt;
    } else if (!dsky_flash && ((simt - dsky_flash_t) >= 0.96)) {
        dsky_flash = true;
        dsky_flash_t = simt;
    }

    if (halted) {
        send_message(MonitorMessage(MON_GROUP_CONTROL, MON_CONTROL_PROCEED, 1));
        send_message(MonitorMessage(MON_GROUP_CONTROL, MON_CONTROL_NHALGA, 0));
    }

    send_message(MonitorMessage(MON_GROUP_MON_CHAN, 005));
    send_message(MonitorMessage(MON_GROUP_MON_CHAN, 006));
    send_message(MonitorMessage(MON_GROUP_MON_CHAN, 010));
    send_message(MonitorMessage(MON_GROUP_MON_CHAN, 011));
    send_message(MonitorMessage(MON_GROUP_MON_CHAN, 012));
    send_message(MonitorMessage(MON_GROUP_MON_CHAN, 013));
    send_message(MonitorMessage(MON_GROUP_MON_CHAN, 014));
    send_message(MonitorMessage(MON_GROUP_NASSP, MON_NASSP_THRUST));
    send_message(MonitorMessage(MON_GROUP_NASSP, MON_NASSP_ALTM));
    send_message(MonitorMessage(MON_GROUP_DSKY, MON_DSKY_STATUS));
    read_messages();
}

void AGCBridge::send_message(MonitorMessage &msg) {
    uint8_t msg_buf[MON_DATA_MSG_SIZE];
    uint8_t slipped_buf[MON_DATA_MSG_SIZE * 2 + 2];
    uint8_t length = MON_READ_MSG_SIZE;
    uint8_t slipped_length;
    FT_STATUS status;
    DWORD bytes_written;

    msg_buf[0] = msg.group;
    msg_buf[1] = (msg.address >> 8) & 0xFF;
    msg_buf[2] = msg.address & 0xFF;

    if (msg.has_data) {
        length = MON_DATA_MSG_SIZE;

        msg_buf[0] |= MON_DATA_FLAG;
        msg_buf[3] = (msg.data >> 8) & 0xFF;
        msg_buf[4] = msg.data & 0xFF;
    }

    slipped_length = slip(slipped_buf, msg_buf, length);

    status = FT_Write(mon_handle, slipped_buf, slipped_length, &bytes_written);
    if (status != FT_OK) {
        mon_log << "Failed write, RC=" << status << " (wrote " << bytes_written << ")" << std::endl;
    }
}

void AGCBridge::read_messages() {
    FT_STATUS status;
    DWORD bytes_read;
    uint16_t bytes_used;
    uint16_t read_offset = 0;
    MonitorMessage msg(0, 0);

    status = FT_Read(mon_handle, read_buf + read_buf_len, 4096, &bytes_read);
    if (status != FT_OK) {
        mon_log << "Failed read, RC=" << status << " (read " << bytes_read << ")" << std::endl;
        return;
    }

    read_buf_len += (uint16_t)bytes_read;

    while (read_offset < read_buf_len) {
        boolean found_msg = unslip_message(read_buf + read_offset, read_buf_len - read_offset, &msg, &bytes_used);

        read_offset += bytes_used;
        if (!found_msg) {
            if (read_offset > 0) {
                memmove(read_buf, read_buf + read_offset, read_buf_len - read_offset);
            }
            break;
        }
        handle_message(msg);
    }
    
    read_buf_len -= read_offset;
}

void AGCBridge::handle_message(MonitorMessage &msg) {
    switch (msg.group) {
    case MON_GROUP_MON_CHAN:
        agc->SetOutputChannel(msg.address, msg.data);
        if (msg.address == 012) {
            chan12 = msg.data;
        } else if (msg.address == 013) {
            chan13 = msg.data;
        }
        break;

    case MON_GROUP_DSKY:
        switch (msg.address) {
        case MON_DSKY_STATUS:
            ChannelValue v = msg.data;
            ChannelValue ch163 = 0;
            ChannelValue ch11 = 0;

            ch163[Ch163LightTemp] = v[6];
            if (!dsky_flash) {
                ch163[Ch163LightKbRel] = v[10];
                ch163[Ch163LightOprErr] = v[9];
            }
            ch163[Ch163LightRestart] = v[3];
            ch163[Ch163LightStandby] = v[11];
            if (v[15]) {
                ch163[Ch163FlashVerbNoun] = dsky_flash;
            }
            agc->ProcessChannel163(ch163);

            ch11[LightComputerActivity] = v[14];
            ch11[LightUplink] = v[13];
            agc->ProcessChannel11(ch11);
            break;
        }
        break;

    case MON_GROUP_NASSP:
        if (msg.address == MON_NASSP_THRUST) {
            if ((msg.data & 0x8000) && ((msg.data & 077777) != 077777)) {
                agc->SetOutputChannel(0142, msg.data & 077777);
            }
        } else if (msg.address == MON_NASSP_ALTM) {
            if (msg.data & 0x8000) {
                agc->SetOutputChannel(0143, msg.data & 077777);
            }
        }
    }

}

uint8_t AGCBridge::slip(uint8_t *slipped, uint8_t *buf, uint8_t length) {
    uint8_t n = 0;

    slipped[n++] = SLIP_END;
    for (uint8_t i = 0; i < length; i++) {
        if (buf[i] == SLIP_END) {
            slipped[n++] = SLIP_ESC;
            slipped[n++] = SLIP_ESC_END;
        } else if (buf[i] == SLIP_ESC) {
            slipped[n++] = SLIP_ESC;
            slipped[n++] = SLIP_ESC_ESC;
        } else {
            slipped[n++] = buf[i];
        }
    }
    slipped[n++] = SLIP_END;

    return n;
}

boolean AGCBridge::unslip_message(uint8_t *buf, uint16_t length, MonitorMessage *msg, uint16_t *bytes_used) {
    uint16_t end = 0;
    uint16_t non_end = 0;
    uint16_t n = 0;
    uint8_t msg_buf[MON_DATA_MSG_SIZE];
    boolean escaped = false;

    *bytes_used = 0;
    while ((buf[*bytes_used] != SLIP_END) && (*bytes_used < length)) {
        (*bytes_used)++;
    }

    if (*bytes_used == length) {
        return false;
    }

    for (non_end = *bytes_used + 1; non_end < length; non_end++) {
        if (buf[non_end] != SLIP_END) {
            break;
        }
    }

    if (non_end == length) {
        return false;
    }

    for (end = non_end; end < length; end++) {
        if (buf[end] == SLIP_END) {
            break;
        }
    }

    if (end == length) {
        return false;
    }
 
    for (uint16_t i = *bytes_used + 1; i < end; i++) {
        if (escaped) {
            escaped = false;
            if (buf[i] == SLIP_ESC_END) {
                msg_buf[n++] = SLIP_END;
            } else {
                msg_buf[n++] = SLIP_ESC;
            }
        } else if (buf[i] == SLIP_ESC) {
            escaped = true;
        } else {
            msg_buf[n++] = buf[i];
        }
    }

    *bytes_used = end + 1;

    msg->group = msg_buf[0] & ~MON_DATA_FLAG;
    msg->address = (msg_buf[1] << 8) | msg_buf[2];
    msg->data = (msg_buf[3] << 8) | msg_buf[4];
    msg->has_data = true;

    return true;
}