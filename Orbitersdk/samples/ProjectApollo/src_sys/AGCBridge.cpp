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
	char ser[20];

	agc = guidance;
	halted = false;

	escaped = false;
	msg_bytes = 0;

	for (uint8_t i = 0; i < 0200; i++) {
		if (i >= 030 && i <= 033) {
			channels[i] = 077777;
		} else {
			channels[i] = 0;
		}
	}

	mon_log.open("monitor.log", std::ios::out | std::ios::trunc);

	strncpy(ser, serial, 20);
	for (int i = 0; i < 20; i++) {
		if (ser[i] == '\n' || ser[i] == '\r') {
			ser[i] = 0;
			break;
		}
	}

	status = FT_OpenEx(ser, FT_OPEN_BY_SERIAL_NUMBER, &mon_handle);
	if (status != FT_OK)
	{
		mon_log << "Failed to open monitor, RC=" << status << std::endl;
		return;
	}

	mon_log << "Monitor connected!" << std::endl;

	FT_SetTimeouts(mon_handle, 1, 1);
	FT_SetBitMode(mon_handle, 0xFF, 0x00);
	FT_SetBitMode(mon_handle, 0xFF, 0x40);
	FT_SetUSBParameters(mon_handle, 16, 16);
	FT_SetLatencyTimer(mon_handle, 1);
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
		resume();
	}

	send_message(MonitorMessage(MON_GROUP_MON_CHAN, 005));
	send_message(MonitorMessage(MON_GROUP_MON_CHAN, 006));
	send_message(MonitorMessage(MON_GROUP_MON_CHAN, 010));
	send_message(MonitorMessage(MON_GROUP_MON_CHAN, 011));
	send_message(MonitorMessage(MON_GROUP_MON_CHAN, 012));
	send_message(MonitorMessage(MON_GROUP_MON_CHAN, 013));
	send_message(MonitorMessage(MON_GROUP_MON_CHAN, 014));
	send_message(MonitorMessage(MON_GROUP_NASSP, MON_NASSP_CDUXCMD));
	send_message(MonitorMessage(MON_GROUP_NASSP, MON_NASSP_CDUYCMD));
	send_message(MonitorMessage(MON_GROUP_NASSP, MON_NASSP_CDUZCMD));
	send_message(MonitorMessage(MON_GROUP_NASSP, MON_NASSP_CDUTCMD));
	send_message(MonitorMessage(MON_GROUP_NASSP, MON_NASSP_CDUSCMD));
	send_message(MonitorMessage(MON_GROUP_NASSP, MON_NASSP_THRUST));
	send_message(MonitorMessage(MON_GROUP_NASSP, MON_NASSP_ALTM));
	send_message(MonitorMessage(MON_GROUP_DSKY, MON_DSKY_STATUS));
	read_messages();
}

void AGCBridge::halt() {
	send_message(MonitorMessage(MON_GROUP_CONTROL, MON_CONTROL_NHALGA, 1));
	send_message(MonitorMessage(MON_GROUP_CONTROL, MON_CONTROL_STOP, MON_STOP_NISQ));
	halted = true;
}

void AGCBridge::resume() {
	send_message(MonitorMessage(MON_GROUP_CONTROL, MON_CONTROL_STOP, 0));
	send_message(MonitorMessage(MON_GROUP_CONTROL, MON_CONTROL_PROCEED, 1));
	send_message(MonitorMessage(MON_GROUP_CONTROL, MON_CONTROL_NHALGA, 0));
	halted = false;
}

void AGCBridge::restart() {
	send_message(MonitorMessage(MON_GROUP_CONTROL, MON_CONTROL_START, 1));
}

void AGCBridge::load_rom(int16_t fixed[40][1024], uint32_t parities[1280]) {
	for (uint16_t bank = 0; bank < 40; bank++) {
		for (uint16_t s = 0; s < 1024; s++) {
			uint16_t faddr = (bank * 1024) + s;
			uint16_t word = (uint16_t)fixed[bank][s] << 1;
			uint16_t parity = (parities[faddr / 32] >> (faddr % 32)) & 0x1;
			mon_log << std::oct << faddr << " = " << (word | parity) << std::endl;
			send_message(MonitorMessage(MON_GROUP_SIM_FIXED, faddr, word | parity));
		}
		Sleep(50);
	}
	send_message(MonitorMessage(MON_GROUP_CONTROL, MON_CONTROL_CRS_BANK_EN0, 0xFFFF));
	send_message(MonitorMessage(MON_GROUP_CONTROL, MON_CONTROL_CRS_BANK_EN1, 0xFFFF));
	send_message(MonitorMessage(MON_GROUP_CONTROL, MON_CONTROL_CRS_BANK_EN2, 0xFFFF));
	send_message(MonitorMessage(MON_GROUP_CONTROL, MON_CONTROL_CRS_BANK_EN3, 0xFFFF));
}

void AGCBridge::simulate_erasable() {
	send_message(MonitorMessage(MON_GROUP_CONTROL, MON_CONTROL_EMS_BANK_EN, 0xFF));
}

void AGCBridge::set_erasable(int bank, int address, int value) {
	// FIXME: Figure out why I need to force a stop to prevent restarts...
	send_message(MonitorMessage(MON_GROUP_SIM_ERASABLE, bank * 256 + address, value << 1));
}

void AGCBridge::pulse_pipa(int reg_pipa, int pulses) {
	if (pulses < 0) {
		pulses = (-pulses) ^ 077777;
	}
	// FIXME: Figure out why I need to force a stop to prevent restarts...
	send_message(MonitorMessage(MON_GROUP_NASSP, MON_NASSP_PIPAX + (reg_pipa - RegPIPAX), pulses));
}

void AGCBridge::set_input_channel(int channel, int value) {
	channels[channel] = value;

	if (channel == 015) {
		send_message(MonitorMessage(MON_GROUP_DSKY, MON_DSKY_BUTTON, value));
	} else if ((channel >= 030) && (channel <= 033)) {
		send_message(MonitorMessage(MON_GROUP_NASSP, channel - 030, 0x8000 | value));
	}
}

void AGCBridge::set_output_channel(int channel, int value) {
	channels[channel] = value;
	send_message(MonitorMessage(MON_GROUP_CHANNELS, channel, ((value << 1) & 0x8000) | value));
}

int AGCBridge::get_channel_value(int channel) {
	return channels[channel];
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
	uint8_t read_buf[4096];

	status = FT_Read(mon_handle, read_buf, sizeof(read_buf), &bytes_read);
	if (status != FT_OK) {
		mon_log << "Failed read, RC=" << status << " (read " << bytes_read << ")" << std::endl;
		return;
	}

	process_bytes(read_buf, (uint16_t)bytes_read);
}

void AGCBridge::handle_message(MonitorMessage &msg) {
	switch (msg.group) {
	case MON_GROUP_MON_CHAN:
		channels[msg.address] = msg.data;
		agc->SetOutputChannel(msg.address, msg.data);
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
			agc->SetOutputChannel(0163, ch163);

			ch11[LightComputerActivity] = v[14];
			ch11[LightUplink] = v[13];
			agc->SetOutputChannel(011, ch11);
			break;
		}
		break;

	case MON_GROUP_NASSP:
		bool new_value = msg.data & 0x8000;
		uint16_t data = msg.data & 077777;
		uint16_t channel;

		switch (msg.address) {
		case MON_NASSP_CDUXCMD:
			channel = 0174;
			if (data & 040000) {
				data ^= 037777;
			}
			break;

		case MON_NASSP_CDUYCMD:
			channel = 0175;
			if (data & 040000) {
				data ^= 037777;
			}
			break;

		case MON_NASSP_CDUZCMD:
			channel = 0176;
			if (data & 040000) {
				data ^= 037777;
			}
			break;

		case MON_NASSP_CDUTCMD:
			channel = 0141;
			break;

		case MON_NASSP_CDUSCMD:
			channel = 0140;
			break;

		case MON_NASSP_THRUST:
			channel = 0142;
			break;

		case MON_NASSP_ALTM:
			channel = 0143;
			break;

		default:
			return;
		}

		if (new_value && (data != 0) && (data != 077777)) {
			mon_log << std::oct << channel << ": " << data << std::endl;
			channels[channel] = data;
			agc->SetOutputChannel(channel, data);
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

void AGCBridge::process_bytes(uint8_t *buf, uint16_t length) {
	MonitorMessage msg(0, 0);

	for (uint16_t i = 0; i < length; i++) {
		if (buf[i] == SLIP_END) {
			if (!escaped && (msg_bytes == sizeof(msg_buf))) {
				msg.group = msg_buf[0] & ~MON_DATA_FLAG;
				msg.address = (msg_buf[1] << 8) | msg_buf[2];
				msg.data = (msg_buf[3] << 8) | msg_buf[4];
				msg.has_data = true;

				handle_message(msg);
			}
			msg_bytes = 0;
		} else if (msg_bytes < sizeof(msg_buf)) {
			if (!escaped && (buf[i] == SLIP_ESC)) {
				escaped = true;
			} else if (escaped) {
				escaped = false;
				if (buf[i] == SLIP_ESC_END) {
					msg_buf[msg_bytes++] = SLIP_END;
				} else if (buf[i] == SLIP_ESC_ESC) {
					msg_buf[msg_bytes++] = SLIP_ESC;
				} else {
					msg_bytes = 0;
				}
			} else {
				msg_buf[msg_bytes++] = buf[i];
			}
		} else {
			escaped = false;
			msg_bytes = 0;
		}
	}
}
