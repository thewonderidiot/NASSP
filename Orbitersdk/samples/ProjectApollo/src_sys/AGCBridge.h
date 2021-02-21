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

#define MON_GROUP_ERASABLE     0x00
#define MON_GROUP_CHANNELS     0x02
#define MON_GROUP_SIM_ERASABLE 0x10
#define MON_GROUP_SIM_FIXED    0x11
#define MON_GROUP_CONTROL      0x20
#define MON_GROUP_MON_CHAN     0x22
#define MON_GROUP_DSKY         0x23
#define MON_GROUP_NASSP        0x26

#define MON_CONTROL_START        0x0000
#define MON_CONTROL_STOP         0x0001
#define MON_CONTROL_PROCEED      0x0003
#define MON_CONTROL_CRS_BANK_EN0 0x001A
#define MON_CONTROL_CRS_BANK_EN1 0x001B
#define MON_CONTROL_CRS_BANK_EN2 0x001C
#define MON_CONTROL_CRS_BANK_EN3 0x001D
#define MON_CONTROL_EMS_BANK_EN  0x001E
#define MON_CONTROL_NHALGA       0x0040

#define MON_DSKY_BUTTON     0x0009
#define MON_DSKY_PROCEED    0x000A
#define MON_DSKY_STATUS     0x000B
#define MON_DSKY_NAV_BUTTON 0x000C

#define MON_NASSP_CHAN10  0x0000
#define MON_NASSP_CHAN30  0x0001
#define MON_NASSP_CHAN31  0x0002
#define MON_NASSP_CHAN32  0x0003
#define MON_NASSP_CHAN33  0x0004
#define MON_NASSP_TLOSS_W 0x0010
#define MON_NASSP_TLOSS_T 0x0011
#define MON_NASSP_PIPAX   0x0020
#define MON_NASSP_PIPAY   0x0021
#define MON_NASSP_PIPAZ   0x0022
#define MON_NASSP_CDUXCMD 0x0030
#define MON_NASSP_CDUYCMD 0x0031
#define MON_NASSP_CDUZCMD 0x0032
#define MON_NASSP_CDUTCMD 0x0033
#define MON_NASSP_CDUSCMD 0x0034
#define MON_NASSP_THRUST  0x0035
#define MON_NASSP_ALTM    0x0036

#define MON_STOP_NISQ 0x0002

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
	void halt();
	void resume();
	void restart();
	void load_rom(int16_t fixed[40][1024], uint32_t parities[1280]);
	void simulate_erasable();
	void set_erasable(int bank, int address, int value);
	void set_input_channel(int channel, int value);
	void set_output_channel(int channel, int value);
	int get_channel_value(int channel);
	void pulse_pipa(int reg_pipa, int pulses);
	void set_tloss_wts(uint16_t wts);
	void set_tloss_t12s(uint16_t t12s);
	void service(double simt);

private:
	void send_message(MonitorMessage &msg);
	void read_messages();
	void handle_message(MonitorMessage &msg);
	void process_bytes(uint8_t *buf, uint16_t length);
	uint8_t slip(uint8_t *slipped, uint8_t *buf, uint8_t length);

	ApolloGuidance *agc;
	FT_HANDLE mon_handle;
	std::ofstream mon_log;
	boolean escaped;
	uint8_t msg_bytes;
	uint8_t msg_buf[MON_DATA_MSG_SIZE];
	boolean dsky_flash;
	double dsky_flash_t;
	uint16_t channels[0200];
	boolean halted;
};

#endif // _PA_AGCBRIDGE_H
