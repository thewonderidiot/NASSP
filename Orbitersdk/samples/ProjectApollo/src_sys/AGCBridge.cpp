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

#include "AGCBridge.h"

AGCBridge::AGCBridge(char *serial) {
    FT_STATUS status;

    mon_log.open("monitor.log", std::ios::out | std::ios::trunc);

    status = FT_OpenEx(serial, FT_OPEN_BY_SERIAL_NUMBER, &mon_handle);
    if (status != FT_OK)
    {
        mon_log << "Failed to open monitor, RC=" << status << std::endl;
        return;
    }

    mon_log << "Monitor connected!" << std::endl;
}

AGCBridge::~AGCBridge() {
    FT_Close(mon_handle);
    mon_log << "Monitor disconnected." << std::endl;
    mon_log.close();
}