/***************************************************************************
This file is part of Project Apollo - NASSP
Copyright 2019

Electrical Support Equipment for the Instrument Unit (Header)

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

#pragma once

class IUUmbilical;

#define LCC_EDS_MODE_OFF 0
#define LCC_EDS_MODE_MONITOR 1
#define LCC_EDS_MODE_TEST 2
#define LCC_EDS_MODE_LAUNCH 3

class IU_ESE
{
public:
	IU_ESE(IUUmbilical *IuUmb);

	void Timestep(double MissionTime, double simdt);
	void SaveState(FILEHANDLE scn);
	void LoadState(FILEHANDLE scn);

	bool GetCommandVehicleLiftoffIndicationInhibit() { return CommandVehicleLiftoffIndicationInhibit; }
	bool GetAutoAbortInhibit() { return AutoAbortInhibit; }
	bool GetOverrateSimulate() { return OverrateSimulate; }
	bool GetThrustOKIndicateEnableInhibitA() { return ThrustOKIndicateEnableInhibitA; }
	bool GetThrustOKIndicateEnableInhibitB() { return ThrustOKIndicateEnableInhibitB; }
	bool GetEDSLiftoffInhibitA() { return EDSLiftoffInhibitA; }
	bool GetEDSLiftoffInhibitB() { return EDSLiftoffInhibitB; }
	bool GetEDSPadAbortRequest() { return PadAbortRequest; }
	bool GetEDSPowerInhibit() { return EDSPowerInhibit; }
	bool GetAutoAbortSimulate() { return AutoAbortSimulate; }
	bool GetSIBurnModeSubstitute() { return SIBurnModeSubstitute; }
protected:

	void SetEDSMode(int mode);

	bool CommandVehicleLiftoffIndicationInhibit;
	bool AutoAbortInhibit;
	bool OverrateSimulate;
	bool ThrustOKIndicateEnableInhibitA;
	bool ThrustOKIndicateEnableInhibitB;
	bool EDSLiftoffInhibitA;
	bool EDSLiftoffInhibitB;
	bool PadAbortRequest;
	bool EDSPowerInhibit;
	bool AutoAbortSimulate;
	bool SIBurnModeSubstitute;

	double LastMissionTime = 0.0;

	IUUmbilical *Umbilical;
};

class IUSV_ESE : public IU_ESE
{
public:
	IUSV_ESE(IUUmbilical *IuUmb);

	bool GetSICOutboardEnginesCantInhibit() { return SICOutboardEnginesCantInhibit; }

	void SetSICOutboardEnginesCantInhibit(bool set) { SICOutboardEnginesCantInhibit = set; }
protected:
	bool SICOutboardEnginesCantInhibit;
};