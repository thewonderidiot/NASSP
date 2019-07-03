/***************************************************************************
This file is part of Project Apollo - NASSP
Copyright 2017

Saturn Launch Vehicle Data Adapter (Header)

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

class IU;

#define SWITCH_SELECTOR_IU 0
#define SWITCH_SELECTOR_SI 1
#define SWITCH_SELECTOR_SII 2
#define SWITCH_SELECTOR_SIVB 3

class LVDA
{
public:
	LVDA();
	void Init(IU* i);
	void SwitchSelector(int stage, int channel);
	void SetFCCAttitudeError(VECTOR3 atterr);
	VECTOR3 GetLVIMUAttitude();
	void ZeroLVIMUPIPACounters();
	void ZeroLVIMUCDUs();
	void ReleaseLVIMUCDUs();
	void ReleaseLVIMU();
	void DriveLVIMUGimbals(double x, double y, double z);
	VECTOR3 GetLVIMUPIPARegisters();
	bool GetLVIMUFailure();
	bool GetGuidanceReferenceFailure();

	bool GeneralizedSwitchSelector(int stage, int channel);
	bool TimebaseUpdate(double dt);
	bool LMAbort();
	bool RestartManeuverEnable();
	bool InhibitAttitudeManeuver();
	bool Timebase8Enable();
	bool EvasiveManeuverEnable();
	bool ExecuteCommManeuver();
	bool SIVBIULunarImpact(double tig, double dt, double pitch, double yaw);
	bool LaunchTargetingUpdate(double V_T, double R_T, double theta_T, double inc, double dsc, double dsc_dot, double t_grr0);

	//LVDC Input Discretes and Interrupts

	bool GetSIInboardEngineOut();
	bool GetSIOutboardEngineOut();
	bool GetSICInboardEngineCutoff();
	bool GetSIIEngineOut();
	bool GetCMCSIVBIgnitionSequenceStart();
	bool GetCMCSIVBCutoff();
	bool GetCMCSIVBTakeover();
	bool SIVBInjectionDelay();
	bool SCInitiationOfSIISIVBSeparation();
	bool GetSIIPropellantDepletionEngineCutoff();
	bool SpacecraftSeparationIndication();
	bool GetSIVBEngineOut();
	bool GetSIPropellantDepletionEngineCutoff();
	bool SIBLowLevelSensorsDry();
	bool GetLiftoff();

	//Not real LVDA functions
	void TLIBegun();
	void TLIEnded();
	void SwitchSelectorOld(int chan);
	double GetMissionTime();
	int GetStage();
	void SetStage(int stage);
	int GetApolloNo();
	void GetRelativePos(VECTOR3 &v);
	void GetRelativeVel(VECTOR3 &v);
	bool GetSCControlPoweredFlight();
protected:
	IU *iu;
};