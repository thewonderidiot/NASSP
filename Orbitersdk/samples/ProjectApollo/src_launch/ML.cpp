/***************************************************************************
  This file is part of Project Apollo - NASSP
  Copyright 2004-2005

  ML vessel

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

#define ORBITER_MODULE

// To force orbitersdk.h to use <fstream> in any compiler version
#pragma include_alias( <fstream.h>, <fstream> )
#include "orbitersdk.h"
#include "stdio.h"
#include "math.h"
#include "nasspsound.h"
#include "soundlib.h"
#include "tracer.h"

#include "ML.h"
#include "nasspdefs.h"
#include "toggleswitch.h"
#include "apolloguidance.h"
#include "csmcomputer.h"
#include "saturn.h"
#include "papi.h"
#include "IUUmbilical.h"
#include "IU_ESE.h"

HINSTANCE g_hDLL;
char trace_file[] = "ProjectApollo ML.log";

// States
#define STATE_VABBUILD		   -2
#define STATE_VABREADY		   -1
#define STATE_ROLLOUT			0
#define STATE_PRELAUNCH			1
#define STATE_CMARM1			2
#define STATE_CMARM2			3
#define STATE_SICINTERTANKARM	4
#define STATE_SICFORWARDARM		5
#define STATE_LIFTOFFSTREAM		6
#define STATE_LIFTOFF			7
#define STATE_POSTLIFTOFF		8

// Pad and VAB coordinates
#define VAB_LON -80.6509353	///\todo fix for Orbiter 2010-P1
#define VAB_LAT 28.5860469

#define PAD_LONA -80.6041122
#define PAD_LATA 28.6083723

#define PAD_LONB -80.6208972
#define PAD_LATB 28.6272345

#define PAD_LV_LONA -80.6041140
#define PAD_LV_LATA 28.6082888

#define PAD_LV_LONB -80.620899
#define PAD_LV_LATB 28.627151


PARTICLESTREAMSPEC liftoffStreamSpec = {
	0, 
	12.0,	// size
	50,		// rate
	120.0,	// velocity
	0.5,	// velocity distribution
	6.0,	// lifetime
	4,		// growthrate
	0.4,	// atmslowdown 
	PARTICLESTREAMSPEC::DIFFUSE,
	PARTICLESTREAMSPEC::LVL_PLIN, 0, 1.0,
	PARTICLESTREAMSPEC::ATM_FLAT, 0.7, 0.7
};


DLLCLBK void InitModule(HINSTANCE hModule) {

	g_hDLL = hModule;
}


DLLCLBK VESSEL *ovcInit(OBJHANDLE hvessel, int flightmodel) {

	return new ML(hvessel, flightmodel);
}


DLLCLBK void ovcExit(VESSEL *vessel) {

	if (vessel) delete (ML*)vessel;
}


ML::ML(OBJHANDLE hObj, int fmodel) : VESSEL2 (hObj, fmodel) {

	firstTimestepDone = false;
	LVName[0] = '\0';
	moveToPadA = false;
	moveToPadB = false;
	moveToVab = false;
	moveLVToPadA = false;
	moveLVToPadB = false;
	touchdownPointHeight = -86.677; // pad height
	hLV = 0;
	state = STATE_ROLLOUT;

	craneProc = 0;
	cmarmProc = 0.00001;
	s1cintertankarmProc = 0;
	s1cforwardarmProc = 0;
	swingarmProc = 0;
	mastProc = 0;

	int i;
	for (i = 0; i < 2; i++) {
		liftoffStream[i] = NULL;
	}
	liftoffStreamLevel = 0;

	soundlib.InitSoundLib(hObj, SOUND_DIRECTORY);

	sat = NULL;

	IuUmb = new IUUmbilical(this);
	IuESE = new IUSV_ESE(IuUmb);
}

ML::~ML() {
	delete IuUmb;
	delete IuESE;
}

void ML::clbkSetClassCaps(FILEHANDLE cfg) {

	SetEmptyMass(100000);
	SetSize(120);

    ClearMeshes();
    ClearExhaustRefs();
    ClearAttExhaustRefs();

	VECTOR3 meshoffset = _V(0,0,0);
    meshindexML = AddMesh(oapiLoadMeshGlobal("ProjectApollo\\Saturn5ML"), &meshoffset);
    //meshindexML = AddMesh(oapiLoadMeshGlobal("ProjectApollo\\LUT"), &meshoffset);
	SetMeshVisibilityMode(meshindexML, MESHVIS_ALWAYS);

	DefineAnimations();

	liftoffStream[0] = AddParticleStream(&liftoffStreamSpec, _V(0, -80, 0), _V(0, 0.4,  1), &liftoffStreamLevel);
	liftoffStream[1] = AddParticleStream(&liftoffStreamSpec, _V(0, -80, 0), _V(0, 0.4, -1), &liftoffStreamLevel);

	SetTouchdownPointHeight(touchdownPointHeight);
}

void ML::clbkPostCreation()
{	
	char buffer[256];

	if (swingarmProc == 0.0)
	{
		double vcount = oapiGetVesselCount();
		for (int i = 0; i < vcount; i++) {
			OBJHANDLE h = oapiGetVesselByIndex(i);
			oapiGetObjectName(h, buffer, 256);
			if (!strcmp(LVName, buffer)) {
				hLV = h;
				Saturn *sat = (Saturn *)oapiGetVesselInterface(hLV);
				if (sat->GetStage() < LAUNCH_STAGE_ONE)
				{
					IuUmb->Connect(sat->GetIU());
				}
			}
		}
	}

	SetAnimation(craneAnim, craneProc);
	SetAnimation(cmarmAnim, cmarmProc);
	SetAnimation(s1cintertankarmAnim, s1cintertankarmProc);
	SetAnimation(s1cforwardarmAnim, s1cforwardarmProc);
	SetAnimation(swingarmAnim, swingarmProc);
	SetAnimation(mastAnim, mastProc);
}

void ML::clbkPreStep(double simt, double simdt, double mjd) {

	ATTACHMENTHANDLE ah;

	if (!firstTimestepDone) DoFirstTimestep();
	
	switch (state) {
	case STATE_VABBUILD:

		if (craneProc < 0.25) {
			craneProc = min(0.25, craneProc + simdt / 600.0);
			SetAnimation(craneAnim, craneProc);
		}
		if (cmarmProc < 0.09) {
			cmarmProc = min(0.09, cmarmProc + simdt / 1000.0);
			SetAnimation(cmarmAnim, cmarmProc);
		}
		if (swingarmProc < 1.0) {
			swingarmProc = min(1.0, swingarmProc + simdt / 200.0);
			SetAnimation(swingarmAnim, swingarmProc);
		}
		if (s1cintertankarmProc < 1.0) {
			s1cintertankarmProc = min(1.0, s1cintertankarmProc + simdt / 200.0);
			SetAnimation(s1cintertankarmAnim, s1cintertankarmProc);
		}
		if (s1cforwardarmProc < 1.0) {
			s1cforwardarmProc = min(1.0, s1cforwardarmProc + simdt / 200.0);
			SetAnimation(s1cforwardarmAnim, s1cforwardarmProc);
		}
		if (mastProc < 1.0) {
			mastProc = min(1.0, mastProc + simdt / 100.0);
			SetAnimation(mastAnim, mastProc);
		}
		break;

	case STATE_VABREADY:
	case STATE_ROLLOUT:

		if (craneProc > 0) {
			craneProc = max(0, craneProc - simdt / 600.0);
			SetAnimation(craneAnim, craneProc);
		}
		if (cmarmProc > 0.00001) {
			cmarmProc = max(0.00001, cmarmProc - simdt / 1000.0);
			SetAnimation(cmarmAnim, cmarmProc);
		}
		if (swingarmProc > 0) {
			swingarmProc = max(0, swingarmProc - simdt / 200.0);
			SetAnimation(swingarmAnim, swingarmProc);
		}
		if (s1cintertankarmProc > 0) {
			s1cintertankarmProc = max(0, s1cintertankarmProc - simdt / 200.0);
			SetAnimation(s1cintertankarmAnim, s1cintertankarmProc);
		}
		if (s1cforwardarmProc > 0) {
			s1cforwardarmProc = max(0, s1cforwardarmProc - simdt / 200.0);
			SetAnimation(s1cforwardarmAnim, s1cforwardarmProc);
		}
		if (mastProc > 0) {
			mastProc = max(0, mastProc - simdt / 100.0);
			SetAnimation(mastAnim, mastProc);
		}

		if (state == STATE_VABREADY) break;

		// Detached from crawler?
		ah = GetAttachmentHandle(true, 0);
		if (GetAttachmentStatus(ah) != NULL) break;
			
		// On pad?
		if ((GetDistanceTo(PAD_LONA, PAD_LATA) > 10.0) && (GetDistanceTo(PAD_LONB, PAD_LATB) > 10.0)) break;

		// T-3h or later?
		if (!hLV) break;
		sat = (Saturn *) oapiGetVesselInterface(hLV);
		if (sat->GetMissionTime() > -3 * 3600) {
			
			if (oapiGetTimeAcceleration() > 1) {
				oapiSetTimeAcceleration(1);
			}
			
			// Detach Saturn
			ah = GetAttachmentHandle(false, 0);
			if (GetAttachmentStatus(ah) != NULL) {
				DetachChild(ah);
				// and move to pad
				if (GetDistanceTo(PAD_LONB, PAD_LATB) < 10.0)
				{
					moveLVToPadB = true;
				}
				else
				{
					moveLVToPadA = true;
				}
				// and notify Saturn
				sat->LaunchVehicleRolloutEnd();
				
			}
			state = STATE_PRELAUNCH;
		}
		break;

	case STATE_PRELAUNCH:
		// Move hammerhead crane, no clue if 60s are OK?
		if (craneProc < 1) {
			craneProc = min(1.0, craneProc + simdt / 60.0);
			SetAnimation(craneAnim, craneProc);
		}

		// T-43min or later?
		if (!hLV) break;
		sat = (Saturn *) oapiGetVesselInterface(hLV);

		sat->ActivatePrelaunchVenting();

		if (sat->GetMissionTime() > -43 * 60) {
			craneProc = 1;
			SetAnimation(craneAnim, craneProc);
			state = STATE_CMARM1;
		}
		break;

	case STATE_CMARM1:
		// Move CM arm 12 deg, no clue if 60s are OK?
		if (cmarmProc < 12.0 / 180.0) {
			cmarmProc = min(12.0 / 180.0, cmarmProc + simdt / 60.0);
			SetAnimation(cmarmAnim, cmarmProc);
		}

		// T-5min or later?
		if (!hLV) break;
		sat = (Saturn *) oapiGetVesselInterface(hLV);

		sat->ActivatePrelaunchVenting();

		if (sat->GetMissionTime() > -5 * 60) {
			cmarmProc = 12.0 / 180.0;
			SetAnimation(cmarmAnim, cmarmProc);
			state = STATE_CMARM2;
		}
		break;

	case STATE_CMARM2:
		// Move CM arm to 180 deg, no clue if 60s are OK?
		if (cmarmProc < 1) {
			cmarmProc = min(1.0, cmarmProc + simdt / 60.0);
			SetAnimation(cmarmAnim, cmarmProc);
		}

		// T-30s or later?
		if (!hLV) break;
		sat = (Saturn *) oapiGetVesselInterface(hLV);

		sat->ActivatePrelaunchVenting();

		if (sat->GetMissionTime() > -30 && !CutoffInterlock()) {
			cmarmProc = 1;
			SetAnimation(cmarmAnim, cmarmProc);
			state = STATE_SICINTERTANKARM;
		}
		break;

	case STATE_SICINTERTANKARM:
		// Move SIC intertank arm
		if (s1cintertankarmProc < 1) {
			s1cintertankarmProc = min(1.0, s1cintertankarmProc + simdt / 13.0);
			SetAnimation(s1cintertankarmAnim, s1cintertankarmProc);
		}

		// T-16.2s or later?
		if (!hLV) break;
		sat = (Saturn *) oapiGetVesselInterface(hLV);

		sat->ActivatePrelaunchVenting();

		if (sat->GetMissionTime() > -16.2 && !CutoffInterlock()) {
			s1cintertankarmProc = 1;
			SetAnimation(s1cintertankarmAnim, s1cintertankarmProc);
			state = STATE_SICFORWARDARM;
		}
		break;

	case STATE_SICFORWARDARM:
		// Move SIC forward arm
		if (s1cforwardarmProc < 1) {
			s1cforwardarmProc = min(1.0, s1cforwardarmProc + simdt / 5.2);
			SetAnimation(s1cforwardarmAnim, s1cforwardarmProc);
		}

		if (!hLV) break;
		sat = (Saturn *) oapiGetVesselInterface(hLV);

		if (sat->GetMissionTime() < -9)
		{
			sat->ActivatePrelaunchVenting();
		}
		else
		{
			sat->DeactivatePrelaunchVenting();
		}

		if (CutoffInterlock())
		{
			sat->SIGSECutoff(true);
		}
		else
		{
			if (sat->GetMissionTime() > -8.9)
			{
				sat->SetSIEngineStart(5);
			}
			if (sat->GetMissionTime() > -8.62)
			{
				sat->SetSIEngineStart(1);
				sat->SetSIEngineStart(3);
			}
			if (sat->GetMissionTime() > -8.2)
			{
				sat->SetSIEngineStart(2);
				sat->SetSIEngineStart(4);
			}

			// T-4.9s or later?
			if (sat->GetMissionTime() > -4.9) {
				s1cforwardarmProc = 1;
				SetAnimation(s1cforwardarmAnim, s1cforwardarmProc);
				state = STATE_LIFTOFFSTREAM;
			}
		}
		break;

	case STATE_LIFTOFFSTREAM:
		if (!hLV) break;
		sat = (Saturn *) oapiGetVesselInterface(hLV);

		if (sat->GetMissionTime() < -2.0)
			liftoffStreamLevel = sat->GetSIThrustLevel()*(sat->GetMissionTime() + 4.9) / 2.9;
		else
			liftoffStreamLevel = sat->GetSIThrustLevel();

		// T-1s or later?
		if (CutoffInterlock())
		{
			sat->SIGSECutoff(true);
		}
		else
		{
			//Hold-down force
			if (sat->GetMissionTime() > -4.0) {
				sat->AddForce(_V(0, 0, -5. * sat->GetFirstStageThrust()), _V(0, 0, 0));
			}

			if (sat->GetMissionTime() > -1) {
				state = STATE_LIFTOFF;
			}
		}

		break;
	
	case STATE_LIFTOFF:
		// Move swingarms
		if (swingarmProc < 1) {
			swingarmProc = min(1.0, swingarmProc + simdt / 5.0);
			SetAnimation(swingarmAnim, swingarmProc);
		}

		// Move masts
		if (mastProc < 1) {
			mastProc = min(1.0, mastProc + simdt / 2.0);
			SetAnimation(mastAnim, mastProc);
		}

		if (!hLV) break;
		sat = (Saturn *)oapiGetVesselInterface(hLV);

		liftoffStreamLevel = sat->GetSIThrustLevel();

		// Disconnect IU Umbilical
		if (CutoffInterlock())
		{
			sat->SIGSECutoff(true);
		}
		else
		{
			// Soft-Release Pin Dragging
			if (sat->GetMissionTime() < 0.5)
			{
				double PinDragFactor = min(1.0, 1.0 - (sat->GetMissionTime() * 2.0));
				sat->AddForce(_V(0, 0, -(sat->GetFirstStageThrust() * PinDragFactor)), _V(0, 0, 0));
			}

			if (Commit()) {
				IuUmb->Disconnect();
			}

			//Cutoff
			if (sat->GetMissionTime() > 6.0 && sat->GetStage() <= PRELAUNCH_STAGE)
			{
				sat->SIGSECutoff(true);
			}

			// T+8s or later?
			if (sat->GetMissionTime() > 8) {
				state = STATE_POSTLIFTOFF;
			}
		}
		break;

	case STATE_POSTLIFTOFF:

		if (!hLV) break;
		sat = (Saturn *) oapiGetVesselInterface(hLV);
		if (sat->GetMissionTime() < 13.0)
			liftoffStreamLevel = sat->GetSIThrustLevel()*(sat->GetMissionTime() - 13.0) / -5.0;
		else
		{
			liftoffStreamLevel = 0;

			//
			// Once the stream is finished, forget about the vessel since we won't be
			// using it again. This prevents a crash if we later delete the vessel.
			//
			hLV = 0;
			sat = 0;
		}
		break;
	}

	//IU ESE
	if (sat)
	{
		IuESE->Timestep(sat->GetMissionTime(), simdt);
	}

	// sprintf(oapiDebugString(), "Dist %f", GetDistanceTo(VAB_LON, VAB_LAT));
}

void ML::clbkPostStep (double simt, double simdt, double mjd) {

	if (!firstTimestepDone) return;

	// Move ML to pad!
	if (moveToPadA) {
		VESSELSTATUS vs;
		GetStatus(vs);

		vs.status = 1;
		vs.vdata[0].x = PAD_LONA * RAD;
		vs.vdata[0].y = PAD_LATA * RAD;
		vs.vdata[0].z = 0.58 * RAD; 
		DefSetState(&vs);
		moveToPadA = false;
	}

	if (moveToPadB) {
		VESSELSTATUS vs;
		GetStatus(vs);

		vs.status = 1;
		vs.vdata[0].x = PAD_LONB * RAD;
		vs.vdata[0].y = PAD_LATB * RAD;
		vs.vdata[0].z = 0.58 * RAD;
		DefSetState(&vs);
		moveToPadB = false;
	}

	// Move ML to VAB, position is hardcoded!
	if (moveToVab) {
		VESSELSTATUS vs;
		GetStatus(vs);

		vs.status = 1;
		vs.vdata[0].x = VAB_LON * RAD;
		vs.vdata[0].y = VAB_LAT * RAD;
		vs.vdata[0].z = 75.04 * RAD; 
		DefSetState(&vs);
		moveToVab = false;
	}

	// Move LV to pad!
	if (moveLVToPadA) {
		if (hLV) {
			VESSELSTATUS vs;
			Saturn *sat = (Saturn *) oapiGetVesselInterface(hLV);
			sat->GetStatus(vs);

			vs.status = 1;
			vs.vdata[0].x = PAD_LV_LONA * RAD;
			vs.vdata[0].y = PAD_LV_LATA * RAD;
			vs.vdata[0].z = 270.0 * RAD; 
			sat->DefSetState(&vs);
		}
		moveLVToPadA = false;
	}
	else if (moveLVToPadB) {
		if (hLV) {
			VESSELSTATUS vs;
			Saturn *sat = (Saturn *)oapiGetVesselInterface(hLV);
			sat->GetStatus(vs);

			vs.status = 1;
			vs.vdata[0].x = PAD_LV_LONB * RAD;
			vs.vdata[0].y = PAD_LV_LATB * RAD;
			vs.vdata[0].z = 270.0 * RAD;
			sat->DefSetState(&vs);
		}
		moveLVToPadB = false;
	}
}

void ML::DoFirstTimestep()
{
	soundlib.SoundOptionOnOff(PLAYCOUNTDOWNWHENTAKEOFF, FALSE);
	soundlib.SoundOptionOnOff(PLAYCABINAIRCONDITIONING, FALSE);
	soundlib.SoundOptionOnOff(PLAYCABINRANDOMAMBIANCE, FALSE);
	soundlib.SoundOptionOnOff(PLAYRADARBIP, FALSE);
	soundlib.SoundOptionOnOff(DISPLAYTIMER, FALSE);

	firstTimestepDone = true;
}

bool ML::Detach() {

	// Is the crawler attached? 
	ATTACHMENTHANDLE ah = GetAttachmentHandle(true, 0);
	if (GetAttachmentStatus(ah) == NULL) return false;

	// Is the pad near?
	if (GetDistanceTo(PAD_LONA, PAD_LATA) < 10.0) {
		
		SetTouchdownPointHeight(-86.677);	// pad height
		moveToPadA = true;
		return true;
	}
	else if (GetDistanceTo(PAD_LONB, PAD_LATB) < 10.0) {

		SetTouchdownPointHeight(-86.677);	// pad height
		moveToPadB = true;
		return true;
	}

	// Is the VAB near?
	if (GetDistanceTo(VAB_LON, VAB_LAT) < 10.0) {
		
		SetTouchdownPointHeight(-71.827);
		moveToVab = true;
		state = STATE_VABREADY;
		return true;
	}
	return false;
}

bool ML::Attach() {

	// Statuscheck
	if (state != STATE_VABREADY && state != STATE_ROLLOUT) 
		return false;

	// Is the crawler detached? 
	ATTACHMENTHANDLE ah = GetAttachmentHandle(true, 0);
	if (GetAttachmentStatus(ah) != NULL) return false;

	// Is the Saturn attached?
	ah = GetAttachmentHandle(false, 0);
	if (GetAttachmentStatus(ah) == NULL) return false;

	state = STATE_ROLLOUT;
	return true;
}

void ML::SetVABBuildState() {

	if (state == STATE_VABREADY) {
		state = STATE_VABBUILD;
	}
}

void ML::SetVABReadyState() {

	if (state == STATE_VABBUILD) {
		state = STATE_VABREADY;
	}
}

bool ML::IsInVAB() {
	
	return (state == STATE_VABREADY || state == STATE_VABBUILD); 
}

double ML::GetDistanceTo(double lon, double lat) {

	double mylon, mylat, myrad, dlon, dlat;

	GetEquPos(mylon, mylat, myrad);
	dlon = (lon / DEG - mylon) * oapiGetSize(GetGravityRef());
	dlat = (lat / DEG - mylat) * oapiGetSize(GetGravityRef());

	return sqrt(dlon * dlon + dlat * dlat);
}

void ML::SetTouchdownPointHeight(double height) {

	touchdownPointHeight = height;
	SetTouchdownPoints(_V(  0, touchdownPointHeight,  10), 
					   _V(-10, touchdownPointHeight, -10), 
					   _V( 10, touchdownPointHeight, -10));
}

void ML::DefineAnimations() {

	// Hammerhead crane
	static UINT crane_groups[2] = {21, 22};
	static MGROUP_ROTATE crane(meshindexML, crane_groups, 2, _V(0, 0, 13.7), _V( 0, 1, 0), (float)(0.5 * PI));
	craneAnim = CreateAnimation(0.0);
	AddAnimationComponent(craneAnim, 0, 1, &crane);

	// CM access arm
	static UINT cmarm_groups[2] = {11, 12};
	static MGROUP_ROTATE cmarm1(meshindexML, cmarm_groups, 2, _V(8.6, 0, 8.6), _V( 0, 1, 0), (float)(3.0 / 180.0 * PI));
	static MGROUP_ROTATE cmarm2(meshindexML, cmarm_groups, 2, _V(8.6, 0, 8.6), _V( 0, 1, 0), (float)(-1.0 * PI));
	cmarmAnim = CreateAnimation(0.0);
	AddAnimationComponent(cmarmAnim, 0,       0.00001, &cmarm1);
	AddAnimationComponent(cmarmAnim, 0.00001, 1,       &cmarm2);

	// SIC intertank arm
	static UINT s1cintertankarm_groups[1] = {13};
	static MGROUP_ROTATE s1cintertankarm(meshindexML, s1cintertankarm_groups, 1, _V(5.2, 0, 5.0), _V( 0, 1, 0), (float)(71.0 / 180.0 * PI));
	s1cintertankarmAnim = CreateAnimation(0.0);
	AddAnimationComponent(s1cintertankarmAnim, 0, 1, &s1cintertankarm);

	// SIC forward arm
	static UINT s1cforwardarm_groups[1] = {14};
	static MGROUP_ROTATE s1cforwardarm(meshindexML, s1cforwardarm_groups, 1, _V(5.3, 0, 6.5), _V( 0, 1, 0), (float)(71.0 / 180.0 * PI));
	s1cforwardarmAnim = CreateAnimation(0.0);
	AddAnimationComponent(s1cforwardarmAnim, 0, 1, &s1cforwardarm);

	// Swingarms
	static UINT swingarm_groups[6] = {15, 16, 17, 18, 19, 20};
	static MGROUP_ROTATE swingarm(meshindexML, swingarm_groups, 6, _V(5.3, 0, 6.5), _V( 0, 1, 0), (float)(71.0 / 180.0 * PI));
	swingarmAnim = CreateAnimation(0.0);
	AddAnimationComponent(swingarmAnim, 0, 1, &swingarm);

	// Masts
	static UINT mast_groups1[1] = {3};
	static UINT mast_groups2[2] = {5, 6};
	static MGROUP_ROTATE mast1(meshindexML, mast_groups1, 1, _V( 9.2, -53.1, 0), _V( 0, 0, 1), (float)(-42.0 / 180.0 * PI));
	static MGROUP_ROTATE mast2(meshindexML, mast_groups2, 2, _V(-9.2, -53.1, 0), _V( 0, 0, 1), (float)(42.0 / 180.0 * PI));
	mastAnim = CreateAnimation(0.0);
	AddAnimationComponent(mastAnim, 0, 1, &mast1);
	AddAnimationComponent(mastAnim, 0, 1, &mast2);
}

void ML::clbkLoadStateEx(FILEHANDLE scn, void *status) {

	char *line;

	while (oapiReadScenario_nextline (scn, line)) {
		if (!strnicmp (line, "STATE", 5)) {
			sscanf (line + 5, "%i", &state);
		} else if (!strnicmp (line, "TOUCHDOWNPOINTHEIGHT", 20)) {
			sscanf (line + 20, "%lf", &touchdownPointHeight);
		} else if (!strnicmp (line, "CRANEPROC", 9)) {
			sscanf (line + 9, "%lf", &craneProc);
		} else if (!strnicmp (line, "CMARMPROC", 9)) {
			sscanf (line + 9, "%lf", &cmarmProc);
		} else if (!strnicmp (line, "S1CINTERTANKARMPROC", 19)) {
			sscanf (line + 19, "%lf", &s1cintertankarmProc);
		} else if (!strnicmp (line, "S1CFORWARDARMPROC", 17)) {
			sscanf (line + 17, "%lf", &s1cforwardarmProc);
		} else if (!strnicmp (line, "SWINGARMPROC", 12)) {
			sscanf (line + 12, "%lf", &swingarmProc);
		} else if (!strnicmp (line, "MASTPROC", 8)) {
			sscanf (line + 8, "%lf", &mastProc);
		} else if (!strnicmp (line, "LVNAME", 6)) {
			strncpy (LVName, line + 7, 64);
		} else {
			ParseScenarioLineEx (line, status);
		}
	}
	SetTouchdownPointHeight(touchdownPointHeight);
}

void ML::clbkSaveState(FILEHANDLE scn) {

	VESSEL2::clbkSaveState(scn);

	oapiWriteScenario_int(scn, "STATE", state);
	papiWriteScenario_double(scn, "TOUCHDOWNPOINTHEIGHT", touchdownPointHeight);
	papiWriteScenario_double(scn, "CRANEPROC", craneProc);
	papiWriteScenario_double(scn, "CMARMPROC", cmarmProc);
	papiWriteScenario_double(scn, "S1CINTERTANKARMPROC", s1cintertankarmProc);
	papiWriteScenario_double(scn, "S1CFORWARDARMPROC", s1cforwardarmProc);
	papiWriteScenario_double(scn, "SWINGARMPROC", swingarmProc);
	papiWriteScenario_double(scn, "MASTPROC", mastProc);
	if (LVName[0])
		oapiWriteScenario_string(scn, "LVNAME", LVName);
}

int ML::clbkConsumeDirectKey(char *kstate) {

	if (KEYMOD_SHIFT(kstate)) {
		return 0; 
	}

/*	ATTACHMENTHANDLE ah = GetAttachmentHandle(false, 0); 
	VECTOR3 pos, dir, rot;
	GetAttachmentParams (ah, pos, dir, rot);

	double step = 0.1;
	if (KEYMOD_CONTROL(kstate))
		step = 0.01;

	if (KEYDOWN (kstate, OAPI_KEY_NUMPAD6)) {
		pos.x -= step;
		SetAttachmentParams (ah, pos, dir, rot);
		RESETKEY(kstate, OAPI_KEY_NUMPAD6);
	}
	if (KEYDOWN (kstate, OAPI_KEY_NUMPAD4)) {
		pos.x += step;
		SetAttachmentParams (ah, pos, dir, rot);
		RESETKEY(kstate, OAPI_KEY_NUMPAD4);
	}
	if (KEYDOWN (kstate, OAPI_KEY_NUMPAD2)) {
		pos.y -= step;
		SetAttachmentParams (ah, pos, dir, rot);
		RESETKEY(kstate, OAPI_KEY_NUMPAD2);
	}
	if (KEYDOWN (kstate, OAPI_KEY_NUMPAD8)) {
		pos.y += step;
		SetAttachmentParams (ah, pos, dir, rot);
		RESETKEY(kstate, OAPI_KEY_NUMPAD8);
	}
	if (KEYDOWN (kstate, OAPI_KEY_NUMPAD1)) {
		pos.z -= step;
		SetAttachmentParams (ah, pos, dir, rot);
		RESETKEY(kstate, OAPI_KEY_NUMPAD1);
	}
	if (KEYDOWN (kstate, OAPI_KEY_NUMPAD3)) {
		pos.z += step;
		SetAttachmentParams (ah, pos, dir, rot);
		RESETKEY(kstate, OAPI_KEY_NUMPAD3);
	}

	sprintf(oapiDebugString(), "x %f y %f z %f", pos.x, pos.y, pos.z);
*/
/*
    VESSELSTATUS vs;
	GetStatus(vs);
	double moveStep = 1.0e-8;
	if (KEYMOD_CONTROL(kstate))
		moveStep = 1.0e-9;

	if (KEYDOWN (kstate, OAPI_KEY_NUMPAD2)) {
		vs.vdata[0].x += moveStep;
		DefSetState(&vs);
		RESETKEY(kstate, OAPI_KEY_NUMPAD2);
	}
	if (KEYDOWN (kstate, OAPI_KEY_NUMPAD4)) {			
		vs.vdata[0].y -= moveStep;
		DefSetState(&vs);
		RESETKEY(kstate, OAPI_KEY_NUMPAD4);			
	}
	if (KEYDOWN (kstate, OAPI_KEY_NUMPAD6)) {			
		vs.vdata[0].y += moveStep;
		DefSetState(&vs);
		RESETKEY(kstate, OAPI_KEY_NUMPAD6);
	}
	if (KEYDOWN (kstate, OAPI_KEY_NUMPAD8)) {
		vs.vdata[0].x -= moveStep;
		DefSetState(&vs);
		RESETKEY(kstate, OAPI_KEY_NUMPAD8);						
	}
	if (KEYDOWN (kstate, OAPI_KEY_NUMPAD1)) {
		vs.vdata[0].z -= 1.0e-4;
		DefSetState(&vs);
		RESETKEY(kstate, OAPI_KEY_NUMPAD1);						
	}
	if (KEYDOWN (kstate, OAPI_KEY_NUMPAD3)) {
		vs.vdata[0].z += 1.0e-4;
		DefSetState(&vs);
		RESETKEY(kstate, OAPI_KEY_NUMPAD3);						
	}
	if (KEYDOWN (kstate, OAPI_KEY_A)) {
		SetTouchdownPoints (_V(0, -GetCOG_elev() + 0.01, 1), _V(-1, -GetCOG_elev() + 0.01, -1), _V(1, -GetCOG_elev() + 0.01, -1));
		RESETKEY(kstate, OAPI_KEY_A);
	}
	if (KEYDOWN (kstate, OAPI_KEY_S)) {
		SetTouchdownPoints (_V(0, -GetCOG_elev() - 0.01, 1), _V(-1, -GetCOG_elev() - 0.01, -1), _V(1, -GetCOG_elev() - 0.01, -1));
		RESETKEY(kstate, OAPI_KEY_S);
	}
	sprintf(oapiDebugString(), "GetCOG_elev %f", GetCOG_elev());
*/
	return 0;
}

int ML::clbkConsumeBufferedKey(DWORD key, bool down, char *kstate) {

	if (KEYMOD_SHIFT(kstate) || KEYMOD_CONTROL(kstate)) {
		return 0;
	}
	return 0;
}

bool ML::CutoffInterlock()
{
	if (!sat) return false;
	return sat->IsEDSUnsafe() || sat->SIStageLogicCutoff();
}

bool ML::Commit()
{
	if (!sat) return false;
	return sat->AllSIEnginesRunning() && sat->GetMissionTime() >= -0.05 && !CutoffInterlock();
}

bool ML::ESEGetCommandVehicleLiftoffIndicationInhibit()
{
	return IuESE->GetCommandVehicleLiftoffIndicationInhibit();
}

bool ML::ESEGetSICOutboardEnginesCantInhibit()
{
	return IuESE->GetSICOutboardEnginesCantInhibit();
}

bool ML::ESEGetAutoAbortInhibit()
{
	return IuESE->GetAutoAbortInhibit();
}

bool ML::ESEGetGSEOverrateSimulate()
{
	return IuESE->GetOverrateSimulate();
}

bool ML::ESEGetEDSPowerInhibit()
{
	return IuESE->GetEDSPowerInhibit();
}

bool ML::ESEPadAbortRequest()
{
	return IuESE->GetEDSPadAbortRequest();
}

bool ML::ESEGetThrustOKIndicateEnableInhibitA()
{
	return IuESE->GetThrustOKIndicateEnableInhibitA();
}

bool ML::ESEGetThrustOKIndicateEnableInhibitB()
{
	return IuESE->GetThrustOKIndicateEnableInhibitB();
}

bool ML::ESEEDSLiftoffInhibitA()
{
	return IuESE->GetEDSLiftoffInhibitA();
}

bool ML::ESEEDSLiftoffInhibitB()
{
	return IuESE->GetEDSLiftoffInhibitB();
}

bool ML::ESEAutoAbortSimulate()
{
	return IuESE->GetAutoAbortSimulate();
}

bool ML::ESEGetSIBurnModeSubstitute()
{
	return IuESE->GetSIBurnModeSubstitute();
}