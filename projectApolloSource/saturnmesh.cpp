/***************************************************************************
  This file is part of Project Apollo - NASSP
  Copyright 2004-2005 Jean-Luc Rocca-Serra, Mark Grant

  ORBITER vessel module: generic Saturn base class
  Saturn mesh code

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

  **************************** Revision History ****************************
  *	$Log$
  *	Revision 1.43  2006/05/06 06:00:35  jasonims
  *	No more venting our Astronauts into space...and no more LRV popping out of an Astronauts pocket....well sorta.
  *	
  *	Revision 1.42  2006/05/01 03:33:22  jasonims
  *	New CM and all the fixin's....
  *	
  *	Revision 1.41  2006/04/25 20:21:09  jasonims
  *	More Mesh Offset Updates.... Including removing extra code thanks to CM, Hatch, and Crew all aligned on same axis.  Only one Mesh offset required for all three
  *	
  *	Revision 1.40  2006/04/25 19:25:08  jasonims
  *	More Mesh Offset Updates.... Including removing extra code thanks to CM, Hatch, and Crew all aligned on same axis.  Only one Mesh offset required for all three
  *	
  *	Revision 1.39  2006/04/17 19:12:27  movieman523
  *	Removed some unused switches.
  *	
  *	Revision 1.38  2006/04/06 19:32:50  movieman523
  *	More Apollo 13 support.
  *	
  *	Revision 1.37  2006/04/05 19:48:05  movieman523
  *	Added low-res SM RCS meshes and updated Apollo 13.
  *	
  *	Revision 1.36  2006/04/04 22:00:54  jasonims
  *	Apollo Spacecraft Mesh offset corrections and SM Umbilical Animation.
  *	
  *	Revision 1.35  2006/03/30 01:59:37  movieman523
  *	Added RCS to SM DLL.
  *	
  *	Revision 1.34  2006/03/30 00:14:47  movieman523
  *	First pass at SM DLL.
  *	
  *	Revision 1.33  2006/03/29 19:06:50  movieman523
  *	First support for new SM.
  *	
  *	Revision 1.32  2006/01/24 13:57:21  tschachim
  *	Smoother staging with more eye-candy.
  *	
  *	Revision 1.31  2006/01/15 02:38:59  movieman523
  *	Moved CoG and removed phantom thrusters. Also delete launch site when we get a reasonable distance away.
  *	
  *	Revision 1.30  2006/01/12 14:49:35  tschachim
  *	Bugfix
  *	
  *	Revision 1.29  2006/01/11 02:16:25  movieman523
  *	Added RCS propellant quantity gauge.
  *	
  *	Revision 1.28  2006/01/10 23:45:35  movieman523
  *	Revised RCS ISP and thrust to historical values.
  *	
  *	Revision 1.27  2006/01/10 23:20:51  movieman523
  *	SM RCS is now enabled per quad.
  *	
  *	Revision 1.26  2006/01/08 17:11:41  movieman523
  *	Added seperation particles to SII/SIVb sep.
  *	
  *	Revision 1.25  2006/01/08 04:00:24  movieman523
  *	Added first two engineering cameras.
  *	
  *	Revision 1.24  2006/01/05 12:02:26  tschachim
  *	Fixed SIVB separation offset (hopefully)
  *	
  *	Revision 1.23  2006/01/04 23:06:03  movieman523
  *	Moved meshes into ProjectApollo directory and renamed a few.
  *	
  *	Revision 1.22  2006/01/04 19:51:54  movieman523
  *	Updated config file names.
  *	
  *	Revision 1.21  2005/12/28 16:19:10  movieman523
  *	Should now be getting all config files from ProjectApollo directory.
  *	
  *	Revision 1.20  2005/11/24 01:07:54  movieman523
  *	Removed code for panel lights which were being set incorrectly. Plus a bit of tidying.
  *	
  *	Revision 1.19  2005/11/21 23:08:15  movieman523
  *	Moved more mesh files into the ProjectApollo directory.
  *	
  *	Revision 1.18  2005/10/19 11:41:43  tschachim
  *	Improved logging.
  *	
  *	Revision 1.17  2005/10/11 16:42:01  tschachim
  *	Renamed LPswitch5.
  *	
  *	Revision 1.16  2005/08/24 00:30:00  movieman523
  *	Revised CM RCS code, and removed a load of switches that aren't used anymore.
  *	
  *	Revision 1.15  2005/08/21 22:21:00  movieman523
  *	Fixed SM RCS and activated SIVB RCS at all times for now.
  *	
  *	Revision 1.14  2005/08/20 11:14:52  movieman523
  *	Added Rot Contr Pwr switches and removed a number of old switches which aren't used anymore.
  *	
  *	Revision 1.13  2005/08/15 19:25:03  movieman523
  *	Added CSM attitude control switches and removed old ones.
  *	
  *	Revision 1.12  2005/08/10 21:54:04  movieman523
  *	Initial IMU implementation based on 'Virtual Apollo' code.
  *	
  *	Revision 1.11  2005/08/01 19:07:47  movieman523
  *	Genericised code to deal with SM destruction on re-entry, and did some tidying up of Saturn 1b code.
  *	
  *	Revision 1.10  2005/07/31 01:43:13  movieman523
  *	Added CM and SM fuel and empty mass to scenario file and adjusted masses to more accurately match reality.
  *	
  *	Revision 1.9  2005/06/06 12:32:08  tschachim
  *	New switch
  *	
  *	Revision 1.8  2005/03/16 17:30:41  yogenfrutz
  *	corrected missing crew in csm stage
  *	
  *	Revision 1.7  2005/03/12 20:51:30  chode99
  *	Reentry airfoil is now deleted when first drogue opens.
  *	By not deleting, the aerodynamics of the chutes were ignored.
  *	Also tweaked the drag of the chutes a bit to match the real velocities.
  *	
  *	Revision 1.6  2005/03/09 05:05:00  chode99
  *	Fixed CSM thruster positions in SetCSM2Stage
  *	
  *	Revision 1.5  2005/03/03 17:58:43  tschachim
  *	panel handling for generic cockpit
  *	
  *	Revision 1.4  2005/02/20 20:20:45  chode99
  *	Changed touchdown points for recovery stage so it is also "above water".
  *	
  *	Revision 1.3  2005/02/20 05:24:58  chode99
  *	Changes to implement realistic CM aerodynamics. Created callback function "CoeffFunc" in Saturn1b.cpp and Saturn5.cpp. Substituted CreateAirfoil for older lift functions.
  *	
  *	Revision 1.2  2005/02/19 19:32:55  chode99
  *	Adjusted touchdown points in splashdown stage so it is no longer "underwater".
  *	
  *	Revision 1.1  2005/02/11 12:54:07  tschachim
  *	Initial version
  *	
  **************************************************************************/

#include "Orbitersdk.h"
#include <stdio.h>
#include <math.h>
#include "OrbiterSoundSDK3.h"
#include "soundlib.h"

#include "resource.h"

#include "nasspdefs.h"
#include "nasspsound.h"

#include "toggleswitch.h"
#include "apolloguidance.h"
#include "dsky.h"
#include "csmcomputer.h"
#include "IMU.h"

#include "saturn.h"
#include "tracer.h"

MESHHANDLE hSM;
MESHHANDLE hSMRCS;
MESHHANDLE hSMRCSLow;
MESHHANDLE hSMSPS;
MESHHANDLE hSMPanel1;
MESHHANDLE hSMPanel2;
MESHHANDLE hSMPanel3;
MESHHANDLE hSMPanel4;
MESHHANDLE hSMPanel5;
MESHHANDLE hSMPanel6;
MESHHANDLE hSMhga;
MESHHANDLE hSMCRYO;
MESHHANDLE hSMSIMBAY;
MESHHANDLE hCM;
MESHHANDLE hCM2;
MESHHANDLE hCMP;
MESHHANDLE hCMInt;
MESHHANDLE hCMVC;
MESHHANDLE hCREW;
MESHHANDLE hFHO;
MESHHANDLE hFHC;
MESHHANDLE hCM2B;
MESHHANDLE hprobe;
MESHHANDLE hprobeext;
MESHHANDLE hCMBALLOON;
MESHHANDLE hCRB;
MESHHANDLE hApollochute;
MESHHANDLE hCMB;
MESHHANDLE hChute30;
MESHHANDLE hChute31;
MESHHANDLE hChute32;
MESHHANDLE hFHC2;
MESHHANDLE hsat5tower;
MESHHANDLE hFHO2;
MESHHANDLE hCMPEVA;

extern void CoeffFunc(double aoa, double M, double Re ,double *cl ,double *cm  ,double *cd);
//extern double LiftCoeff (double aoa);

#define LOAD_MESH(var, name) var = oapiLoadMeshGlobal(name);

// "o2 venting" particle streams
PARTICLESTREAMSPEC o2_venting_spec = {
	0,		// flag
	0.3,	// size
	30,		// rate
	2,	    // velocity
	0.5,    // velocity distribution
	2,		// lifetime
	0.2,	// growthrate
	0.5,    // atmslowdown 
	PARTICLESTREAMSPEC::DIFFUSE,
	PARTICLESTREAMSPEC::LVL_FLAT, 1.0, 1.0,
	PARTICLESTREAMSPEC::ATM_FLAT, 1.0, 1.0
};

void SaturnInitMeshes()

{
	LOAD_MESH(hSM, "ProjectApollo/SM-core");
	LOAD_MESH(hSMRCS, "ProjectApollo/SM-RCSHI");
	LOAD_MESH(hSMRCSLow, "ProjectApollo/SM-RCSLO");
	LOAD_MESH(hSMSPS, "ProjectApollo/SM-SPS");
	LOAD_MESH(hSMPanel1, "ProjectApollo/SM-Panel1");
	LOAD_MESH(hSMPanel2, "ProjectApollo/SM-Panel2");
	LOAD_MESH(hSMPanel3, "ProjectApollo/SM-Panel3");
	LOAD_MESH(hSMPanel4, "ProjectApollo/SM-Panel4");
	LOAD_MESH(hSMPanel5, "ProjectApollo/SM-Panel5");
	LOAD_MESH(hSMPanel6, "ProjectApollo/SM-Panel6");
	LOAD_MESH(hSMhga, "ProjectApollo/SM-HGA");
	LOAD_MESH(hSMCRYO, "ProjectApollo/SM-CRYO");
	LOAD_MESH(hSMSIMBAY, "ProjectApollo/SM-SIMBAY");
	LOAD_MESH(hCM, "ProjectApollo/CM");
	LOAD_MESH(hCM2, "ProjectApollo/CM-Recov");
	LOAD_MESH(hCMP, "ProjectApollo/CM-CMP");
	LOAD_MESH(hCMInt, "ProjectApollo/CM-Interior");
	LOAD_MESH(hCMVC, "ProjectApollo/CM-VC");
	LOAD_MESH(hCREW, "ProjectApollo/CM-CREW");
	LOAD_MESH(hFHC, "ProjectApollo/CM-HatchC");
	LOAD_MESH(hFHO, "ProjectApollo/CM-HatchO");
	LOAD_MESH(hCM2B, "ProjectApollo/CMB-Recov");
	LOAD_MESH(hprobe, "ProjectApollo/CM-Probe");
	LOAD_MESH(hprobeext, "ProjectApollo/CM-ProbeExtended");
	LOAD_MESH(hCMBALLOON, "ProjectApollo/CM-Balloons");
	LOAD_MESH(hCRB, "ProjectApollo/CM-CrewRecovery");
	LOAD_MESH(hCMB, "ProjectApollo/CMB");
	LOAD_MESH(hChute30, "ProjectApollo/Apollo_2chute");
	LOAD_MESH(hChute31, "ProjectApollo/Apollo_3chuteEX");
	LOAD_MESH(hChute32, "ProjectApollo/Apollo_3chuteHD");
	LOAD_MESH(hApollochute, "ProjectApollo/Apollo_3chute");
	LOAD_MESH(hFHC2, "ProjectApollo/CMB-HatchC");
	LOAD_MESH(hsat5tower, "ProjectApollo/BoostCover");
	LOAD_MESH(hFHO2, "ProjectApollo/CMB-HatchO");
	LOAD_MESH(hCMPEVA, "ProjectApollo/CM-CMPEVA");
}

void Saturn::AddSM(double offset, bool showSPS)

{
	VECTOR3 mesh_dir=_V(0, SMVO, offset);

	AddMesh (hSM, &mesh_dir);

	if (LowRes)
		AddMesh(hSMRCSLow, &mesh_dir);
	else
		AddMesh (hSMRCS, &mesh_dir);

	AddMesh (hSMPanel1, &mesh_dir);
	AddMesh (hSMPanel2, &mesh_dir);
	AddMesh (hSMPanel3, &mesh_dir);

	if (!ApolloExploded)
		AddMesh (hSMPanel4, &mesh_dir);
	else
		AddMesh (hSMCRYO, &mesh_dir);

	AddMesh (hSMPanel5, &mesh_dir);
	AddMesh (hSMPanel6, &mesh_dir);
	AddMesh (hSMSIMBAY, &mesh_dir);

	if (showSPS) {
		mesh_dir = _V(0, SMVO, offset - 1.654);
		SPSidx = AddMesh(hSMSPS, &mesh_dir);
	}
}

void Saturn::ToggelHatch()

{
	ClearMeshes();

	UINT meshidx;
	VECTOR3 mesh_dir=_V(0,SMVO,30.25-12.25-21.5);
	meshidx = AddMesh (hSM, &mesh_dir);
	SetMeshVisibilityMode (meshidx, MESHVIS_VCEXTERNAL);

	//
	// Skylab SM and Apollo 7 have no HGA.
	//
	if (!NoHGA) {
		mesh_dir=_V(-1.308,-1.18,29.04-12.25-21.5);
		AddMesh (hSMhga, &mesh_dir);
	}

	mesh_dir=_V(0,0,34.4-12.25-21.5);

	if (Crewed) {
		meshidx = AddMesh (hCMP, &mesh_dir);
		SetMeshVisibilityMode (meshidx, MESHVIS_VCEXTERNAL);

		meshidx = AddMesh (hCREW, &mesh_dir);
		SetMeshVisibilityMode (meshidx, MESHVIS_VCEXTERNAL);
	}

	meshidx = AddMesh (hCMInt, &mesh_dir);
	SetMeshVisibilityMode (meshidx, MESHVIS_EXTERNAL);

	meshidx = AddMesh (hCMVC, &mesh_dir);
	SetMeshVisibilityMode (meshidx, MESHVIS_VC);

	//Don't Forget the Hatch
	if (HatchOpen){
		meshidx = AddMesh(hFHC, &mesh_dir);
		HatchOpen = false;
	}
	else{
		meshidx = AddMesh(hFHO, &mesh_dir);
		HatchOpen = true;
	}

	SetMeshVisibilityMode (meshidx, MESHVIS_VCEXTERNAL);

	meshidx = AddMesh (hCM, &mesh_dir);
	SetMeshVisibilityMode (meshidx, MESHVIS_VCEXTERNAL);

	if (dockstate <=4 ){
		if (dockingprobe.ProbeExtended){
			probeidx = AddMesh (hprobeext, &mesh_dir);
		}else {
			probeidx = AddMesh (hprobe, &mesh_dir);
		}
	}
}

void Saturn::ToggelHatch2()

{
	ClearMeshes();
	UINT meshidx ;
	VECTOR3 mesh_dir=_V(0,0,-1.2);
	if (Burned){
		meshidx = AddMesh (hCM2B, &mesh_dir);
	}else{
		meshidx = AddMesh (hCM2, &mesh_dir);
	}
	SetMeshVisibilityMode (meshidx, MESHVIS_VCEXTERNAL);

	meshidx=AddMesh (hCMBALLOON, &mesh_dir);
	SetMeshVisibilityMode (meshidx, MESHVIS_VCEXTERNAL);

	if (Crewed) {
		meshidx = AddMesh (hCMP, &mesh_dir);
		SetMeshVisibilityMode (meshidx, MESHVIS_VCEXTERNAL);

		meshidx = AddMesh (hCREW, &mesh_dir);
		SetMeshVisibilityMode (meshidx, MESHVIS_VCEXTERNAL);
	}

	meshidx = AddMesh (hCMInt, &mesh_dir);
	SetMeshVisibilityMode (meshidx, MESHVIS_EXTERNAL);

	meshidx = AddMesh (hCMVC, &mesh_dir);
	SetMeshVisibilityMode (meshidx, MESHVIS_VC);

	//Don't Forget the Hatch
	
	mesh_dir=_V(0,0,-1.2);
	if (HatchOpen){
		meshidx = AddMesh (hFHC, &mesh_dir);
		HatchOpen= false;
	}
	else{
		meshidx = AddMesh (hFHO, &mesh_dir);
		HatchOpen= true;
	}
	SetMeshVisibilityMode (meshidx, MESHVIS_VCEXTERNAL);
}

void Saturn::ToggleEVA()

{
	UINT meshidx;

	//
	// EVA does nothing if we're unmanned.
	//

	if (!Crewed)
		return;

	ToggleEva = false;

	if (EVA_IP){
		EVA_IP =false;

		ClearMeshes();
		VECTOR3 mesh_dir=_V(0,SMVO,30.25-12.25-21.5);
		AddMesh (hSM, &mesh_dir);

		//
		// Skylab SM and Apollo 7 have no HGA.
		//
		if (!NoHGA) {
			mesh_dir=_V(-1.308,-1.18,29.042-12.25-21.5);
			AddMesh (hSMhga, &mesh_dir);
		}

		mesh_dir=_V(0,0,34.4-12.25-21.5);

		if (Crewed) {
			meshidx = AddMesh (hCMP, &mesh_dir);
			SetMeshVisibilityMode (meshidx, MESHVIS_VCEXTERNAL);

			meshidx = AddMesh (hCREW, &mesh_dir);
			SetMeshVisibilityMode (meshidx, MESHVIS_VCEXTERNAL);
		}

		meshidx = AddMesh (hCMInt, &mesh_dir);
		SetMeshVisibilityMode (meshidx, MESHVIS_EXTERNAL);

		meshidx = AddMesh (hCMVC, &mesh_dir);
		SetMeshVisibilityMode (meshidx, MESHVIS_VC);

		meshidx = AddMesh (hCM, &mesh_dir);
		SetMeshVisibilityMode (meshidx, MESHVIS_VCEXTERNAL);

		//Don't Forget the Hatch
		meshidx = AddMesh (hFHO, &mesh_dir);
		SetMeshVisibilityMode (meshidx, MESHVIS_VCEXTERNAL);

		HatchOpen = true;
		if (dockstate <=4 ){
			if (dockingprobe.ProbeExtended){
				probeidx = AddMesh (hprobeext, &mesh_dir);
			}else {
				probeidx = AddMesh (hprobe, &mesh_dir);
			}
		}
	}
	else {
		EVA_IP = true;

		ClearMeshes();
		VECTOR3 mesh_dir=_V(0,SMVO,30.25-12.25-21.5);
		AddMesh (hSM, &mesh_dir);
		mesh_dir=_V(-1.308,-1.18,29.042-12.25-21.5);
		AddMesh (hSMhga, &mesh_dir);

		mesh_dir=_V(0,0,34.4-12.25-21.5);

		if (Crewed) {
			meshidx = AddMesh (hCREW, &mesh_dir);
			SetMeshVisibilityMode (meshidx, MESHVIS_VCEXTERNAL);
		}

		meshidx = AddMesh (hCMInt, &mesh_dir);
		SetMeshVisibilityMode (meshidx, MESHVIS_EXTERNAL);

		meshidx = AddMesh (hCMVC, &mesh_dir);
		SetMeshVisibilityMode (meshidx, MESHVIS_VC);

		meshidx = AddMesh (hCM, &mesh_dir);
		SetMeshVisibilityMode (meshidx, MESHVIS_VCEXTERNAL);

		//Don't Forget the Hatch
		meshidx = AddMesh (hFHO, &mesh_dir);
		SetMeshVisibilityMode (meshidx, MESHVIS_VCEXTERNAL);

		HatchOpen= true;

		if (dockstate <=4 ){
			if (dockingprobe.ProbeExtended){
				probeidx = AddMesh (hprobeext, &mesh_dir);
			}else {
				probeidx = AddMesh (hprobe, &mesh_dir);
			}
		}

		VESSELSTATUS vs1;
		GetStatus(vs1);
		VECTOR3 ofs1 = _V(0,0.15,34.25-12.25-21.5);
		VECTOR3 vel1 = _V(0,0,0);
		VECTOR3 rofs1, rvel1 = {vs1.rvel.x, vs1.rvel.y, vs1.rvel.z};
		Local2Rel (ofs1, vs1.rpos);
		GlobalRot (vel1, rofs1);
		vs1.rvel.x = rvel1.x+rofs1.x;
		vs1.rvel.y = rvel1.y+rofs1.y;
		vs1.rvel.z = rvel1.z+rofs1.z;
		char VName[256]="";
		strcpy (VName, GetName()); strcat (VName, "-EVA");
		hEVA = oapiCreateVessel(VName,"ProjectApollo/EVA",vs1);
		oapiSetFocusObject(hEVA);
	}
}

void Saturn::SetupEVA()

{
	UINT meshidx;

	if (EVA_IP){
		EVA_IP =true;

		ClearMeshes();
		VECTOR3 mesh_dir=_V(0,SMVO,30.25-12.25-21.5);
		AddMesh (hSM, &mesh_dir);

		//
		// Skylab SM and Apollo 7 have no HGA.
		//
		if (!NoHGA) {
			mesh_dir=_V(-1.308,-1.18,29.042-12.25-21.5);
			AddMesh (hSMhga, &mesh_dir);
		}

		mesh_dir=_V(0,0,34.4-12.25-21.5);

		if (Crewed) {
			meshidx = AddMesh (hCREW, &mesh_dir);
			SetMeshVisibilityMode (meshidx, MESHVIS_VCEXTERNAL);
		}

		meshidx = AddMesh (hCMInt, &mesh_dir);
		SetMeshVisibilityMode (meshidx, MESHVIS_EXTERNAL);

		meshidx = AddMesh (hCMVC, &mesh_dir);
		SetMeshVisibilityMode (meshidx, MESHVIS_VC);

		meshidx = AddMesh (hCM, &mesh_dir);
		SetMeshVisibilityMode (meshidx, MESHVIS_VCEXTERNAL);

		//Don't Forget the Hatch
		meshidx = AddMesh (hFHO, &mesh_dir);
		SetMeshVisibilityMode (meshidx, MESHVIS_VCEXTERNAL);

		HatchOpen = true;

		if (dockstate <=4 ){
			if (dockingprobe.ProbeExtended){
				probeidx = AddMesh (hprobeext, &mesh_dir);
			}else {
				probeidx = AddMesh (hprobe, &mesh_dir);
			}
		}
	}
}


//check this...I think it's a bit wierd
void Saturn::SetRecovery()

{
	ClearMeshes();
	UINT meshidx;

	VECTOR3 mesh_dir=_V(0,0,-1.2);
	if (Burned){
		meshidx = AddMesh (hCM2B, &mesh_dir);
	}
	else{
		meshidx = AddMesh (hCM2, &mesh_dir);
	}
	SetMeshVisibilityMode (meshidx, MESHVIS_VCEXTERNAL);

	meshidx = AddMesh (hFHO, &mesh_dir);
	SetMeshVisibilityMode (meshidx, MESHVIS_VCEXTERNAL);

	HatchOpen = true;
	meshidx=AddMesh (hCMBALLOON, &mesh_dir);
	SetMeshVisibilityMode (meshidx, MESHVIS_VCEXTERNAL);

	meshidx = AddMesh (hCMInt, &mesh_dir);
	SetMeshVisibilityMode (meshidx, MESHVIS_EXTERNAL);

	meshidx = AddMesh (hCMVC, &mesh_dir);
	SetMeshVisibilityMode (meshidx, MESHVIS_VC);

	if (Crewed) {
		mesh_dir =_V(2.7,1.8,-1.5);
		meshidx = AddMesh (hCRB, &mesh_dir);
		SetMeshVisibilityMode (meshidx, MESHVIS_VCEXTERNAL);
	}



	SetTouchdownPoints (_V(0,-1.0,-2.0), _V(-.7,.7,-2.0), _V(.7,.7,-2.0));
	SetView(-1.35);
}

void Saturn::SetCSMStage ()

{
	ClearMeshes();
    ClearThrusterDefinitions();

	if(ph_3rd) {
		DelPropellantResource(ph_3rd);
		ph_3rd = 0;
	}

	if (ph_sep) {
		DelPropellantResource(ph_sep);
		ph_sep = 0;
	}

	SetSize (20);
	SetCOG_elev (3.5);
	SetEmptyMass (CM_Mass + SM_EmptyMass);
	// ************************* propellant specs **********************************

	if (!ph_sps) {
		ph_sps  = CreatePropellantResource(SM_FuelMass, SM_FuelMass); //SPS stage Propellant
	}

	if (ApolloExploded && !ph_o2_vent) {

		double tank_mass = CSM_O2TANK_CAPACITY / 500.0;

		ph_o2_vent = CreatePropellantResource(tank_mass, tank_mass); //SPS stage Propellant

		TankQuantities t;
		GetTankQuantities(t);

		SetPropellantMass(ph_o2_vent, t.O2Tank1QuantityKg + t.O2Tank2QuantityKg);
	}

	SetDefaultPropellantResource (ph_sps); // display SPS stage propellant level in generic HUD

	// *********************** thruster definitions ********************************

	const double CGOffset = 12.25+21.5-1.8+0.35;

	VECTOR3 m_exhaust_pos1= {0,0,-8.-STG2O};
	// orbiter main thrusters
	th_main[0] = CreateThruster (_V( 0,0,-6.5), _V( 0,0,1), 100552.5 , ph_sps, 3778.5);
	DelThrusterGroup(THGROUP_MAIN,true);
	thg_main = CreateThrusterGroup (th_main, 1, THGROUP_MAIN);

	AddExhaust (th_main[0], 20.0, 2.25, SMExhaustTex);
	SetPMI (_V(12,12,7));
	SetCrossSections (_V(40,40,14));
	SetCW (0.1, 0.3, 1.4, 1.4);
	SetRotDrag (_V(0.7,0.7,0.3));
	SetPitchMomentScale (0);
	SetBankMomentScale (0);
	SetLiftCoeffFunc (0);

	if (FIRSTCSM) {
		FIRSTCSM = false;
	}
	else if (bManualUnDock){
		dockstate = 4;
	}

	AddSM(30.25 - CGOffset, true);

	VECTOR3 mesh_dir;

	//
	// Skylab SM and Apollo 7 have no HGA.
	//
	if (!NoHGA) {
		mesh_dir=_V(-1.308,-1.18,29.042-CGOffset);
		AddMesh (hSMhga, &mesh_dir);
	}

	mesh_dir=_V(0,0,34.4-CGOffset);

	UINT meshidx;
	meshidx = AddMesh (hCM, &mesh_dir);
	SetMeshVisibilityMode (meshidx, MESHVIS_VCEXTERNAL);

	//Don't Forget the Hatch
	mesh_dir=_V(0,0,34.4-CGOffset);
	meshidx = AddMesh (hFHC, &mesh_dir);
	SetMeshVisibilityMode (meshidx, MESHVIS_VCEXTERNAL);

	// And the Crew
	if (Crewed) {
		mesh_dir=_V(0,0,34.4-CGOffset);
		meshidx = AddMesh (hCMP, &mesh_dir);
		SetMeshVisibilityMode (meshidx, MESHVIS_VCEXTERNAL);

		mesh_dir=_V(0,0,34.4-CGOffset);
		meshidx = AddMesh (hCREW, &mesh_dir);
		SetMeshVisibilityMode (meshidx, MESHVIS_VCEXTERNAL);
	}

	meshidx = AddMesh (hCMInt, &mesh_dir);
	SetMeshVisibilityMode (meshidx, MESHVIS_EXTERNAL);

	meshidx = AddMesh (hCMVC, &mesh_dir);
	SetMeshVisibilityMode (meshidx, MESHVIS_VC);

	if (dockingprobe.ProbeExtended){
		probeidx = AddMesh (hprobeext, &mesh_dir);
	}else {
		probeidx = AddMesh (hprobe, &mesh_dir);
	}

	VECTOR3 dockpos = {0,0,35.90-CGOffset};
	VECTOR3 dockdir = {0,0,1};
	VECTOR3 dockrot = {0,1,0};
	SetDockParams(dockpos, dockdir, dockrot);

	AddRCSJets(-0.18, SM_RCS_THRUST);

	if (ApolloExploded) {
		VECTOR3 vent_pos = {0, 1.5, 30.25 - CGOffset};
		VECTOR3 vent_dir = {0.5, 1, 0};

		th_o2_vent = CreateThruster (vent_pos, vent_dir, 450.0, ph_o2_vent, 300.0);
		AddExhaustStream(th_o2_vent, &o2_venting_spec);
	}

	SetView(0.4 + 1.8 - 0.35);
	// **************************** NAV radios *************************************

	InitNavRadios (4);
	SetEnableFocus(true);
	EnableTransponder (true);

	OrbiterAttitudeToggle.SetActive(true);

	ThrustAdjust = 1.0;
	ActivateASTP = false;
}

void Saturn::SetCSM2Stage ()
{
	ClearMeshes();
	DelThrusterGroup(THGROUP_MAIN,true);
    ClearThrusterDefinitions();

	if(ph_3rd) {
		DelPropellantResource(ph_3rd);
		ph_3rd = 0;
	}

	if (ph_sep) {
		DelPropellantResource(ph_sep);
		ph_sep = 0;
	}

	SetSize (7);
	SetCOG_elev (3.5);
	SetEmptyMass (CM_Mass + SM_EmptyMass);
	// ************************* propellant specs **********************************

	if (!ph_sps) {
		ph_sps  = CreatePropellantResource(SM_FuelMass, SM_FuelMass); //SPS stage Propellant
	}

	SetDefaultPropellantResource (ph_sps); // display SPS stage propellant level in generic HUD

	// *********************** thruster definitions ********************************

	VECTOR3 m_exhaust_pos1= {0,0,-8.-STG2O};
	// orbiter main thrusters
	th_main[0] = CreateThruster (_V( 0,0,-6.5), _V( 0,0,1),100552.5 , ph_sps, 3778.5);
	thg_main = CreateThrusterGroup (th_main, 1, THGROUP_MAIN);

	AddExhaust (th_main[0], 20.0, 2.25, SMExhaustTex);
	SetEngineLevel(ENGINE_MAIN, 0.0);
	SetPMI (_V(12,12,7));
	SetCrossSections (_V(40,40,14));
	SetCW (0.1, 0.3, 1.4, 1.4);
	SetRotDrag (_V(0.7,0.7,0.3));
	SetPitchMomentScale (0);
	SetBankMomentScale (0);
	SetLiftCoeffFunc (0);

	const double CGOffset = 12.25+21.5-1.8+0.35;

	UINT meshidx;

	AddSM(30.25-CGOffset, true);

	VECTOR3 mesh_dir;

	//
	// Skylab SM and Apollo 7 have no HGA.
	//
	if (!NoHGA) {
		mesh_dir=_V(-1.308,-1.18,29.042-CGOffset);
		AddMesh (hSMhga, &mesh_dir);
	}

	mesh_dir=_V(0,0,34.4-CGOffset);
	meshidx = AddMesh (hCM, &mesh_dir);
	SetMeshVisibilityMode (meshidx, MESHVIS_VCEXTERNAL);

	//Don't Forget the Hatch
	mesh_dir=_V(0,0,34.4-CGOffset);
	meshidx = AddMesh (hFHC, &mesh_dir);
	SetMeshVisibilityMode (meshidx, MESHVIS_VCEXTERNAL);

	// And the Crew
	if (Crewed) {
		mesh_dir=_V(0,0,34.4-CGOffset);
		meshidx = AddMesh (hCMP, &mesh_dir);
		SetMeshVisibilityMode (meshidx, MESHVIS_VCEXTERNAL);

		mesh_dir=_V(0,0,34.4-CGOffset);
		meshidx = AddMesh (hCREW, &mesh_dir);
		SetMeshVisibilityMode (meshidx, MESHVIS_VCEXTERNAL);
	}

	meshidx = AddMesh (hCMInt, &mesh_dir);
	SetMeshVisibilityMode (meshidx, MESHVIS_EXTERNAL);

	meshidx = AddMesh (hCMVC, &mesh_dir);
	SetMeshVisibilityMode (meshidx, MESHVIS_VC);

	SetEngineLevel(ENGINE_MAIN, 0.0);

	AddRCSJets(0.18, SM_RCS_THRUST);

	SetView(0.4 + 1.8 - 0.35);

	EnableTransponder (true);

	// **************************** NAV radios *************************************

	InitNavRadios (4);
	probeidx=0;

	SetEnableFocus(true);

	OrbiterAttitudeToggle.SetActive(true);

	ThrustAdjust = 1.0;
	ActivateASTP = false;
}

void Saturn::SetReentryStage ()

{
	ClearMeshes();
    ClearThrusterDefinitions();
	SetSize (6.0);
	SetCOG_elev (2.0);
	SetEmptyMass (5500);
	SetPMI (_V(12,12,7));
	//SetPMI (_V(1.5,1.35,1.35));
	SetCrossSections (_V(9.17,7.13,7.0));
	SetCW (5.5, 0.1, 3.4, 3.4);
	SetRotDrag (_V(0.07,0.07,0.003));
	if (GetFlightModel() >= 1)
	{
		CreateAirfoil(LIFT_VERTICAL, _V(0.0,0.16,1.12), CoeffFunc, 3.5 ,11.95, 1.0);
    }
  	ShiftCentreOfMass (_V(0,0,0.5));

	UINT meshidx;
	VECTOR3 mesh_dir=_V(0,0,0);
	meshidx = AddMesh (hCM, &mesh_dir);
	SetMeshVisibilityMode (meshidx, MESHVIS_VCEXTERNAL);

	mesh_dir = _V(0,0,0);
	meshidx = AddMesh (hFHC, &mesh_dir);
	SetMeshVisibilityMode (meshidx, MESHVIS_VCEXTERNAL);

	// And the Crew
	if (Crewed) {
		mesh_dir=_V(0,0,0);
		meshidx = AddMesh (hCMP, &mesh_dir);
		SetMeshVisibilityMode (meshidx, MESHVIS_VCEXTERNAL);

		mesh_dir = _V(0,0,0);
		meshidx = AddMesh (hCREW, &mesh_dir);
		SetMeshVisibilityMode (meshidx, MESHVIS_VCEXTERNAL);
	}

	meshidx = AddMesh (hCMInt, &mesh_dir);
	SetMeshVisibilityMode (meshidx, MESHVIS_EXTERNAL);

	meshidx = AddMesh (hCMVC, &mesh_dir);
	SetMeshVisibilityMode (meshidx, MESHVIS_VC);

	SetView(-0.15);

	if (ph_sps) DelPropellantResource(ph_sps);
	if (ph_rcs0) DelPropellantResource(ph_rcs0);
	if (ph_rcs1) DelPropellantResource(ph_rcs1);
	if (ph_rcs2) DelPropellantResource(ph_rcs2);
	if (ph_rcs3) DelPropellantResource(ph_rcs3);

	DelThrusterGroup(THGROUP_MAIN,true);

	if (CMTex) SetReentryTexture(CMTex,1e6,5,0.7);

	AddRCS_CM(CM_RCS_THRUST);

	VECTOR3 dockpos = {0,0,1.5};
	VECTOR3 dockdir = {0,0,1};
	VECTOR3 dockrot = {0,1,0};

	SetDockParams(dockpos, dockdir, dockrot);
}

void Saturn::StageSeven(double simt)

{
	if (CsmLmFinalSep1Switch.GetState() || CsmLmFinalSep2Switch.GetState()) {
		Undock(0);
		dockingprobe.SetEnabled(false);
	}

	if (!Crewed) {
		switch (StageState) {
		case 0:
			if (GetAltitude() < 145000) {
				SlowIfDesired();
				ActivateCMRCS();
				ActivateNavmode(NAVMODE_RETROGRADE);
				StageState++;
			}
			break;
		}
	}

	if (GetAtmPressure() > 300) { // We 're looking wether the CM has burned or not
		ClearMeshes();

		UINT meshidx;
		VECTOR3 mesh_dir=_V(0,0,0);
		meshidx = AddMesh (hCMB, &mesh_dir);
		SetMeshVisibilityMode (meshidx, MESHVIS_VCEXTERNAL);

		mesh_dir=_V(0,0,0);
		meshidx = AddMesh (hFHC, &mesh_dir);
		SetMeshVisibilityMode (meshidx, MESHVIS_VCEXTERNAL);

		// And the Crew
		if (Crewed) {
			mesh_dir=_V(0,0,0);
			meshidx = AddMesh (hCMP, &mesh_dir);
			SetMeshVisibilityMode (meshidx, MESHVIS_VCEXTERNAL);

			mesh_dir=_V(0,0,0);
			meshidx = AddMesh (hCREW, &mesh_dir);
			SetMeshVisibilityMode (meshidx, MESHVIS_VCEXTERNAL);
		}

		meshidx = AddMesh (hCMInt, &mesh_dir);
		SetMeshVisibilityMode (meshidx, MESHVIS_EXTERNAL);

		meshidx = AddMesh (hCMVC, &mesh_dir);
		SetMeshVisibilityMode (meshidx, MESHVIS_VC);

		AddRCS_CM(CM_RCS_THRUST);

		SetStage(CM_ENTRY_STAGE);
		Burned = true;

		SetView(-0.15);
	}
}

void Saturn::StageEight(double simt)

{
	UINT meshidx;
	VECTOR3 mesh_dir=_V(0,0,-1);

	if (Burned) {
		ClearMeshes();
		mesh_dir=_V(0,0,34.40-12.25-21.5);
		meshidx = AddMesh (hCM2B, &mesh_dir);
		SetMeshVisibilityMode (meshidx, MESHVIS_VCEXTERNAL);

		mesh_dir=_V(0,0,34.40-12.25-21.5);
		meshidx = AddMesh (hFHC, &mesh_dir);
		SetMeshVisibilityMode (meshidx, MESHVIS_VCEXTERNAL);

		// And the Crew
		if (Crewed) {
			mesh_dir=_V(0,0,34.4-12.25-21.5);
			meshidx = AddMesh (hCMP, &mesh_dir);
			SetMeshVisibilityMode (meshidx, MESHVIS_VCEXTERNAL);

			mesh_dir=_V(0,0,34.4-12.25-21.5);
			meshidx = AddMesh (hCREW, &mesh_dir);
			SetMeshVisibilityMode (meshidx, MESHVIS_VCEXTERNAL);
		}

		meshidx = AddMesh (hCMInt, &mesh_dir);
		SetMeshVisibilityMode (meshidx, MESHVIS_EXTERNAL);

		meshidx = AddMesh (hCMVC, &mesh_dir);
		SetMeshVisibilityMode (meshidx, MESHVIS_VC);
	}
	else {
		ClearMeshes();
		VECTOR3 mesh_dir=_V(0,0,34.40-12.25-21.5);
		meshidx = AddMesh (hCM2, &mesh_dir);
		SetMeshVisibilityMode (meshidx, MESHVIS_VCEXTERNAL);

	    mesh_dir=_V(0,0,34.40-12.25-21.5);
		meshidx = AddMesh (hFHC, &mesh_dir);
		SetMeshVisibilityMode (meshidx, MESHVIS_VCEXTERNAL);

		// And the Crew
		if (Crewed) {
			mesh_dir=_V(0,0,34.4-12.25-21.5);
			meshidx = AddMesh (hCMP, &mesh_dir);
			SetMeshVisibilityMode (meshidx, MESHVIS_VCEXTERNAL);

			mesh_dir=_V(0,0,34.4-12.25-21.5);
			meshidx = AddMesh (hCREW, &mesh_dir);
			SetMeshVisibilityMode (meshidx, MESHVIS_VCEXTERNAL);
		}

		meshidx = AddMesh (hCMInt, &mesh_dir);
		SetMeshVisibilityMode (meshidx, MESHVIS_EXTERNAL);

		meshidx = AddMesh (hCMVC, &mesh_dir);
		SetMeshVisibilityMode (meshidx, MESHVIS_VC);
	}

	AddRCS_CM(CM_RCS_THRUST);

	SetView(0.5);

	if (!Crewed) {
		switch (StageState) {
		case 0:
			if (GetAltitude() < 50000) {
				SlowIfDesired();
				DeactivateNavmode(NAVMODE_RETROGRADE);
				DeactivateCMRCS();
				StageState++;
			}
			break;
		}
	}

	LAUNCHIND[1] = true;
	SetStage(CM_ENTRY_STAGE_TWO);
}

void Saturn::SetChuteStage1()
{
	UINT meshidx;
	SetSize (15);
	SetCOG_elev (1);
	SetEmptyMass (5500);
	SetMaxFuelMass (100);
	SetFuelMass (0);
	ClearAirfoilDefinitions();
	SetMaxThrust (ENGINE_MAIN,  0);
	SetMaxThrust (ENGINE_RETRO, 0);
	SetMaxThrust (ENGINE_HOVER, 0);
	SetMaxThrust (ENGINE_ATTITUDE, 0);
	SetEngineLevel(ENGINE_ATTITUDE,0);
	SetEngineLevel(ENGINE_MAIN, 0.0);
	SetPMI (_V(20,20,12));
	SetCrossSections (_V(2.8,2.8,80.0));
	SetCW (1.0, 1.5, 1.4, 1.4);
	SetRotDrag (_V(0.7,0.7,1.2));
	if (GetFlightModel() >= 1)
	{
		SetPitchMomentScale (-5e-3);
		SetBankMomentScale (-5e-3);
	}
	SetLiftCoeffFunc (0);
    ClearMeshes();
    ClearExhaustRefs();
    ClearAttExhaustRefs();

	VECTOR3 mesh_dir=_V(0,0,34.40-12.25-16.5-6.5-6.25);
	if (Burned){
		ClearMeshes();
		mesh_dir=_V(0,0,34.40-12.25-21.5-7.75);
		meshidx = AddMesh (hCM2B, &mesh_dir);
		SetMeshVisibilityMode (meshidx, MESHVIS_VCEXTERNAL);

		mesh_dir=_V(0,0,34.40-12.25-21.5-7.75);
		meshidx = AddMesh (hFHC, &mesh_dir);
		SetMeshVisibilityMode (meshidx, MESHVIS_VCEXTERNAL);

		// And the Crew
		if (Crewed) {
			mesh_dir=_V(0,0,34.4-12.25-21.5-7.75);
			meshidx = AddMesh (hCMP, &mesh_dir);
			SetMeshVisibilityMode (meshidx, MESHVIS_VCEXTERNAL);

			mesh_dir=_V(0,0,34.4-12.25-21.5-7.75);
			meshidx = AddMesh (hCREW, &mesh_dir);
			SetMeshVisibilityMode (meshidx, MESHVIS_VCEXTERNAL);
		}

		meshidx = AddMesh (hCMInt, &mesh_dir);
		SetMeshVisibilityMode (meshidx, MESHVIS_EXTERNAL);

		meshidx = AddMesh (hCMVC, &mesh_dir);
		SetMeshVisibilityMode (meshidx, MESHVIS_VC);
	}
	else {
		ClearMeshes();
		mesh_dir=_V(0,0,34.40-12.25-21.5-7.75);
		meshidx = AddMesh (hCM2, &mesh_dir);
		SetMeshVisibilityMode (meshidx, MESHVIS_VCEXTERNAL);

		mesh_dir=_V(0,0,34.40-12.25-21.5-7.75);
		meshidx = AddMesh (hFHC, &mesh_dir);
		SetMeshVisibilityMode (meshidx, MESHVIS_VCEXTERNAL);

		// And the Crew
		if (Crewed) {
			mesh_dir =_V(0,0,34.4-12.25-21.5-7.75);
			meshidx = AddMesh (hCMP, &mesh_dir);
			SetMeshVisibilityMode (meshidx, MESHVIS_VCEXTERNAL);

			mesh_dir=_V(0,0,34.4-12.25-21.5-7.75);
			meshidx = AddMesh (hCREW, &mesh_dir);
			SetMeshVisibilityMode (meshidx, MESHVIS_VCEXTERNAL);
		}
		meshidx = AddMesh (hCMInt, &mesh_dir);
		SetMeshVisibilityMode (meshidx, MESHVIS_EXTERNAL);

		meshidx = AddMesh (hCMVC, &mesh_dir);
		SetMeshVisibilityMode (meshidx, MESHVIS_VC);
	}

	AddRCS_CM(CM_RCS_THRUST);

	mesh_dir=_V(0,0,44.00-12.25-21.5-7.75);
	meshidx = AddMesh (hChute30, &mesh_dir);
	SetMeshVisibilityMode (meshidx, MESHVIS_ALWAYS);

	SetView(-7.25);
	DeactivateNavmode(NAVMODE_KILLROT);
	SetTouchdownPoints (_V(0,-1.0,0), _V(-.7,.7,0), _V(.7,.7,0));
	LAUNCHIND[3] = true;
	LAUNCHIND[1] = true;
}

void Saturn::SetChuteStage2()
{
	UINT meshidx;

	SetCOG_elev (1);
	SetEmptyMass (5500);
	SetMaxFuelMass (100);
	SetFuelMass (0);
	SetMaxThrust (ENGINE_MAIN,  0);
	SetMaxThrust (ENGINE_RETRO, 0);
	SetMaxThrust (ENGINE_HOVER, 0);
	SetMaxThrust (ENGINE_ATTITUDE, 0);
	SetEngineLevel(ENGINE_ATTITUDE,0);
	SetEngineLevel(ENGINE_MAIN, 0.0);
	SetPMI (_V(20,20,12));
	SetCrossSections (_V(2.8,2.8,140.0));
	SetCW (1.0, 1.5, 1.4, 1.4);
	SetRotDrag (_V(0.7,0.7,1.2));
	if (GetFlightModel() >= 1)
	{
		SetPitchMomentScale (-5e-3);
		SetBankMomentScale (-5e-3);
	}
	SetLiftCoeffFunc (0);
    ClearMeshes();
    ClearExhaustRefs();
    ClearAttExhaustRefs();
	//ShiftCentreOfMass (_V(0,0,6.25));

	VECTOR3 mesh_dir=_V(0,0,34.40-12.25-16.5-6.5-6.25);

	if (Burned) {
		ClearMeshes();
		mesh_dir=_V(0,0,34.40-12.25-21.5-7.75);
		meshidx = AddMesh (hCM2B, &mesh_dir);
		SetMeshVisibilityMode (meshidx, MESHVIS_VCEXTERNAL);

		mesh_dir=_V(0,0,34.40-12.25-21.5-7.75);
		meshidx = AddMesh (hFHC, &mesh_dir);
		SetMeshVisibilityMode (meshidx, MESHVIS_VCEXTERNAL);

		// And the Crew
		if (Crewed) {
			mesh_dir=_V(0,0,34.4-12.25-21.5-7.75);
			meshidx = AddMesh (hCMP, &mesh_dir);
			SetMeshVisibilityMode (meshidx, MESHVIS_VCEXTERNAL);

			mesh_dir=_V(0,0,34.4-12.25-21.5-7.75);
			meshidx = AddMesh (hCREW, &mesh_dir);
			SetMeshVisibilityMode (meshidx, MESHVIS_VCEXTERNAL);
		}
		meshidx = AddMesh (hCMInt, &mesh_dir);
		SetMeshVisibilityMode (meshidx, MESHVIS_EXTERNAL);

		meshidx = AddMesh (hCMVC, &mesh_dir);
		SetMeshVisibilityMode (meshidx, MESHVIS_VC);
	}
	else {
		ClearMeshes();
		mesh_dir=_V(0,0,34.40-12.25-21.5-7.75);
		meshidx = AddMesh (hCM2, &mesh_dir);
		SetMeshVisibilityMode (meshidx, MESHVIS_VCEXTERNAL);

		mesh_dir=_V(0,0,34.40-12.25-21.5-7.75);
		meshidx = AddMesh (hFHC, &mesh_dir);
		SetMeshVisibilityMode (meshidx, MESHVIS_VCEXTERNAL);

		// And the Crew
		if (Crewed) {
			mesh_dir=_V(0,0,34.4-12.25-21.5-7.75);
			meshidx = AddMesh (hCMP, &mesh_dir);
			SetMeshVisibilityMode (meshidx, MESHVIS_VCEXTERNAL);

			mesh_dir=_V(0,0,34.4-12.25-21.5-7.75);
			meshidx = AddMesh (hCREW, &mesh_dir);
			SetMeshVisibilityMode (meshidx, MESHVIS_VCEXTERNAL);
		}
		meshidx = AddMesh (hCMInt, &mesh_dir);
		SetMeshVisibilityMode (meshidx, MESHVIS_EXTERNAL);

		meshidx = AddMesh (hCMVC, &mesh_dir);
		SetMeshVisibilityMode (meshidx, MESHVIS_VC);
	}

	AddRCS_CM(CM_RCS_THRUST);

	mesh_dir=_V(0,-0.25,39.7-12.25-21.5-7.75);
	meshidx=AddMesh (hChute31, &mesh_dir);
	SetMeshVisibilityMode (meshidx, MESHVIS_ALWAYS);

	SetView(-7.25);
	SetTouchdownPoints (_V(0,-1.0,0), _V(-.7,.7,0), _V(.7,.7,0));
	LAUNCHIND[3] = true;
	LAUNCHIND[1] = true;
}

void Saturn::SetChuteStage3()
{
	UINT meshidx;
	SetSize (12);
	SetCOG_elev (1);
	SetEmptyMass (5500);
	SetMaxFuelMass (100);
	SetFuelMass (0);
	SetMaxThrust (ENGINE_MAIN,  0);
	SetMaxThrust (ENGINE_RETRO, 0);
	SetMaxThrust (ENGINE_HOVER, 0);
	SetMaxThrust (ENGINE_ATTITUDE, 0);
	SetEngineLevel(ENGINE_ATTITUDE,0);
	SetEngineLevel(ENGINE_MAIN, 0.0);
	SetPMI (_V(20,20,12));
	SetCrossSections (_V(2.8,2.8,480.0));
	SetCW (0.7, 1.5, 1.4, 1.4);
	SetRotDrag (_V(0.7,0.7,1.2));
	if (GetFlightModel() >= 1)
	{
		SetPitchMomentScale (-5e-3);
		SetBankMomentScale (-5e-3);
	}
	SetLiftCoeffFunc (0);
    ClearMeshes();
    ClearExhaustRefs();
    ClearAttExhaustRefs();
	//ShiftCentreOfMass (_V(0,0,6.25));
	VECTOR3 mesh_dir=_V(0,0,34.40-12.25-16.5-6.5-6.25);
	if (Burned){
		ClearMeshes();
		mesh_dir=_V(0,0,34.40-12.25-21.5-7.75);
		meshidx = AddMesh (hCM2B, &mesh_dir);
		SetMeshVisibilityMode (meshidx, MESHVIS_VCEXTERNAL);

		mesh_dir=_V(0,0,34.40-12.25-21.5-7.75);
		meshidx = AddMesh (hFHC, &mesh_dir);
		SetMeshVisibilityMode (meshidx, MESHVIS_VCEXTERNAL);

		// And the Crew
		if (Crewed) {
			mesh_dir=_V(0,0,34.4-12.25-21.5-7.75);
			meshidx = AddMesh (hCMP, &mesh_dir);
			SetMeshVisibilityMode (meshidx, MESHVIS_VCEXTERNAL);

			mesh_dir=_V(0,0,34.4-12.25-21.5-7.75);
			meshidx = AddMesh (hCREW, &mesh_dir);
			SetMeshVisibilityMode (meshidx, MESHVIS_VCEXTERNAL);
		}
		meshidx = AddMesh (hCMInt, &mesh_dir);
		SetMeshVisibilityMode (meshidx, MESHVIS_EXTERNAL);

		meshidx = AddMesh (hCMVC, &mesh_dir);
		SetMeshVisibilityMode (meshidx, MESHVIS_VC);
	}
	else {
		ClearMeshes();
		mesh_dir=_V(0,0,34.40-12.25-21.5-7.75);
		meshidx = AddMesh (hCM2, &mesh_dir);
		SetMeshVisibilityMode (meshidx, MESHVIS_VCEXTERNAL);

		mesh_dir=_V(0,0,34.40-12.25-21.5-7.75);
		meshidx = AddMesh (hFHC, &mesh_dir);
		SetMeshVisibilityMode (meshidx, MESHVIS_VCEXTERNAL);

		// And the Crew
		if (Crewed) {
			mesh_dir=_V(0,0,34.4-12.25-21.5-7.75);
			meshidx = AddMesh (hCMP, &mesh_dir);
			SetMeshVisibilityMode (meshidx, MESHVIS_VCEXTERNAL);

			mesh_dir=_V(0,0,34.4-12.25-21.5-7.75);
			meshidx = AddMesh (hCREW, &mesh_dir);
			SetMeshVisibilityMode (meshidx, MESHVIS_VCEXTERNAL);
		}
		meshidx = AddMesh (hCMInt, &mesh_dir);
		SetMeshVisibilityMode (meshidx, MESHVIS_EXTERNAL);

		meshidx = AddMesh (hCMVC, &mesh_dir);
		SetMeshVisibilityMode (meshidx, MESHVIS_VC);
	}

	mesh_dir=_V(0,-1.3,17);
	meshidx = AddMesh (hChute32, &mesh_dir);
	SetMeshVisibilityMode (meshidx, MESHVIS_ALWAYS);

	AddRCS_CM(CM_RCS_THRUST);

	SetView(-7.25);
	SetTouchdownPoints (_V(0,-1.0,0), _V(-.7,.7,0), _V(.7,.7,0));
	LAUNCHIND[3] = true;
	LAUNCHIND[1] = true;
}

void Saturn::SetChuteStage4()
{
	UINT meshidx;
	SetSize (12);
	SetCOG_elev (1);
	SetEmptyMass (5500);
	SetMaxFuelMass (500);
	SetFuelMass (0);
	SetTouchdownPoints (_V(0,0.0,0), _V(-1,0,0), _V(1,0,0));
	SetPMI (_V(20,20,12));
	SetCrossSections (_V(2.8,2.8,3280.0));
	SetCW (0.7, 1.5, 1.4, 1.4);
	SetRotDrag (_V(0.7,0.7,1.2));
	if (GetFlightModel() >= 1)
	{
		SetPitchMomentScale (-5e-3);
		SetBankMomentScale (-5e-3);
	}
	SetLiftCoeffFunc (0);
    ClearMeshes();
    ClearExhaustRefs();
    ClearAttExhaustRefs();
	//ShiftCentreOfMass (_V(0,0,6.25));
	VECTOR3 mesh_dir=_V(0,0,34.40-12.25-16.5-6.5-6.25);
	if (Burned){
		ClearMeshes();
		mesh_dir=_V(0,0,34.40-12.25-21.5-7.75);
		meshidx = AddMesh (hCM2B, &mesh_dir);
		SetMeshVisibilityMode (meshidx, MESHVIS_VCEXTERNAL);

		mesh_dir=_V(0,0,34.40-12.25-21.5-7.75);
		meshidx = AddMesh (hFHC, &mesh_dir);
		SetMeshVisibilityMode (meshidx, MESHVIS_VCEXTERNAL);

		// And the Crew
		if (Crewed) {
			mesh_dir=_V(0,0,34.4-12.25-21.5-7.75);
			meshidx = AddMesh (hCMP, &mesh_dir);
			SetMeshVisibilityMode (meshidx, MESHVIS_VCEXTERNAL);

			mesh_dir=_V(0,0,34.4-12.25-21.5-7.75);
			meshidx = AddMesh (hCREW, &mesh_dir);
			SetMeshVisibilityMode (meshidx, MESHVIS_VCEXTERNAL);
		}
		meshidx = AddMesh (hCMInt, &mesh_dir);
		SetMeshVisibilityMode (meshidx, MESHVIS_EXTERNAL);

		meshidx = AddMesh (hCMVC, &mesh_dir);
		SetMeshVisibilityMode (meshidx, MESHVIS_VC);
	}
	else{
		ClearMeshes();
		mesh_dir=_V(0,0,34.40-12.25-21.5-7.75);
		meshidx = AddMesh (hCM2, &mesh_dir);
		SetMeshVisibilityMode (meshidx, MESHVIS_VCEXTERNAL);

		mesh_dir=_V(0,0,34.40-12.25-21.5-7.75);
		meshidx = AddMesh (hFHC, &mesh_dir);
		SetMeshVisibilityMode (meshidx, MESHVIS_VCEXTERNAL);

		// And the Crew
		if (Crewed) {
			mesh_dir=_V(0,0,34.4-12.25-21.5-7.75);
			meshidx = AddMesh (hCMP, &mesh_dir);
			SetMeshVisibilityMode (meshidx, MESHVIS_VCEXTERNAL);

			mesh_dir=_V(0,0,34.4-12.25-21.5-7.75);
			meshidx = AddMesh (hCREW, &mesh_dir);
			SetMeshVisibilityMode (meshidx, MESHVIS_VCEXTERNAL);
		}
		meshidx = AddMesh (hCMInt, &mesh_dir);
		SetMeshVisibilityMode (meshidx, MESHVIS_EXTERNAL);

		meshidx = AddMesh (hCMVC, &mesh_dir);
		SetMeshVisibilityMode (meshidx, MESHVIS_VC);
	}
	mesh_dir = OFS_MAINCHUTE;
	meshidx=AddMesh (hApollochute, &mesh_dir);
	SetMeshVisibilityMode (meshidx, MESHVIS_ALWAYS);

	AddRCS_CM(CM_RCS_THRUST);

	SetView(-7.25);
	SetTouchdownPoints (_V(0,-1.0,0), _V(-.7,.7,0), _V(.7,.7,0));
	LAUNCHIND[5]=true;
	LAUNCHIND[3]=true;
	LAUNCHIND[1]=true;
}

void Saturn::SetSplashStage()
{
	UINT meshidx;
	SetSize (4);
	SetCOG_elev (2);
	SetEmptyMass (5500);
	SetMaxFuelMass (0);
	SetFuelMass (0);
	SetMaxThrust (ENGINE_MAIN,  0);
	SetMaxThrust (ENGINE_ATTITUDE, 0);
	SetEngineLevel(ENGINE_ATTITUDE,0);
	SetEngineLevel(ENGINE_MAIN, 0.0);
	SetPMI (_V(20,20,12));
	SetCrossSections (_V(2.8,2.8,7.0));
	SetCW (0.5, 1.5, 1.4, 1.4);
	SetRotDrag (_V(0.7,0.7,1.2));
	SetTouchdownPoints (_V(0,-1.0,-2.0), _V(-.7,.7,-2.0), _V(.7,.7,-2.0));
	if (GetFlightModel() >= 1)
	{
		SetPitchMomentScale (-5e-3);
		SetBankMomentScale (-5e-3);
	}
	SetLiftCoeffFunc (0);
    ClearMeshes();
    ClearExhaustRefs();
    ClearAttExhaustRefs();
	//ShiftCentreOfMass (_V(0,0,6.25));

	VECTOR3 mesh_dir=_V(0,0,-1.2);
	if (Burned){
		meshidx = AddMesh (hCM2B, &mesh_dir);
		SetMeshVisibilityMode (meshidx, MESHVIS_VCEXTERNAL);

		meshidx = AddMesh (hFHC2, &mesh_dir);
		SetMeshVisibilityMode (meshidx, MESHVIS_VCEXTERNAL);
	}
	else {
		meshidx = AddMesh (hCM2, &mesh_dir);
		SetMeshVisibilityMode (meshidx, MESHVIS_VCEXTERNAL);

		meshidx = AddMesh (hFHC, &mesh_dir);
		SetMeshVisibilityMode (meshidx, MESHVIS_VCEXTERNAL);
	}

	meshidx=AddMesh (hCMBALLOON, &mesh_dir);
	SetMeshVisibilityMode (meshidx, MESHVIS_ALWAYS);

	if (Crewed) {
		meshidx = AddMesh (hCMP, &mesh_dir);
		SetMeshVisibilityMode (meshidx, MESHVIS_VCEXTERNAL);

		meshidx = AddMesh (hCREW, &mesh_dir);
		SetMeshVisibilityMode (meshidx, MESHVIS_VCEXTERNAL);
	}
	meshidx = AddMesh (hCMInt, &mesh_dir);
	SetMeshVisibilityMode (meshidx, MESHVIS_EXTERNAL);

	meshidx = AddMesh (hCMVC, &mesh_dir);
	SetMeshVisibilityMode (meshidx, MESHVIS_VC);

	AddRCS_CM(CM_RCS_THRUST);

	SetView(-1.35);
}

void Saturn::SetAbortStage ()
{

	ClearMeshes();
    ClearThrusterDefinitions();
	UINT meshidx;
	SetSize (8);
	SetCOG_elev (2.0);
	SetEmptyMass (6718);
	SetPMI (_V(12,12,7));
	//SetPMI (_V(1.5,1.35,1.35));
	SetCrossSections (_V(9.17,7.13,7.0));
	SetCW (5.5, 0.1, 3.4, 3.4);
	SetRotDrag (_V(0.07,0.07,0.003));
	if (GetFlightModel() >= 1)
	{
//		SetPitchMomentScale (-1e-5);
//		SetBankMomentScale (-1e-5);
//		SetLiftCoeffFunc (LiftCoeff); 
//		CreateAirfoil(LIFT_VERTICAL, _V(-0.014,0.107,0.75), CoeffFunc, 3.5 ,11.95, 1.0);
		CreateAirfoil(LIFT_VERTICAL, _V(0.0,0.16,1.12), CoeffFunc, 3.5 ,11.95, 1.0);
    }
	ShiftCentreOfMass (_V(0,0,1.5));
	VECTOR3 mesh_dir=_V(0,0,33.0-12.25-21.5-1.5+1);
	meshidx = AddMesh (hCM, &mesh_dir);
	SetMeshVisibilityMode (meshidx, MESHVIS_VCEXTERNAL);

	TowerOffset = 38.0-12.25-21.5-1.5+1;

	mesh_dir=_V(0, 0, TowerOffset);
	meshidx = AddMesh (hsat5tower, &mesh_dir);
	SetMeshVisibilityMode (meshidx, MESHVIS_VCEXTERNAL);

	//Don't Forget the Hatch
	mesh_dir=_V(0,0,33.0-12.25-21.5-1.5+1);
	meshidx = AddMesh (hFHC, &mesh_dir);
	SetMeshVisibilityMode (meshidx, MESHVIS_VCEXTERNAL);

	// And the Crew
	if (Crewed) {
		mesh_dir=_V(0,0,33.0-12.25-21.5-1.5+1);
		meshidx = AddMesh (hCMP, &mesh_dir);
		SetMeshVisibilityMode (meshidx, MESHVIS_VCEXTERNAL);

		mesh_dir=_V(0,0,33.0-12.25-21.5-1.5+1);
		meshidx = AddMesh (hCREW, &mesh_dir);
		SetMeshVisibilityMode (meshidx, MESHVIS_VCEXTERNAL);
	}
	meshidx = AddMesh (hCMInt, &mesh_dir);
	SetMeshVisibilityMode (meshidx, MESHVIS_EXTERNAL);

	meshidx = AddMesh (hCMVC, &mesh_dir);
	SetMeshVisibilityMode (meshidx, MESHVIS_VC);

	//VECTOR3 bdir = _V(0,0,1);
	//ReEntryID = AddExhaustRef(EXHAUST_CUSTOM,_V(0,0,-0.5), 0, 0, &bdir);
	if (ph_3rd)  {
		DelPropellantResource(ph_3rd); //SPS stage Propellant
		ph_3rd = 0;
	}
	if(ph_1st) {
		DelPropellantResource(ph_1st);
		ph_1st = 0;
	}
	if(ph_2nd) {
		DelPropellantResource(ph_2nd);
		ph_2nd = 0;
	}

	if (ph_sps)
		DelPropellantResource(ph_sps); //SPS stage Propellant
		ph_sps = 0;
	if (!ph_sps)
		ph_sps  = CreatePropellantResource(2500); //SPS stage Propellant
	SetDefaultPropellantResource (ph_sps); // display SPS stage propellant level in generic HUD

	AddRCS_CM(CM_RCS_THRUST);

	// *********************** thruster definitions ********************************

	// orbiter main thrusters
	th_main[0] = CreateThruster (_V( 0,0,-6.5), _V( 0,0,1),721035 , ph_sps, 900);
	thg_main = CreateThrusterGroup (th_main, 1, THGROUP_MAIN);

	SetThrusterLevel(th_main[0], 1.0);

	VECTOR3 m_exhaust_pos1= {0.4,0.0,1.6};
    VECTOR3 m_exhaust_pos2= {-0.4,0.0,1.6};
	VECTOR3 m_exhaust_pos3= {0.0,0.1,6.8};
	VECTOR3 m_exhaust_pos4= {0.0,0.4,1.6};
	VECTOR3 m_exhaust_pos5= {0.0,-0.4,1.6};
	VECTOR3 m_exhaust_ref1 = {0.65,0,-1};
	VECTOR3 m_exhaust_ref2 = {-0.65,0,-1};
	VECTOR3 m_exhaust_ref3 = {0,0.5,-1};
	VECTOR3 m_exhaust_ref4 = {0.0,0.65,-1};
	VECTOR3 m_exhaust_ref5 = {0.0,-0.65,-1};

	AddExhaustRef (EXHAUST_MAIN, m_exhaust_pos1, 5.0, 0.15, &m_exhaust_ref1);
	AddExhaustRef (EXHAUST_MAIN, m_exhaust_pos2, 5.0, 0.15, &m_exhaust_ref2);
	AddExhaustRef (EXHAUST_MAIN, m_exhaust_pos3, 3.0, 0.10, &m_exhaust_ref3);
	AddExhaustRef (EXHAUST_MAIN, m_exhaust_pos4, 5.0, 0.15, &m_exhaust_ref4);
	AddExhaustRef (EXHAUST_MAIN, m_exhaust_pos5, 5.0, 0.15, &m_exhaust_ref5);

	//AddExhaustRef (EXHAUST_MAIN, m_exhaust_pos4, 5.0, 0.25, &m_exhaust_ref4);

	SetView(0.0);

	ABORT_IND = true;

	OrbiterAttitudeToggle.SetState(false);
}

bool Saturn::clbkLoadGenericCockpit ()

{
	TRACESETUP("Saturn::clbkLoadGenericCockpit");

	//
	// VC-only in engineering camera view.
	//

	if (viewpos == SATVIEW_ENG1 || viewpos == SATVIEW_ENG2)
		return false;

	SetCameraRotationRange(0.0, 0.0, 0.0, 0.0);
	SetCameraDefaultDirection(_V(0.0, 0.0, 1.0));
	InVC = false;
	InPanel = false;
	SetView();
	return true;
}

//
// Generic function to jettison the escape tower.
//

void Saturn::JettisonLET()

{
	VECTOR3 ofs1 = _V(0.0, 0.0, TowerOffset); // OFS_TOWER;
	VECTOR3 vel1 = _V(15.0,15.0,106.0);

	VESSELSTATUS vs1;
	GetStatus (vs1);

	vs1.eng_main = vs1.eng_hovr = 0.0;

	VECTOR3 rofs1, rvel1 = {vs1.rvel.x, vs1.rvel.y, vs1.rvel.z};

	Local2Rel (ofs1, vs1.rpos);

	GlobalRot (vel1, rofs1);

	vs1.rvel.x = rvel1.x+rofs1.x;
	vs1.rvel.y = rvel1.y+rofs1.y;
	vs1.rvel.z = rvel1.z+rofs1.z;

	vs1.vrot.x = 0.0;
	vs1.vrot.y = 0.0;
	vs1.vrot.z = 0.0;

	TowerJS.play();
	TowerJS.done();

	char VName[256];

	GetApolloName(VName);
	strcat (VName, "-TWR");

	hesc1 = oapiCreateVessel(VName,"ProjectApollo/sat5btower",vs1);
	LESAttached = false;

	ConfigureStageMeshes(stage);
}
