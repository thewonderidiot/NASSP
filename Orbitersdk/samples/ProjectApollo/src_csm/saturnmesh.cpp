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

  **************************************************************************/

// To force orbitersdk.h to use <fstream> in any compiler version
#pragma include_alias( <fstream.h>, <fstream> )
#include "Orbitersdk.h"
#include <stdio.h>
#include <math.h>
#include "soundlib.h"

#include "resource.h"

#include "nasspdefs.h"
#include "nasspsound.h"

#include "toggleswitch.h"
#include "apolloguidance.h"
#include "csmcomputer.h"
#include "ioChannels.h"

#include "saturn.h"
#include "tracer.h"
#include "sivb.h"

#include "LES.h"

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
MESHHANDLE hCMnh;
MESHHANDLE hCM2;
MESHHANDLE hCMP;
MESHHANDLE hCMInt;
MESHHANDLE hCMVC;
MESHHANDLE hCREW;
MESHHANDLE hFHO;
MESHHANDLE hFHC;
MESHHANDLE hFHF;
MESHHANDLE hCM2B;
MESHHANDLE hprobe;
MESHHANDLE hprobeext;
//MESHHANDLE hCMBALLOON;
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
MESHHANDLE hopticscover;
MESHHANDLE hcmdocktgt;

#define LOAD_MESH(var, name) var = oapiLoadMeshGlobal(name);

// "O2 venting" particle streams
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

static PARTICLESTREAMSPEC lem_exhaust = {
	0,		// flag
	0.5,	// size
	100.0, 	// rate
	25.0,	// velocity
	0.1,	// velocity distribution
	0.3, 	// lifetime
	3.0,	// growthrate
	0.0,	// atmslowdown
	PARTICLESTREAMSPEC::EMISSIVE,
	PARTICLESTREAMSPEC::LVL_FLAT, 1.0, 1.0,
	PARTICLESTREAMSPEC::ATM_FLAT, 1.0, 1.0
};

PARTICLESTREAMSPEC dyemarker_spec = {
	0,		// flag
	0.15,	// size
	15,	    // rate
	1,	    // velocity
	0.3,    // velocity distribution
	3,		// lifetime
	0.2,	// growthrate
	0.2,    // atmslowdown 
	PARTICLESTREAMSPEC::EMISSIVE,
	PARTICLESTREAMSPEC::LVL_FLAT, 1.0, 1.0,
	PARTICLESTREAMSPEC::ATM_FLAT, 1.0, 1.0
};

PARTICLESTREAMSPEC wastewaterdump_spec = {
	0,		// flag
	0.005,	// size
	1000,	// rate
	1.5,    // velocity
	0.2,    // velocity distribution
	100,	// lifetime
	0.001,	// growthrate
	0,      // atmslowdown 
	PARTICLESTREAMSPEC::DIFFUSE,
	PARTICLESTREAMSPEC::LVL_FLAT, 1.0, 1.0,
	PARTICLESTREAMSPEC::ATM_FLAT, 1.0, 1.0
};

PARTICLESTREAMSPEC urinedump_spec = {
	0,		// flag
	0.005,	// size
	1000,	// rate
	1.5,    // velocity
	0.2,    // velocity distribution
	100,	// lifetime
	0.001,	// growthrate
	0,		// atmslowdown 
	PARTICLESTREAMSPEC::DIFFUSE,
	PARTICLESTREAMSPEC::LVL_FLAT, 1.0, 1.0,
	PARTICLESTREAMSPEC::ATM_FLAT, 1.0, 1.0
};

void CMCoeffFunc(double aoa, double M, double Re, double *cl, double *cm, double *cd)

{
	const int nlift = 11;
	double factor, dfact, lfact, frac, drag, lift;
	static const double AOA[nlift] =
	{ -180.*RAD,-160.*RAD,-150.*RAD,-120.*RAD,-90.*RAD,0 * RAD,90.*RAD,120.*RAD,150.*RAD,160.*RAD,180.*RAD };
	static const double Mach[17] = { 0.0,0.7,0.9,1.1,1.2,1.35,1.65,2.0,3.0,5.0,8.0,10.5,13.5,18.2,21.5,31.0,50.0 };
	static const double LFactor[17] = { 0.3,0.392,0.466,0.607,0.641,0.488,0.446,0.435,0.416,0.415,0.405,0.400,0.385,0.385,0.375,0.35,0.33 };
	static const double DFactor[17] = { 0.9,0.944,0.991,1.068,1.044,1.270,1.28,1.267,1.213,1.134,1.15,1.158,1.18,1.18,1.193,1.224,1.25 };
	static const double CL[nlift] = { 0.0,-0.9,-1.1,-0.5,0.0,0.0,0.0,0.5,1.1,0.9,0.0 };
	static const double CM[nlift] = { 0.0,0.004,0.006,0.012,0.015,0.0,-0.015,-0.012,-0.006,-0.004,0. };
	static const double CD[nlift] = { 1.143,1.0,1.0,0.8,0.8,0.8,0.8,0.8,1.0,1.0,1.143 };
	int j;
	factor = -5.0;
	dfact = 1.05;
	lfact = 0.94;
	for (j = 0; (j < 16) && (Mach[j + 1] < M); j++);
	frac = (M - Mach[j]) / (Mach[j + 1] - Mach[j]);
	drag = dfact * (frac*DFactor[j + 1] + (1.0 - frac)*DFactor[j]);
	lift = drag * lfact*(frac*LFactor[j + 1] + (1.0 - frac)*LFactor[j]);
	for (j = 0; (j < nlift - 1) && (AOA[j + 1] < aoa); j++);
	frac = (aoa - AOA[j]) / (AOA[j + 1] - AOA[j]);
	*cd = drag * (frac*CD[j + 1] + (1.0 - frac)*CD[j]);
	*cl = lift * (frac*CL[j + 1] + (1.0 - frac)*CL[j]);
	*cm = factor * (frac*CM[j + 1] + (1.0 - frac)*CM[j]);
}

void CMLETVertCoeffFunc(double aoa, double M, double Re, double *cl, double *cm, double *cd)

{
	const int nlift = 19;
	double factor, frac, drag, lift;
	static const double AOA[nlift] =
	{ -180.*RAD,-160.*RAD,-150.*RAD,-120.*RAD,-90.*RAD,-40.*RAD,-30.*RAD,-20.*RAD,-10.*RAD,0 * RAD,10.*RAD,20.*RAD,30.*RAD,40.*RAD,90.*RAD,120.*RAD,150.*RAD,160.*RAD,180.*RAD };
	static const double Mach[17] = { 0.0,0.7,0.9,1.1,1.2,1.35,1.65,2.0,3.0,5.0,8.0,10.5,13.5,18.2,21.5,31.0,50.0 };
	static const double LFactor[17] = { 0.3,0.392,0.466,0.607,0.641,0.488,0.446,0.435,0.416,0.415,0.405,0.400,0.385,0.385,0.375,0.35,0.33 };
	static const double DFactor[17] = { 0.9,0.944,0.991,1.068,1.044,1.270,1.28,1.267,1.213,1.134,1.15,1.158,1.18,1.18,1.193,1.224,1.25 };
	static const double CL[nlift] = { 0.0,-0.9,-1.1,-0.5,0.0,-0.316196,-0.239658,-0.193466,-0.110798,0.0,0.110798,0.193466,0.239658,0.316196,0.0,0.5,1.1,0.9,0.0 };
	static const double CM[nlift] = { 0.0,-0.02,-0.03,-0.06,-0.075,0.08,0.1,0.11,0.09,0.0,-0.09,-0.11,-0.1,-0.08,0.075,0.06,0.03,0.02,0. };
	static const double CD[nlift] = { 1.143,1.0,1.0,0.8,0.8,0.72946,0.65157,0.63798,0.65136,0.5778,0.65136,0.63798,0.65157,0.72946,0.8,0.8,1.0,1.0,1.143 };
	int j;
	factor = 2.0;
	for (j = 0; (j < 16) && (Mach[j + 1] < M); j++);
	frac = (M - Mach[j]) / (Mach[j + 1] - Mach[j]);
	drag = (frac*DFactor[j + 1] + (1.0 - frac)*DFactor[j]);
	lift = drag * (frac*LFactor[j + 1] + (1.0 - frac)*LFactor[j]);
	for (j = 0; (j < nlift - 1) && (AOA[j + 1] < aoa); j++);
	frac = (aoa - AOA[j]) / (AOA[j + 1] - AOA[j]);
	*cd = drag * (frac*CD[j + 1] + (1.0 - frac)*CD[j]);
	*cl = lift * (frac*CL[j + 1] + (1.0 - frac)*CL[j]);
	*cm = factor * (frac*CM[j + 1] + (1.0 - frac)*CM[j]);
}

void CMLETCanardVertCoeffFunc(double aoa, double M, double Re, double *cl, double *cm, double *cd)

{
	const int nlift = 19;
	double factor, frac, drag, lift;
	static const double AOA[nlift] =
	{ -180.*RAD,-160.*RAD,-150.*RAD,-120.*RAD,-90.*RAD,-40.*RAD,-30.*RAD,-20.*RAD,-10.*RAD,0 * RAD,10.*RAD,20.*RAD,30.*RAD,40.*RAD,90.*RAD,120.*RAD,150.*RAD,160.*RAD,180.*RAD };
	static const double Mach[17] = { 0.0,0.7,0.9,1.1,1.2,1.35,1.65,2.0,3.0,5.0,8.0,10.5,13.5,18.2,21.5,31.0,50.0 };
	static const double LFactor[17] = { 0.3,0.392,0.466,0.607,0.641,0.488,0.446,0.435,0.416,0.415,0.405,0.400,0.385,0.385,0.375,0.35,0.33 };
	static const double DFactor[17] = { 0.9,0.944,0.991,1.068,1.044,1.270,1.28,1.267,1.213,1.134,1.15,1.158,1.18,1.18,1.193,1.224,1.25 };
	static const double CL[nlift] = { 0.0,-0.9,-1.1,-0.5,0.0,-0.316196,-0.239658,-0.193466,-0.110798,0.0,0.110798,0.193466,0.239658,0.316196,0.0,0.5,1.1,0.9,0.0 };
	static const double CM[nlift] = { -0.05,-0.375,-0.425,-0.35,-0.25,-0.1,0.0,0.1,0.2,0.3,0.2,0.15,0.15,0.2,0.375,0.325,0.325,0.05,-0.05 };
	static const double CD[nlift] = { 1.143,1.0,1.0,0.8,0.8,0.72946,0.65157,0.63798,0.65136,0.5778,0.65136,0.63798,0.65157,0.72946,0.8,0.8,1.0,1.0,1.143 };
	int j;
	factor = 2.0;
	for (j = 0; (j < 16) && (Mach[j + 1] < M); j++);
	frac = (M - Mach[j]) / (Mach[j + 1] - Mach[j]);
	drag = (frac*DFactor[j + 1] + (1.0 - frac)*DFactor[j]);
	lift = drag * (frac*LFactor[j + 1] + (1.0 - frac)*LFactor[j]);
	for (j = 0; (j < nlift - 1) && (AOA[j + 1] < aoa); j++);
	frac = (aoa - AOA[j]) / (AOA[j + 1] - AOA[j]);
	*cd = drag * (frac*CD[j + 1] + (1.0 - frac)*CD[j]);
	*cl = lift * (frac*CL[j + 1] + (1.0 - frac)*CL[j]);
	*cm = factor * (frac*CM[j + 1] + (1.0 - frac)*CM[j]);
}

void CMLETHoriCoeffFunc(double aoa, double M, double Re, double *cl, double *cm, double *cd)

{
	const int nlift = 19;
	double factor, frac, drag, lift;
	static const double AOA[nlift] =
	{ -180.*RAD,-160.*RAD,-150.*RAD,-120.*RAD,-90.*RAD,-40.*RAD,-30.*RAD,-20.*RAD,-10.*RAD,0 * RAD,10.*RAD,20.*RAD,30.*RAD,40.*RAD,90.*RAD,120.*RAD,150.*RAD,160.*RAD,180.*RAD };
	static const double Mach[17] = { 0.0,0.7,0.9,1.1,1.2,1.35,1.65,2.0,3.0,5.0,8.0,10.5,13.5,18.2,21.5,31.0,50.0 };
	static const double LFactor[17] = { 0.3,0.392,0.466,0.607,0.641,0.488,0.446,0.435,0.416,0.415,0.405,0.400,0.385,0.385,0.375,0.35,0.33 };
	static const double DFactor[17] = { 0.9,0.944,0.991,1.068,1.044,1.270,1.28,1.267,1.213,1.134,1.15,1.158,1.18,1.18,1.193,1.224,1.25 };
	static const double CL[nlift] = { 0.0,-0.9,-1.1,-0.5,0.0,-0.316196,-0.239658,-0.193466,-0.110798,0.0,0.110798,0.193466,0.239658,0.316196,0.0,0.5,1.1,0.9,0.0 };
	static const double CM[nlift] = { 0.0,0.02,0.03,0.06,0.075,-0.08,-0.1,-0.11,-0.09,0.0,0.09,0.11,0.1,0.08,-0.075,-0.06,-0.03,-0.02,0. };
	static const double CD[nlift] = { 1.143,1.0,1.0,0.8,0.8,0.72946,0.65157,0.63798,0.65136,0.5778,0.65136,0.63798,0.65157,0.72946,0.8,0.8,1.0,1.0,1.143 };
	int j;
	factor = 2.0;
	for (j = 0; (j < 16) && (Mach[j + 1] < M); j++);
	frac = (M - Mach[j]) / (Mach[j + 1] - Mach[j]);
	drag = (frac*DFactor[j + 1] + (1.0 - frac)*DFactor[j]);
	lift = drag * (frac*LFactor[j + 1] + (1.0 - frac)*LFactor[j]);
	for (j = 0; (j < nlift - 1) && (AOA[j + 1] < aoa); j++);
	frac = (aoa - AOA[j]) / (AOA[j + 1] - AOA[j]);
	*cd = drag * (frac*CD[j + 1] + (1.0 - frac)*CD[j]);
	*cl = lift * (frac*CL[j + 1] + (1.0 - frac)*CL[j]);
	*cm = factor * (frac*CM[j + 1] + (1.0 - frac)*CM[j]);
}

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
	LOAD_MESH(hCMnh, "ProjectApollo/CM-Nohatch");
	LOAD_MESH(hCM2, "ProjectApollo/CM-Recov");
	LOAD_MESH(hCMP, "ProjectApollo/CM-CMP");
	LOAD_MESH(hCMInt, "ProjectApollo/CM-Interior");
	LOAD_MESH(hCMVC, "ProjectApollo/CM-VC");
	LOAD_MESH(hCREW, "ProjectApollo/CM-CREW");
	LOAD_MESH(hFHC, "ProjectApollo/CM-HatchC");
	LOAD_MESH(hFHO, "ProjectApollo/CM-HatchO");
	LOAD_MESH(hFHF, "ProjectApollo/CM-HatchF");
	LOAD_MESH(hCM2B, "ProjectApollo/CMB-Recov");
	LOAD_MESH(hprobe, "ProjectApollo/CM-Probe");
	LOAD_MESH(hprobeext, "ProjectApollo/CM-ProbeExtended");
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
	LOAD_MESH(hopticscover, "ProjectApollo/CM-OpticsCover");
	LOAD_MESH(hcmdocktgt, "ProjectApollo/CM-Docktgt");

	SURFHANDLE contrail_tex = oapiRegisterParticleTexture("Contrail2");
	lem_exhaust.tex = contrail_tex;
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
		SPSEngine.DefineAnimations(SPSidx);
	}
}




///\todo needs to be redesigned

/*
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

		// And the Crew
		if (Crewed) {
			cmpidx = AddMesh (hCMP, &mesh_dir);
			crewidx = AddMesh (hCREW, &mesh_dir);
			SetCrewMesh();
		} else {
			cmpidx = -1;
			crewidx = -1;
		}

		meshidx = AddMesh (hCMInt, &mesh_dir);
		SetMeshVisibilityMode (meshidx, MESHVIS_EXTERNAL);

		meshidx = AddMesh (hCMVC, &mesh_dir);
		SetMeshVisibilityMode (meshidx, MESHVIS_VC);
		VCMeshOffset = mesh_dir;

		meshidx = AddMesh (hCM, &mesh_dir);
		SetMeshVisibilityMode (meshidx, MESHVIS_VCEXTERNAL);

		//Don't Forget the Hatch
		meshidx = AddMesh (hFHO, &mesh_dir);
		SetMeshVisibilityMode (meshidx, MESHVIS_VCEXTERNAL);

		HatchOpen = true;
		if (HasProbe) {
			probeidx = AddMesh(hprobe, &mesh_dir);
			probeextidx = AddMesh(hprobeext, &mesh_dir);
			SetDockingProbeMesh();
		} else {
			probeidx = -1;
			probeextidx = -1;
		}
	}
	else 
	{
		EVA_IP = true;

		ClearMeshes();
		VECTOR3 mesh_dir=_V(0,SMVO,30.25-12.25-21.5);
		AddMesh (hSM, &mesh_dir);
		mesh_dir=_V(-1.308,-1.18,29.042-12.25-21.5);
		AddMesh (hSMhga, &mesh_dir);

		mesh_dir=_V(0,0,34.4-12.25-21.5);

		// And the Crew, CMP is outside
		if (Crewed) {
			crewidx = AddMesh (hCREW, &mesh_dir);
			SetCrewMesh();
		} else {
			crewidx = -1;
		}
		cmpidx = -1;

		meshidx = AddMesh (hCMInt, &mesh_dir);
		SetMeshVisibilityMode (meshidx, MESHVIS_EXTERNAL);

		meshidx = AddMesh (hCMVC, &mesh_dir);
		SetMeshVisibilityMode (meshidx, MESHVIS_VC);
		VCMeshOffset = mesh_dir;

		meshidx = AddMesh (hCM, &mesh_dir);
		SetMeshVisibilityMode (meshidx, MESHVIS_VCEXTERNAL);

		//Don't Forget the Hatch
		meshidx = AddMesh (hFHO, &mesh_dir);
		SetMeshVisibilityMode (meshidx, MESHVIS_VCEXTERNAL);

		HatchOpen= true;

		if (HasProbe) {
			probeidx = AddMesh(hprobe, &mesh_dir);
			probeextidx = AddMesh(hprobeext, &mesh_dir);
			SetDockingProbeMesh();
		} else {
			probeidx = -1;
			probeextidx = -1;
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

		// And the Crew, CMP is outside
		if (Crewed) {
			crewidx = AddMesh (hCREW, &mesh_dir);
			SetCrewMesh();
		} else {
			crewidx = -1;
		}
		cmpidx = -1;

		meshidx = AddMesh (hCMInt, &mesh_dir);
		SetMeshVisibilityMode (meshidx, MESHVIS_EXTERNAL);

		meshidx = AddMesh (hCMVC, &mesh_dir);
		SetMeshVisibilityMode (meshidx, MESHVIS_VC);
		VCMeshOffset = mesh_dir;

		meshidx = AddMesh (hCM, &mesh_dir);
		SetMeshVisibilityMode (meshidx, MESHVIS_VCEXTERNAL);

		//Don't Forget the Hatch
		meshidx = AddMesh (hFHO, &mesh_dir);
		SetMeshVisibilityMode (meshidx, MESHVIS_VCEXTERNAL);

		HatchOpen = true;

		if (HasProbe) {
			probeidx = AddMesh(hprobe, &mesh_dir);
			probeextidx = AddMesh(hprobeext, &mesh_dir);
			SetDockingProbeMesh();
		} else {
			probeidx = -1;
			probeextidx = -1;
		}
	}
}
*/


void Saturn::SetCSMStage ()
{
	ClearMeshes();
    ClearThrusterDefinitions();
	ClearEngineIndicators();
	ClearLVGuidLight();
	ClearLVRateLight();
	ClearSIISep();

	//
	// Delete any dangling propellant resources.
	//

	if (ph_ullage1)
	{
		DelPropellantResource(ph_ullage1);
		ph_ullage1 = 0;
	}

	if (ph_ullage2)
	{
		DelPropellantResource(ph_ullage2);
		ph_ullage2 = 0;
	}

	if (ph_ullage3)
	{
		DelPropellantResource(ph_ullage3);
		ph_ullage3 = 0;
	}

	if (ph_1st) {
		DelPropellantResource(ph_1st);
		ph_1st = 0;
	}

	if (ph_2nd) {
		DelPropellantResource(ph_2nd);
		ph_2nd = 0;
	}

	if (ph_3rd) {
		DelPropellantResource(ph_3rd);
		ph_3rd = 0;
	}

	if (ph_sep) {
		DelPropellantResource(ph_sep);
		ph_sep = 0;
	}

	if (ph_sep2) {
		DelPropellantResource(ph_sep2);
		ph_sep2 = 0;
	}

	if (ph_aps1) {
		DelPropellantResource(ph_aps1);
		ph_aps1 = 0;
	}

	if (ph_aps2) {
		DelPropellantResource(ph_aps2);
		ph_aps2 = 0;
	}

	SetSize(10);
	SetCOG_elev(3.5);
	SetEmptyMass(CM_EmptyMass + SM_EmptyMass + (LESAttached ? Abort_Mass : 0.0));

	// ************************* propellant specs **********************************

	if (!ph_sps) {
		ph_sps = CreatePropellantResource(SM_FuelMass, SM_FuelMass); //SPS stage propellant
	}

	SetDefaultPropellantResource (ph_sps); // display SPS stage propellant level in generic HUD

	// *********************** thruster definitions ********************************

	th_sps[0] = CreateThruster(_V(-SPS_YAW_OFFSET * RAD * 5.0, -SPS_PITCH_OFFSET * RAD * 5.0, -5.0), _V(0, 0, 1), SPS_THRUST, ph_sps, SPS_ISP);

	DelThrusterGroup(THGROUP_MAIN, true);
	thg_sps = CreateThrusterGroup(th_sps, 1, THGROUP_MAIN);

	VECTOR3 spspos0 = _V(-0.000043, -0.001129, -5);
	EXHAUSTSPEC es_sps[1] = {
		{ th_sps[0], NULL, &spspos0, NULL, 20.0, 2.25, 0, 0.1, SMExhaustTex, EXHAUST_CONSTANTPOS }
	};

	AddExhaust(es_sps);
	//SetPMI(_V(12, 12, 7));
	SetPMI(_V(4.3972, 4.6879, 1.6220));
	SetCrossSections(_V(40,40,14));
	SetCW(0.1, 0.3, 1.4, 1.4);
	SetRotDrag(_V(0.7,0.7,0.3));
	SetPitchMomentScale(0);
	SetYawMomentScale(0);
	SetLiftCoeffFunc(0);

	const double CGOffset = 12.25+21.5-1.8+0.35;
	AddSM(30.25 - CGOffset, true);

	double td_mass = CM_EmptyMass + SM_EmptyMass + (SM_FuelMass / 2);
	double td_width = 4.0;
	double td_tdph = -6.0;
	double td_height = 5.5;

	ConfigTouchdownPoints(td_mass, td_width, td_tdph, td_height, -0.1);

	VECTOR3 mesh_dir;

	//
	// Skylab SM and Apollo 7 have no HGA.
	//
	if (!NoHGA) {
		UINT HGAidx;
		mesh_dir=_V(-1.308,-1.18,29.042-CGOffset);
		HGAidx = AddMesh (hSMhga, &mesh_dir);
		hga.DefineAnimations(HGAidx);
	}

	mesh_dir=_V(0, 0, 34.4 - CGOffset);

	UINT meshidx;
	meshidx = AddMesh (hCMnh, &mesh_dir);
	SetMeshVisibilityMode (meshidx, MESHVIS_VCEXTERNAL);

	if (LESAttached) {
		TowerOffset = 4.95;
		VECTOR3 mesh_dir_tower = mesh_dir + _V(0, 0, TowerOffset);

		meshidx = AddMesh(hsat5tower, &mesh_dir_tower);
		SetMeshVisibilityMode(meshidx, MESHVIS_VCEXTERNAL);
	}

	// And the Crew
	if (Crewed) {
		cmpidx = AddMesh (hCMP, &mesh_dir);
		crewidx = AddMesh (hCREW, &mesh_dir);
		SetCrewMesh();
	} else {
		cmpidx = -1;
		crewidx = -1;
	}

	//CM docking target
	VECTOR3 dt_dir = _V(0.66, 1.07, 2.1);
	cmdocktgtidx = AddMesh(hcmdocktgt, &dt_dir);
	SetCMdocktgtMesh();

	//Don't Forget the Hatch
	sidehatchidx = AddMesh (hFHC, &mesh_dir);
	sidehatchopenidx = AddMesh (hFHO, &mesh_dir);
	SetSideHatchMesh();

	//Forward Hatch
	fwdhatchidx = AddMesh(hFHF, &mesh_dir);
	SetFwdHatchMesh();

	meshidx = AddMesh (hCMVC, &mesh_dir);
	SetMeshVisibilityMode (meshidx, MESHVIS_VC);
	VCMeshOffset = mesh_dir;

	//Interior
	meshidx = AddMesh(hCMInt, &mesh_dir);
	SetMeshVisibilityMode(meshidx, MESHVIS_EXTERNAL);

	// Docking probe
	if (HasProbe) {
		probeidx = AddMesh(hprobe, &mesh_dir);
		probeextidx = AddMesh(hprobeext, &mesh_dir);
		SetDockingProbeMesh();
	} else {
		probeidx = -1;
		probeextidx = -1;
	}

	// Optics Cover
	opticscoveridx = AddMesh (hopticscover, &mesh_dir);
	SetOpticsCoverMesh();

	// Docking port
	VECTOR3 dockpos = {0,0,35.90-CGOffset};
	VECTOR3 dockdir = {0,0,1};
	VECTOR3 dockrot = {0,1,0};
	SetDockParams(dockpos, dockdir, dockrot);

	//
	// SM RCS
	//
	AddRCSJets(-0.18, SM_RCS_THRUST);

	//
	// CM RCS
	//
	AddRCS_CM(CM_RCS_THRUST, 34.4 - CGOffset, false);

	//
	// Waste dump streams
	//
	wastewaterdump_spec.tex = oapiRegisterParticleTexture("ProjectApollo/WaterDump");
	if (wastewaterdump) DelExhaustStream(wastewaterdump);
	wastewaterdump = AddParticleStream(&wastewaterdump_spec, _V(-1.258, 1.282, 33.69 - CGOffset), _V(-0.57, 0.57, 0.59), WaterController.GetWasteWaterDumpLevelRef());

	urinedump_spec.tex = oapiRegisterParticleTexture("ProjectApollo/UrineDump");
	if (urinedump) DelExhaustStream(urinedump);
	urinedump = AddParticleStream(&urinedump_spec, _V(-1.358, 1.192, 33.69 - CGOffset), _V(-0.57, 0.57, 0.59), WaterController.GetUrineDumpLevelRef());

	//
	// Apollo 13 special handling
	//

	if (ApolloExploded && !ph_o2_vent) {

		double tank_mass = CSM_O2TANK_CAPACITY / 1000.0;

		ph_o2_vent = CreatePropellantResource(tank_mass, tank_mass); //"Thruster" created by O2 venting

		TankQuantities t;

		GetTankQuantities(t);

		SetPropellantMass(ph_o2_vent, t.O2Tank1QuantityKg);

	}
	
	if (ApolloExploded) {
		VECTOR3 vent_pos = {0, 1.5, 30.25 - CGOffset};
		VECTOR3 vent_dir = {0.5, 1, 0};

		th_o2_vent = CreateThruster (vent_pos, vent_dir, 30.0, ph_o2_vent, 300.0);
		AddExhaustStream(th_o2_vent, &o2_venting_spec);
	}


	SetView(0.4 + 1.8 - 0.35);

	// **************************** NAV radios *************************************
	InitNavRadios (4);
	EnableTransponder (true);
	OrbiterAttitudeToggle.SetActive(true);
}

void Saturn::CreateSIVBStage(char *config, VESSELSTATUS &vs1, bool SaturnVStage)

{
	char VName[256]="";

	GetApolloName(VName); strcat (VName, "-S4BSTG");
	hs4bM = oapiCreateVessel(VName, config, vs1);

	SIVBSettings S4Config;

	//
	// For now we'll only seperate the panels on ASTP.
	//

	S4Config.SettingsType.word = 0;
	S4Config.SettingsType.SIVB_SETTINGS_FUEL = 1;
	S4Config.SettingsType.SIVB_SETTINGS_GENERAL = 1;
	S4Config.SettingsType.SIVB_SETTINGS_MASS = 1;
	S4Config.SettingsType.SIVB_SETTINGS_PAYLOAD = 1;
	S4Config.SettingsType.SIVB_SETTINGS_ENGINES = 1;
	S4Config.SettingsType.SIVB_SETTINGS_PAYLOAD_INFO = 1;
	S4Config.Payload = SIVBPayload;
	S4Config.VehicleNo = VehicleNo;
	S4Config.EmptyMass = S4B_EmptyMass;
	S4Config.MainFuelKg = GetPropellantMass(ph_3rd);
	S4Config.ApsFuel1Kg = GetPropellantMass(ph_aps1);
	S4Config.ApsFuel2Kg = GetPropellantMass(ph_aps2);
	S4Config.PayloadMass = S4PL_Mass;
	S4Config.SaturnVStage = SaturnVStage;
	S4Config.IUSCContPermanentEnabled = IUSCContPermanentEnabled;
	S4Config.MissionNo = ApolloNo;
	S4Config.MissionTime = MissionTime;
	S4Config.LowRes = LowRes;
	S4Config.ISP_VAC = ISP_THIRD_VAC;
	S4Config.THRUST_VAC = THRUST_THIRD_VAC;
	S4Config.PanelsHinged = !SLAWillSeparate;
	S4Config.SLARotationLimit = (double) SLARotationLimit;
	S4Config.PanelProcess = 0.0;

	GetPayloadName(S4Config.PayloadName);
	strncpy(S4Config.CSMName, GetName(), 63);
	S4Config.Crewed = Crewed;

	S4Config.LMAscentFuelMassKg = LMAscentFuelMassKg;
	S4Config.LMDescentFuelMassKg = LMDescentFuelMassKg;
	S4Config.LMAscentEmptyMassKg = LMAscentEmptyMassKg;
	S4Config.LMDescentEmptyMassKg = LMDescentEmptyMassKg;
	S4Config.LMPad = LMPad;
	S4Config.LMPadCount = LMPadCount;
	S4Config.AEAPad = AEAPad;
	S4Config.AEAPadCount = AEAPadCount;
	sprintf(S4Config.LEMCheck, LEMCheck);

	S4Config.iu_pointer = iu;
	DontDeleteIU = true;

	SIVB *SIVBVessel = static_cast<SIVB *> (oapiGetVesselInterface(hs4bM));
	SIVBVessel->SetState(S4Config);

	PayloadDataTransfer = true;
}

void Saturn::SetDockingProbeMesh() {

	if (probeidx == -1 || probeextidx == -1)
		return;

	if (HasProbe) {
		if (!dockingprobe.IsRetracted()) {
			SetMeshVisibilityMode(probeidx, MESHVIS_NEVER);
			SetMeshVisibilityMode(probeextidx, MESHVIS_VCEXTERNAL);
		} else {
			SetMeshVisibilityMode(probeidx, MESHVIS_VCEXTERNAL);
			SetMeshVisibilityMode(probeextidx, MESHVIS_NEVER);
		}
	} else {
		SetMeshVisibilityMode(probeidx, MESHVIS_NEVER);
		SetMeshVisibilityMode(probeextidx, MESHVIS_NEVER);
	}
}

void Saturn::SetSideHatchMesh() {

	if (sidehatchidx == -1 || sidehatchopenidx == -1)
		return;

	if (SideHatch.IsOpen()) {
		SetMeshVisibilityMode(sidehatchidx, MESHVIS_NEVER);
		SetMeshVisibilityMode(sidehatchopenidx, MESHVIS_VCEXTERNAL);
	} else {
		SetMeshVisibilityMode(sidehatchidx, MESHVIS_VCEXTERNAL);
		SetMeshVisibilityMode(sidehatchopenidx, MESHVIS_NEVER);
	}

	if (sidehatchburnedidx == -1 || sidehatchburnedopenidx == -1)
		return;

	if (!Burned) {
		SetMeshVisibilityMode(sidehatchburnedidx, MESHVIS_NEVER);
		SetMeshVisibilityMode(sidehatchburnedopenidx, MESHVIS_NEVER);
		return;
	}

	SetMeshVisibilityMode(sidehatchidx, MESHVIS_NEVER);
	SetMeshVisibilityMode(sidehatchopenidx, MESHVIS_NEVER);

	if (SideHatch.IsOpen()) {
		SetMeshVisibilityMode(sidehatchburnedidx, MESHVIS_NEVER);
		SetMeshVisibilityMode(sidehatchburnedopenidx, MESHVIS_VCEXTERNAL);
	} else {
		SetMeshVisibilityMode(sidehatchburnedidx, MESHVIS_VCEXTERNAL);
		SetMeshVisibilityMode(sidehatchburnedopenidx, MESHVIS_NEVER);
	}
}

void Saturn::SetFwdHatchMesh() {

	if (fwdhatchidx == -1)
		return;

	if (ForwardHatch.IsOpen()) {
		SetMeshVisibilityMode(fwdhatchidx, MESHVIS_NEVER);
	}
	else {
		SetMeshVisibilityMode(fwdhatchidx, MESHVIS_EXTERNAL);
	}
}

void Saturn::SetCrewMesh() {

	if (cmpidx != -1) {
		if (Crewed && (Crew->number == 1 || Crew->number >= 3)) {
			SetMeshVisibilityMode(cmpidx, MESHVIS_EXTERNAL);
		} else {
			SetMeshVisibilityMode(cmpidx, MESHVIS_NEVER);
		}
	}
	if (crewidx != -1) {
		if (Crewed && Crew->number >= 2) {
			SetMeshVisibilityMode(crewidx, MESHVIS_EXTERNAL);
		} else {
			SetMeshVisibilityMode(crewidx, MESHVIS_NEVER);
		}
	}
}

void Saturn::SetOpticsCoverMesh() {

	if (opticscoveridx == -1)
		return;
	
	if (optics.OpticsCovered) {
		SetMeshVisibilityMode(opticscoveridx, MESHVIS_EXTERNAL);
	} else {
		SetMeshVisibilityMode(opticscoveridx, MESHVIS_NEVER);
	}
}

void Saturn::SetCMdocktgtMesh() {

	if (cmdocktgtidx == -1)
		return;

	if (CMdocktgt && ApexCoverAttached) {
		SetMeshVisibilityMode(cmdocktgtidx, MESHVIS_VCEXTERNAL);
	}
	else {
		SetMeshVisibilityMode(cmdocktgtidx, MESHVIS_NEVER);
	}
}

void Saturn::SetNosecapMesh() {

	if (nosecapidx == -1)
		return;

	if (NosecapAttached) {
		SetMeshVisibilityMode(nosecapidx, MESHVIS_EXTERNAL);
	}
	else {
		SetMeshVisibilityMode(nosecapidx, MESHVIS_NEVER);
	}
}

void Saturn::ProbeVis() {

	if (!probe)
		return;

	GROUPEDITSPEC ges;

	if (ForwardHatch.IsOpen()) {
		ges.flags = (GRPEDIT_ADDUSERFLAG);
		ges.UsrFlag = 3;
		oapiEditMeshGroup(probe, 2, &ges);
	}
	else
	{
		ges.flags = (GRPEDIT_SETUSERFLAG);
		ges.UsrFlag = 0;
		oapiEditMeshGroup(probe, 2, &ges);
	}
}

void Saturn::SetReentryStage ()

{
    ClearThrusters();
	ClearPropellants();
	ClearAirfoilDefinitions();
	ClearVariableDragElements();
	ClearEngineIndicators();
	ClearLVGuidLight();
	ClearLVRateLight();
	ClearSIISep();
	hga.DeleteAnimations();
	SPSEngine.DeleteAnimations();
	double EmptyMass = CM_EmptyMass + (LESAttached ? 2000.0 : 0.0);
	SetSize(6.0);
	SetEmptyMass(EmptyMass);

	double td_mass = 5430.0;
	double td_width = 2.0;
	double td_tdph = -2.5;
	if (ApexCoverAttached) {
		td_tdph = -1.3;
	}
	double td_height = 5.0;

	ConfigTouchdownPoints(td_mass, td_width, td_tdph, td_height);

	if (LESAttached)
	{
		SetPMI(_V(15.0, 15.0, 1.5));
		SetRotDrag(_V(1.5, 1.5, 0.003));
	}
	else
	{
		SetPMI(_V(1.25411, 1.11318, 1.41524)); //Calculated from CSM-109 Mass Properties at CM/SM Separation
		SetRotDrag(_V(0.07, 0.07, 0.002));
	}
	SetCrossSections (_V(9.17,7.13,7.0));
	SetCW(1.5, 1.5, 1.2, 1.2);
	SetSurfaceFrictionCoeff(1, 1);
	if (GetFlightModel() >= 1) {
		if (LESAttached)
		{
			if (canard.IsDeployed())
			{
				CMLETCanardAirfoilConfig();
			}
			else
			{
				CMLETAirfoilConfig();
			}
		}
		else
		{
			CreateAirfoil(LIFT_VERTICAL, _V(0.0, 0.12, 1.12), CMCoeffFunc, 3.5, 11.95, 1.0);
		}
    }

	SetReentryMeshes();
	if (ApexCoverAttached) {
		SetView(-0.15);
	} else {
		SetView(-1.35);
	}
	if (CMTex) SetReentryTexture(CMTex, 1e6, 5, 0.7);

	// CM RCS
	double CGOffset = 34.4;
	if (ApexCoverAttached) {
		AddRCS_CM(CM_RCS_THRUST);
	} else {
		AddRCS_CM(CM_RCS_THRUST, -1.2);
		CGOffset += 1.2;
	}

	if (LESAttached) {
		//if (!ph_tjm)
		//	ph_tjm  = CreatePropellantResource(93.318);
		if (!ph_lem)
			ph_lem = CreatePropellantResource(1425.138);
		if (!ph_pcm)
			ph_pcm = CreatePropellantResource(4.07247);

		SetDefaultPropellantResource (ph_lem); // display LEM propellant level in generic HUD

		//
		// *********************** thruster definitions ********************************
		//

		VECTOR3 m_exhaust_pos1 = _V(0.0, -0.5, TowerOffset-2.2);
		VECTOR3 m_exhaust_pos2 = _V(0.0, 0.5, TowerOffset-2.2);
		VECTOR3 m_exhaust_pos3 = _V(-0.5, 0.0, TowerOffset-2.2);
		VECTOR3 m_exhaust_pos4 = _V(0.5, 0.0, TowerOffset-2.2);

		//
		// Main thrusters.
		//

		th_lem[0] = CreateThruster (m_exhaust_pos1, _V(0.0, sin(35.0*RAD), cos(35.0*RAD)), THRUST_VAC_LEM, ph_lem, ISP_LEM_VAC, ISP_LEM_SL);
		th_lem[1] = CreateThruster (m_exhaust_pos2, _V(0.0, -sin(35.0*RAD), cos(35.0*RAD)), THRUST_VAC_LEM, ph_lem, ISP_LEM_VAC, ISP_LEM_SL);
		th_lem[2] = CreateThruster (m_exhaust_pos3, _V(sin(35.0*RAD), 0.0, cos(35.0*RAD)), THRUST_VAC_LEM, ph_lem, ISP_LEM_VAC, ISP_LEM_SL);
		th_lem[3] = CreateThruster (m_exhaust_pos4, _V(-sin(35.0*RAD), 0.0, cos(35.0*RAD)), THRUST_VAC_LEM, ph_lem, ISP_LEM_VAC, ISP_LEM_SL);

		//th_tjm[0] = CreateThruster(_V(0.0, -0.5, TowerOffset), _V(0.030524, 0.49907, 0.8660254), THRUST_VAC_TJM, ph_tjm, ISP_TJM_VAC, ISP_TJM_SL);
		//th_tjm[1] = CreateThruster(_V(0.0, 0.5, TowerOffset), _V(0.030524, -0.49907, 0.8660254), THRUST_VAC_TJM, ph_tjm, ISP_TJM_VAC, ISP_TJM_SL);

		th_pcm = CreateThruster(_V(0.0, 0.0, TowerOffset + 4.5), _V(0.0, 1.0, 0.0), THRUST_VAC_PCM, ph_pcm, ISP_PCM_VAC, ISP_PCM_SL);

		//
		// Add exhausts
		//

		int i;
		for (i = 0; i < 4; i++)
		{
			AddExhaust (th_lem[i], 8.0, 0.5, SIVBRCSTex);
			AddExhaustStream (th_lem[i], &lem_exhaust);
		}
		//for (i = 0; i < 2; i++)
		//{
		//	AddExhaust(th_tjm[i], 8.0, 0.5, SIVBRCSTex);
		//	AddExhaustStream(th_tjm[i], &lem_exhaust);
		//}
		AddExhaust(th_pcm, 8.0, 0.5, SIVBRCSTex);
		AddExhaustStream(th_pcm, &lem_exhaust);

		thg_lem = CreateThrusterGroup (th_lem, 4, THGROUP_USER);
		//thg_tjm = CreateThrusterGroup(th_tjm, 2, THGROUP_USER);
	}

	VECTOR3 dockpos = {0, 0, 1.5};
	VECTOR3 dockdir = {0, 0, 1};
	VECTOR3 dockrot = {0, 1, 0};
	SetDockParams(dockpos, dockdir, dockrot);

	if (!DrogueS.isValid())
		soundlib.LoadMissionSound(DrogueS, DROGUES_SOUND);

	//
	// Waste dump streams
	//

	wastewaterdump_spec.tex = oapiRegisterParticleTexture("ProjectApollo/WaterDump");
	if (wastewaterdump) DelExhaustStream(wastewaterdump);
	wastewaterdump = AddParticleStream(&wastewaterdump_spec, _V(-1.258, 1.282, 33.69 - CGOffset), _V(-0.57, 0.57, 0.59), WaterController.GetWasteWaterDumpLevelRef());

	urinedump_spec.tex = oapiRegisterParticleTexture("ProjectApollo/UrineDump");
	if (urinedump) DelExhaustStream(urinedump);
	urinedump = AddParticleStream(&urinedump_spec, _V(-1.358, 1.192, 33.69 - CGOffset), _V(-0.57, 0.57, 0.59), WaterController.GetUrineDumpLevelRef());
}

void Saturn::SetReentryMeshes() {

	ClearMeshes();

	UINT meshidx;
	VECTOR3 mesh_dir=_V(0,0,0);
	if (Burned)	{
		if (ApexCoverAttached) {
			meshidx = AddMesh (hCMB, &mesh_dir);
		} else {
			mesh_dir=_V(0, 0, -1.2);
			meshidx = AddMesh (hCM2B, &mesh_dir);
		}
	} else {
		if (ApexCoverAttached) {
			meshidx = AddMesh (hCMnh, &mesh_dir);
		} else {
			mesh_dir=_V(0, 0, -1.2);
			meshidx = AddMesh (hCM2, &mesh_dir);
		}
	}
	SetMeshVisibilityMode (meshidx, MESHVIS_VCEXTERNAL);

	if (LESAttached) {
		TowerOffset = 4.95;
		VECTOR3 mesh_dir_tower = mesh_dir + _V(0, 0, TowerOffset);

		meshidx = AddMesh (hsat5tower, &mesh_dir_tower);
		SetMeshVisibilityMode (meshidx, MESHVIS_VCEXTERNAL);
	}

	// And the Crew
	if (Crewed) {		
		cmpidx = AddMesh (hCMP, &mesh_dir);
		crewidx = AddMesh (hCREW, &mesh_dir);
		SetCrewMesh();
	} else {
		cmpidx = -1;
		crewidx = -1;
	}
	
	//CM docking target
	VECTOR3 dt_dir = _V(0.66, 1.07, 0);
	cmdocktgtidx = AddMesh(hcmdocktgt, &dt_dir);
	SetCMdocktgtMesh();

	// Hatch
	sidehatchidx = AddMesh (hFHC, &mesh_dir);
	sidehatchopenidx = AddMesh (hFHO, &mesh_dir);
	sidehatchburnedidx = AddMesh (hFHC2, &mesh_dir);
	sidehatchburnedopenidx = AddMesh (hFHO2, &mesh_dir);
	SetSideHatchMesh();

	//Forward Hatch
	if (ApexCoverAttached) {
		fwdhatchidx = AddMesh(hFHF, &mesh_dir);
		SetFwdHatchMesh();
	}

	//Interior
	meshidx = AddMesh (hCMInt, &mesh_dir);
	SetMeshVisibilityMode (meshidx, MESHVIS_EXTERNAL);

	meshidx = AddMesh (hCMVC, &mesh_dir);
	SetMeshVisibilityMode (meshidx, MESHVIS_VC);

	//
	// Docking probe
	//

	if (HasProbe)
	{
		probeidx = AddMesh(hprobe, &mesh_dir);
		probeextidx = AddMesh(hprobeext, &mesh_dir);
		SetDockingProbeMesh();
	} else
	{
		probeidx = -1;
		probeextidx = -1;
	}
	VCMeshOffset = mesh_dir;
}

void Saturn::StageSeven(double simt)

{
	if (!Crewed)
	{
		switch (StageState) {
		case 0:
			if (GetAltitude() < 350000) {
				SlowIfDesired();
				ActivateCMRCS();
				ActivateNavmode(NAVMODE_RETROGRADE);
				StageState++;
			}
			break;
		}
	}

	// Entry heat according to Orbiter reference manual
	double entryHeat = 0.5 * GetAtmDensity() * pow(GetAirspeed(), 3);
	if (entryHeat > 2e7 ) { // We 're looking wether the CM has burned or not
		Burned = true;
		SetReentryMeshes();

		ClearThrusters();
		AddRCS_CM(CM_RCS_THRUST);

		SetStage(CM_ENTRY_STAGE);
		SetView(-0.15);
	}
}

void Saturn::StageEight(double simt)

{
	ConfigTouchdownPoints(CM_EmptyMass, 2.0, -2.5, 5.0);

	// Mark apex as detached
	ApexCoverAttached = false;
	SetReentryMeshes();

	ClearThrusters();
	AddRCS_CM(CM_RCS_THRUST, -1.2);

	SetView(-1.35);

	if (!Crewed)
	{
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

	//
	// Create the apex cover vessel
	//
	VECTOR3 posOffset = _V(0, 0, 0);
	VECTOR3 velOffset = _V(0, 0, 3);

	VESSELSTATUS vs;
	GetStatus(vs);
	Local2Rel(posOffset, vs.rpos);
	VECTOR3 vog;
	GlobalRot(velOffset, vog);
	vs.rvel += vog;

	char VName[256]="";
	GetApolloName(VName);
	strcat(VName, "-APEX");
	if (Burned) {
		hApex = oapiCreateVessel(VName,"ProjectApollo/CMBapex", vs);
	} else {
		hApex = oapiCreateVessel(VName,"ProjectApollo/CMapex", vs);
	}

	// New stage
	SetStage(CM_ENTRY_STAGE_TWO);
}

void Saturn::SetChuteStage1()
{
	SetSize(15);
	SetCOG_elev(2.2);
	SetEmptyMass(CM_EmptyMass);
	ConfigTouchdownPoints(CM_EmptyMass, 2.0, -2.5, 5.0);
	ClearAirfoilDefinitions();
	SetPMI(_V(20,20,12));
	SetCrossSections(_V(2.8,2.8,80.0));
	SetCW(1.0, 1.5, 1.4, 1.4);
	SetRotDrag(_V(0.7,0.7,1.2));
	SetSurfaceFrictionCoeff(1, 1);
	if (GetFlightModel() >= 1)
	{
		SetPitchMomentScale(-5e-3);
		SetYawMomentScale(-5e-3);
	}
	SetLiftCoeffFunc(0);
    ClearExhaustRefs();
    ClearAttExhaustRefs();

	SetReentryMeshes();
	
	ClearThrusters();
	AddRCS_CM(CM_RCS_THRUST, -1.2);
	SetView(-1.35);

	DeactivateNavmode(NAVMODE_KILLROT);
}

void Saturn::SetChuteStage2()
{
	SetSize(22);
	SetCOG_elev(2.2);
	SetEmptyMass (CM_EmptyMass);
	ConfigTouchdownPoints(CM_EmptyMass, 2.0, -2.5, 5.0);
	SetPMI (_V(20,20,12));
	SetCrossSections (_V(2.8,2.8,140.0));
	SetCW (1.0, 1.5, 1.4, 1.4);
	SetRotDrag (_V(0.7,0.7,1.2));
	SetSurfaceFrictionCoeff(1, 1);
	if (GetFlightModel() >= 1)
	{
		SetPitchMomentScale (-5e-3);
		SetYawMomentScale (-5e-3);
	}
	SetLiftCoeffFunc(0);
    ClearExhaustRefs();
    ClearAttExhaustRefs();

	SetReentryMeshes();

	ClearThrusters();
	AddRCS_CM(CM_RCS_THRUST, -1.2);
	SetView(-1.35);
}

void Saturn::SetChuteStage3()
{
	SetSize(22);
	SetCOG_elev(2.2);
	SetEmptyMass (CM_EmptyMass);
	ConfigTouchdownPoints(CM_EmptyMass, 2.0, -2.5, 5.0);
	SetPMI(_V(20,20,12));
	SetCrossSections(_V(2.8,2.8,480.0));
	SetCW(0.7, 1.5, 1.4, 1.4);
	SetRotDrag(_V(0.7,0.7,1.2));
	SetSurfaceFrictionCoeff(1, 1);
	if (GetFlightModel() >= 1)
	{
		SetPitchMomentScale(-5e-3);
		SetYawMomentScale(-5e-3);
	}
	SetLiftCoeffFunc (0);
    ClearExhaustRefs();
    ClearAttExhaustRefs();

	SetReentryMeshes();

	ClearThrusters();
	AddRCS_CM(CM_RCS_THRUST, -1.2);
	SetView(-1.35);
}

void Saturn::SetChuteStage4()
{
	SetSize(22);
	SetCOG_elev(2.2);
	SetEmptyMass(CM_EmptyMass);
	ConfigTouchdownPoints(CM_EmptyMass, 2.0, -2.5, 5.0);
	SetPMI(_V(20,20,12));
	SetCrossSections (_V(2.8,2.8,3280.0));
	SetCW (0.7, 1.5, 1.4, 1.4);
	SetRotDrag(_V(0.7, 0.7, 1.2));
	SetSurfaceFrictionCoeff(1, 1);
	if (GetFlightModel() >= 1)
	{
		SetPitchMomentScale (-5e-3);
		SetYawMomentScale (-5e-3);
	}
	SetLiftCoeffFunc(0);
    ClearExhaustRefs();
    ClearAttExhaustRefs();

	SetReentryMeshes();

	ClearThrusters();
	AddRCS_CM(CM_RCS_THRUST, -1.2);
	SetView(-1.35);
}

void Saturn::SetSplashStage()
{
	SetSize(6.0);
	SetCOG_elev(2.2);
	SetEmptyMass(CM_EmptyMass);
	ConfigTouchdownPoints(CM_EmptyMass, 2.0, -2.5, 5.0);
	SetPMI(_V(20,20,12));
	SetCrossSections(_V(2.8,2.8,7.0));
	SetCW(0.5, 1.5, 1.4, 1.4);
	SetRotDrag(_V(0.7,0.7,1.2));
	SetSurfaceFrictionCoeff(1, 1);
	if (GetFlightModel() >= 1)
	{
		SetPitchMomentScale (-5e-3);
		SetYawMomentScale (-5e-3);
	}
	SetLiftCoeffFunc(0);
    ClearExhaustRefs();
    ClearAttExhaustRefs();

	SetReentryMeshes();

	ClearThrusters();
	AddRCS_CM(CM_RCS_THRUST, -1.2);

	dyemarker_spec.tex = oapiRegisterParticleTexture("ProjectApollo/Dyemarker");
	if (dyemarker) DelExhaustStream(dyemarker);
	dyemarker = AddParticleStream(&dyemarker_spec, _V(-0.5, 1.5, -2), _V(-0.8660254, 0.5, 0), els.GetDyeMarkerLevelRef());

	SetView(-1.35);
}

void Saturn::SetRecovery()

{
	SetSize(10.0);
	SetCOG_elev(2.2);
	ConfigTouchdownPoints(CM_EmptyMass, 2.0, -2.5, 5.0);
	SetEmptyMass(CM_EmptyMass);
	SetPMI(_V(20,20,12));
	SetCrossSections(_V(2.8,2.8,7.0));
	SetCW(0.5, 1.5, 1.4, 1.4);
	SetRotDrag(_V(0.7,0.7,1.2));
	SetSurfaceFrictionCoeff(1, 1);
	if (GetFlightModel() >= 1)
	{
		SetPitchMomentScale (-5e-3);
		SetYawMomentScale (-5e-3);
	}
	SetLiftCoeffFunc(0);
    ClearExhaustRefs();
    ClearAttExhaustRefs();

	// Meshes
	ClearMeshes();

	UINT meshidx;
	VECTOR3 mesh_dir=_V(0,0,-1.2);
	if (Burned)	{
		meshidx = AddMesh (hCM2B, &mesh_dir);
	} else {
		meshidx = AddMesh (hCM2, &mesh_dir);
	}
	SetMeshVisibilityMode (meshidx, MESHVIS_VCEXTERNAL);

	// Hatch
	sidehatchidx = AddMesh (hFHC, &mesh_dir);
	sidehatchopenidx = AddMesh (hFHO, &mesh_dir);
	sidehatchburnedidx = AddMesh (hFHC2, &mesh_dir);
	sidehatchburnedopenidx = AddMesh (hFHO2, &mesh_dir);
	SetSideHatchMesh();

	meshidx = AddMesh (hCMInt, &mesh_dir);
	SetMeshVisibilityMode (meshidx, MESHVIS_EXTERNAL);

	meshidx = AddMesh (hCMVC, &mesh_dir);
	SetMeshVisibilityMode (meshidx, MESHVIS_VC);
	VCMeshOffset = mesh_dir;

	if (Crewed) {
		mesh_dir =_V(2.7,1.8,-1.5);
		meshidx = AddMesh (hCRB, &mesh_dir);
		SetMeshVisibilityMode (meshidx, MESHVIS_VCEXTERNAL);
	}

	ClearThrusters();
	AddRCS_CM(CM_RCS_THRUST, -1.2);

	dyemarker_spec.tex = oapiRegisterParticleTexture("ProjectApollo/Dyemarker");
	if (dyemarker) DelExhaustStream(dyemarker);
	dyemarker = AddParticleStream(&dyemarker_spec, _V(-0.5, 1.5, -2), _V(-0.8660254, 0.5, 0), els.GetDyeMarkerLevelRef());

	SetView(-1.35);
}

bool Saturn::clbkLoadGenericCockpit ()

{
	TRACESETUP("Saturn::clbkLoadGenericCockpit");

	//
	// VC-only in engineering camera view.
	//

	if (viewpos == SATVIEW_ENG1 || viewpos == SATVIEW_ENG2 || viewpos == SATVIEW_ENG3)
		return false;

	SetCameraRotationRange(0.0, 0.0, 0.0, 0.0);
	SetCameraDefaultDirection(_V(0.0, 0.0, 1.0));
	oapiCameraSetCockpitDir(0,0);
	InVC = false;
	InPanel = false;

	SetView();
	return true;
}

//
// Generic function to jettison the escape tower.
//

void Saturn::JettisonLET(bool AbortJettison)

{		
	//
	// Don't do anything if the tower isn't attached!
	//
	if (!LESAttached || !LESLegsCut)
		return;

	//
	// Otherwise jettison the LES.
	//
	VECTOR3 ofs1 = _V(0.0, 0.0, TowerOffset);
	VECTOR3 vel1 = _V(0.0,0.0,0.5);

	VESSELSTATUS vs1;
	GetStatus (vs1);

	vs1.eng_main = vs1.eng_hovr = 0.0;

	//
	// We must set status to zero to ensure the LET is in 'free flight'. Otherwise if we jettison
	// on the pad, the LET thinks it's on the ground!
	//

	vs1.status = 0;

	VECTOR3 rofs1, rvel1 = {vs1.rvel.x, vs1.rvel.y, vs1.rvel.z};

	Local2Rel (ofs1, vs1.rpos);

	GlobalRot (vel1, rofs1);

	vs1.rvel.x = rvel1.x+rofs1.x;
	vs1.rvel.y = rvel1.y+rofs1.y;
	vs1.rvel.z = rvel1.z+rofs1.z;

	TowerJS.play();
	TowerJS.done();

	char VName[256];

	GetApolloName(VName);
	strcat (VName, "-TWR");

	hesc1 = oapiCreateVessel(VName, "ProjectApollo/LES", vs1);
	LESAttached = false;

	LESSettings LESConfig;

	LESConfig.SettingsType.word = 0;
	LESConfig.SettingsType.LES_SETTINGS_GENERAL = 1;
	LESConfig.SettingsType.LES_SETTINGS_ENGINES = 1;

	LESConfig.FireLEM = FireLEM;
	LESConfig.FireTJM = FireTJM;
	LESConfig.FirePCM = FirePCM;

	LESConfig.LowRes = LowRes;
	LESConfig.ProbeAttached = AbortJettison && HasProbe;

	if (ph_lem)
	{
		LESConfig.LaunchEscapeFuelKg = GetPropellantMass(ph_lem);
		LESConfig.SettingsType.LES_SETTINGS_MFUEL = 1;
	}
	//if (ph_tjm)
	//{
		//LESConfig.JettisonFuelKg = GetPropellantMass(ph_tjm);
		//LESConfig.SettingsType.LES_SETTINGS_MFUEL = 1;
	//}
	if (ph_pcm)
	{
		LESConfig.PitchControlFuelKg = GetPropellantMass(ph_pcm);
		LESConfig.SettingsType.LES_SETTINGS_PFUEL = 1;
	}

	LES *les_vessel = (LES *) oapiGetVesselInterface(hesc1);
	les_vessel->SetState(LESConfig);

	//
	// AOH SECS page 2.9-8 says that in the case of an abort, the docking probe is pulled away
	// from the CM by the LES when it's jettisoned.
	//
	if (AbortJettison)
	{
		dockingprobe.SetEnabled(false);
		HasProbe = false;
	}
	else
	{
		//
		// Enable docking probe because the tower is gone
		//
		dockingprobe.SetEnabled(HasProbe);			
	}

	ConfigureStageMeshes(stage);

	if (Crewed)
	{
		SwindowS.play();
	}
	SwindowS.done();

	//
	// Event management
	//

	if (eventControl.TOWER_JETTISON == MINUS_INFINITY)
		eventControl.TOWER_JETTISON = MissionTime;
}

void Saturn::JettisonDockingProbe() 

{
	char VName[256];

	// Use VC offset to calculate the docking probe offset
	VECTOR3 ofs = _V(0, 0, CurrentViewOffset + 0.25);
	VECTOR3 vel = {0.0, 0.0, 2.5};
	VESSELSTATUS vs4b;
	GetStatus (vs4b);
	StageTransform(this, &vs4b,ofs,vel);
	vs4b.vrot.x = 0.0;
	vs4b.vrot.y = 0.0;
	vs4b.vrot.z = 0.0;
	GetApolloName(VName); 
	strcat (VName, "-DCKPRB");
	hPROBE = oapiCreateVessel(VName, "ProjectApollo/CMprobe", vs4b);
}

void Saturn::JettisonOpticsCover() 

{
	char VName[256];

	// Use VC offset to calculate the optics cover offset
	VECTOR3 ofs = _V(0, 0, CurrentViewOffset + 0.25);
	VECTOR3 vel = {0.0, -0.16, 0.1};
	VESSELSTATUS vs4b;
	GetStatus (vs4b);
	StageTransform(this, &vs4b, ofs, vel);
	vs4b.vrot.x = 0.05;
	vs4b.vrot.y = 0.0;
	vs4b.vrot.z = 0.0;
	GetApolloName(VName); 
	strcat (VName, "-OPTICSCOVER");
	hOpticsCover = oapiCreateVessel(VName, "ProjectApollo/CMOpticsCover", vs4b);
}

void Saturn::JettisonNosecap()

{
	char VName[256];

	// Use VC offset to calculate the optics cover offset
	VECTOR3 ofs = _V(0, 0, CurrentViewOffset + 0.25);
	VECTOR3 vel = { 0.0, 0.0, 2.5 };
	VESSELSTATUS vs4b;
	GetStatus(vs4b);
	StageTransform(this, &vs4b, ofs, vel);
	vs4b.vrot.x = 0.0;
	vs4b.vrot.y = 0.0;
	vs4b.vrot.z = 0.0;
	GetApolloName(VName);
	strcat(VName, "-NOSECAP");
	hNosecapVessel = oapiCreateVessel(VName, "ProjectApollo/Sat1Aerocap", vs4b);
}

void Saturn::DeployCanard()
{
	if (!LESAttached) return;
	if (canard.IsDeployed()) return;

	canard.Deploy();

	CMLETCanardAirfoilConfig();
}

void Saturn::CMLETAirfoilConfig()
{
	ClearAirfoilDefinitions();

	CreateAirfoil(LIFT_VERTICAL, _V(0.0, 0.0, 1.12), CMLETVertCoeffFunc, 3.5, 11.95 / 2.0, 1.0);
	CreateAirfoil(LIFT_HORIZONTAL, _V(0.0, 0.0, 1.12), CMLETHoriCoeffFunc, 3.5, 11.95 / 2.0, 1.0);
}

void Saturn::CMLETCanardAirfoilConfig()
{
	ClearAirfoilDefinitions();

	CreateAirfoil(LIFT_VERTICAL, _V(0.0, 0.0, 1.12), CMLETCanardVertCoeffFunc, 3.5, 11.95 / 2.0, 1.0);
	CreateAirfoil(LIFT_HORIZONTAL, _V(0.0, 0.0, 1.12), CMLETHoriCoeffFunc, 3.5, 11.95 / 2.0, 1.0);
}
void Saturn::ConfigTouchdownPoints(double mass, double ro, double tdph, double height, double x_target)
{

	TOUCHDOWNVTX td[4];
	double stiffness = (-1)*(mass*9.80655) / (3 * x_target);
	double damping = 0.9*(2 * sqrt(mass*stiffness));
	for (int i = 0; i < 4; i++) {
		td[i].damping = damping;
		td[i].mu = 3;
		td[i].mu_lng = 3;
		td[i].stiffness = stiffness;
	}
	td[0].pos.x = -cos(30 * RAD)*ro;
	td[0].pos.y = -sin(30 * RAD)*ro;
	td[0].pos.z = tdph;
	td[1].pos.x = 0;
	td[1].pos.y = 1 * ro;
	td[1].pos.z = tdph;
	td[2].pos.x = cos(30 * RAD)*ro;
	td[2].pos.y = -sin(30 * RAD)*ro;
	td[2].pos.z = tdph;
	td[3].pos.x = 0;
	td[3].pos.y = 0;
	td[3].pos.z = tdph + height;

	SetTouchdownPoints(td, 4);
}