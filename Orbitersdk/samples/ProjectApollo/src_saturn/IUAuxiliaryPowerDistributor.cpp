/***************************************************************************
This file is part of Project Apollo - NASSP
Copyright 2019

IU Auxiliary Power Distributors 601A33, 602A34

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
#include "iu.h"
#include "IUAuxiliaryPowerDistributor.h"

IUAuxiliaryPowerDistributor1::IUAuxiliaryPowerDistributor1(IU *iu)
{
	this->iu = iu;

	MotorSwitchLogic = false;
	CommandPowerTransfer = false;
}

void IUAuxiliaryPowerDistributor1::Timestep(double simdt)
{
	if (CommandPowerTransfer && iu->IsUmbilicalConnected())
		MotorSwitchLogic = true;
	else
		MotorSwitchLogic = false;
}

IUAuxiliaryPowerDistributor2::IUAuxiliaryPowerDistributor2(IU *iu)
{
	this->iu = iu;

	EDSBus1PowerOff = false;
	EDSBus2PowerOff = false;
	EDSBus3PowerOff = false;
}

void IUAuxiliaryPowerDistributor2::Timestep(double simdt)
{
	if (iu->ESEGetEDSPowerInhibit())
	{
		EDSBus1PowerOff = true;
		EDSBus2PowerOff = true;
		EDSBus3PowerOff = true;
	}
	else
	{
		EDSBus1PowerOff = false;
		EDSBus2PowerOff = false;
		EDSBus3PowerOff = false;
	}
}