//###########################################################################
// This file is part of LImA, a Library for Image Acquisition
//
// Copyright (C) : 2009-2011
// European Synchrotron Radiation Facility
// BP 220, Grenoble 38043
// FRANCE
//
// This is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 3 of the License, or
// (at your option) any later version.
//
// This software is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, see <http://www.gnu.org/licenses/>.
//###########################################################################

namespace SlsDetector
{

%TypeHeaderCode
#include "SlsDetectorDefs.h"
%End

namespace Defs
{

%TypeHeaderCode
#include "SlsDetectorDefs.h"
%End

enum TrigMode {
	Auto            = AUTO_TIMING,
	TriggerExposure = TRIGGER_EXPOSURE,
	TriggerReadout  = TRIGGER_READOUT,
	Gating          = GATE_FIX_NUMBER,
	TriggeredGating = GATE_WITH_START_TRIGGER,
	BurstTrigger    = BURST_TRIGGER,
};

enum Settings {
	Standard      = STANDARD,
	Fast          = FAST,
	HighGain      = HIGHGAIN,
	DynamicGain   = DYNAMICGAIN,
	LowGain       = LOWGAIN,
	MediumGain    = MEDIUMGAIN,
	VeryHighGain  = VERYHIGHGAIN,
	LowNoise      = LOWNOISE,
	DynamicHG0    = DYNAMICHG0,
	FixGain1      = FIXGAIN1,
	FixGain2      = FIXGAIN2,
	ForceSwitchG1 = FORCESWITCHG1,
	ForceSwitchG2 = FORCESWITCHG2,
	VeryLowGain   = VERYLOWGAIN,
	Undefined     = UNDEFINED,
	Unitialized   = UNINITIALIZED,
};

}; // namespace Defs

}; // namespace SlsDetector
