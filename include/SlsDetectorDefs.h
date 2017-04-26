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

#ifndef __SLS_DETECTOR_DEFS_H
#define __SLS_DETECTOR_DEFS_H

#include "sls_detector_defs.h"

namespace lima 
{

namespace SlsDetector
{

namespace Defs
{

enum TrigMode {
	Auto            = slsDetectorDefs::AUTO_TIMING, 
	TriggerExposure = slsDetectorDefs::TRIGGER_EXPOSURE, 
	TriggerReadout  = slsDetectorDefs::TRIGGER_READOUT,
	Gating          = slsDetectorDefs::GATE_FIX_NUMBER, 
	TriggeredGating = slsDetectorDefs::GATE_WITH_START_TRIGGER,
	BurstTrigger    = slsDetectorDefs::BURST_TRIGGER,
};

enum Settings {
	Standard      = slsDetectorDefs::STANDARD,
	Fast          = slsDetectorDefs::FAST,
	HighGain      = slsDetectorDefs::HIGHGAIN,
	DynamicGain   = slsDetectorDefs::DYNAMICGAIN,
	LowGain       = slsDetectorDefs::LOWGAIN,
	MediumGain    = slsDetectorDefs::MEDIUMGAIN,
	VeryHighGain  = slsDetectorDefs::VERYHIGHGAIN,
	LowNoise      = slsDetectorDefs::LOWNOISE,
	DynamicHG0    = slsDetectorDefs::DYNAMICHG0,
	FixGain1      = slsDetectorDefs::FIXGAIN1,
	FixGain2      = slsDetectorDefs::FIXGAIN2,
	ForceSwitchG1 = slsDetectorDefs::FORCESWITCHG1,
	ForceSwitchG2 = slsDetectorDefs::FORCESWITCHG2,
	VeryLowGain   = slsDetectorDefs::VERYLOWGAIN,
	Undefined     = slsDetectorDefs::UNDEFINED,
	Unitialized   = slsDetectorDefs::UNINITIALIZED,
};

std::ostream& operator <<(std::ostream& os, TrigMode trig_mode);
std::ostream& operator <<(std::ostream& os, Settings settings);

} // namespace Defs

} // namespace SlsDetector

} // namespace lima



#endif // __SLS_DETECTOR_DEFS_H
