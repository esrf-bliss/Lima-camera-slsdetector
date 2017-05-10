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

#include "SlsDetectorCamera.h"
#include "lima/MiscUtils.h"

using namespace std;
using namespace lima;
using namespace lima::SlsDetector;
using namespace lima::SlsDetector::Defs;

ostream& lima::SlsDetector::Defs::operator <<(ostream& os, TrigMode trig_mode)
{
	const char *name = "Invalid";
	switch (trig_mode) {
	case Auto:		name = "Auto";			break;
	case TriggerExposure:	name = "TriggerExposure";	break;
	case TriggerReadout:	name = "TriggerReadout";	break;
	case Gating:		name = "Gating";		break;
	case TriggeredGating:	name = "TriggeredGating";	break;
	case BurstTrigger:	name = "BurstTrigger";		break;
	}
	return os << name;
}

ostream& lima::SlsDetector::Defs::operator <<(ostream& os, Settings settings)
{
	const char *name = "Unknown";
	switch (settings) {
	case Standard:		name = "Standard";		break;
	case Fast:		name = "Fast";			break;
	case HighGain:		name = "HighGain";		break;
	case DynamicGain:	name = "DynamicGain";		break;
	case LowGain:		name = "LowGain";		break;
	case MediumGain:	name = "MediumGain";		break;
	case VeryHighGain:	name = "VeryHighGain";		break;
	case LowNoise:		name = "LowNoise";		break;
	case DynamicHG0:	name = "DynamicHG0";		break;
	case FixGain1:		name = "FixGain1";		break;
	case FixGain2:		name = "FixGain2";		break;
	case ForceSwitchG1:	name = "ForceSwitchG1";		break;
	case ForceSwitchG2:	name = "ForceSwitchG2";		break;
	case VeryLowGain:	name = "VeryLowGain";		break;
	case Undefined:		name = "Undefined";		break;
	case Unitialized:	name = "Unitialized";		break;
	}
	return os << name;
}

ostream& lima::SlsDetector::Defs::operator <<(ostream& os, DACIndex dac_idx)
{
	const char *name = "Unknown";
	switch (dac_idx) {
	case Threshold:        name = "Threshold";        break;
	case CalibPulse:       name = "CalibPulse";       break;
	case TrimBitSize:      name = "TrimBitSize";      break;
	case PreAmp:           name = "PreAmp";           break;
	case Shaper1:          name = "Shaper1";          break;
	case Shaper2:          name = "Shaper2";          break;
	case Humidity:         name = "Humidity";         break;
	case DetectorBias:     name = "DetectorBias";     break;
	case PowerVa:          name = "PowerVa";          break;
	case PowerVdd:         name = "PowerVdd";         break;
	case PowerVsh:         name = "PowerVsh";         break;
	case PowerVio:         name = "PowerVio";         break;
	case PowerHV:          name = "PowerHV";          break;
	case GotthardVrefDS:   name = "GotthardVrefDS";   break;
	case GotthardVcascnPB: name = "GotthardVcascnPB"; break;
	case GotthardVcascpPB: name = "GotthardVcascpPB"; break;
	case GotthardVoutCM:   name = "GotthardVoutCM";   break;
	case GotthardVcascOut: name = "GotthardVcascOut"; break;
	case GotthardVinCM:    name = "GotthardVinCM";    break;
	case GotthardVrefComp: name = "GotthardVrefComp"; break;
	case GotthardIBTestC:  name = "GotthardIBTestC";  break;
	case VoltDAC0:         name = "VoltDAC0";         break;
	case VoltDAC1:         name = "VoltDAC1";         break;
	case VoltDAC2:         name = "VoltDAC2";         break;
	case VoltDAC3:         name = "VoltDAC3";         break;
	case VoltDAC4:         name = "VoltDAC4";         break;
	case VoltDAC5:         name = "VoltDAC5";         break;
	case VoltDAC6:         name = "VoltDAC6";         break;
	case VoltDAC7:         name = "VoltDAC7";         break;
	case EigerSvP:         name = "EigerSvP";         break;
	case EigerSvN:         name = "EigerSvN";         break;
	case EigerVtr:         name = "EigerVtr";         break;
	case EigerVrf:         name = "EigerVrf";         break;
	case EigerVrs:         name = "EigerVrs";         break;
	case EigerVtgstv:      name = "EigerVtgstv";      break;
	case EigerVcmpLL:      name = "EigerVcmpLL";      break;
	case EigerVcmpLR:      name = "EigerVcmpLR";      break;
	case EigerVcal:        name = "EigerVcal";         break;
	case EigerVcmpRL:      name = "EigerVcmpRL";      break;
	case EigerVcmpRR:      name = "EigerVcmpRR";      break;
	case EigerRxbRB:       name = "EigerRxbRB";       break;
	case EigerRxbLB:       name = "EigerRxbLB";       break;
	case EigerVcp:         name = "EigerVcp";         break;
	case EigerVcn:         name = "EigerVcn";         break;
	case EigerVis:         name = "EigerVis";         break;
	case IODelay:          name = "IODelay";          break;
	case ADCVpp:           name = "ADCVpp";           break;
	case HVNew:            name = "HVNew";            break;
	case PowerA:           name = "PowerA";           break;
	case PowerB:           name = "PowerB";           break;
	case PowerC:           name = "PowerC";           break;
	case PowerD:           name = "PowerD";           break;
	case PowerIO:          name = "PowerIO";          break;
	case PowerChip:        name = "PowerChip";        break;
	}
	return os << name;
}

ostream& lima::SlsDetector::Defs::operator <<(ostream& os, ADCIndex adc_idx)
{
	const char *name = "Unknown";
	switch (adc_idx) {
	case TempADC:          name = "TempADC";          break;
	case TempFPGA:         name = "TempFPGA";         break;
	case TempFPGAExt:      name = "TempFPGAExt";      break;
	case Temp10GE:         name = "Temp10GE";         break;
	case TempDCDC:         name = "TempDCDC";         break;
	case TempSODL:         name = "TempSODL";         break;
	case TempSODR:         name = "TempSODR";         break;
	case TempFPGAFL:       name = "TempFPGAFL";       break;
	case TempFPGAFR:       name = "TempFPGAFR";       break;
	}
	return os << name;
}

ostream& lima::SlsDetector::Defs::operator <<(ostream& os, ClockDiv clock_div)
{
	const char *name = "Unknown";
	switch (clock_div) {
	case FullSpeed:		name = "FullSpeed";		break;
	case HalfSpeed:		name = "HalfSpeed";		break;
	case QuarterSpeed:	name = "QuarterSpeed";		break;
	case SuperSlowSpeed:	name = "SuperSlowSpeed";	break;
	}
	return os << name;
}

ostream& lima::SlsDetector::Defs::operator <<(ostream& os, ReadoutFlags flags)
{
#define READOUT_FLAG(x)		{#x, x}

	static struct FlagData {
		const char *name;
		ReadoutFlags flag;
	} ReadoutFlagNamesCList[] = {
		READOUT_FLAG(StoreInRAM),
		READOUT_FLAG(ReadHits),
		READOUT_FLAG(ZeroCompress),
		READOUT_FLAG(PumpProbe),
		READOUT_FLAG(BackgndCorr),
		READOUT_FLAG(TOTMode),
		READOUT_FLAG(Continous),
		READOUT_FLAG(Parallel),
		READOUT_FLAG(NonParallel),
		READOUT_FLAG(Safe),
	};
	const unsigned int size = C_LIST_SIZE(ReadoutFlagNamesCList);

	struct FlagData *data = &ReadoutFlagNamesCList[size - 1];
	bool empty = true;
	for (unsigned int i = 0; i < size; ++i, --data) {
		if (flags & data->flag) {
			if (!empty)
				os << " + ";
			os << data->name;
			empty = false;
		}
	}
	if (empty)
		os << ((flags == Normal) ? "Normal" : "Unknown");
	return os;
}

