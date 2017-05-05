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

enum DACIndex {
	Threshold        = slsDetectorDefs::THRESHOLD,
	CalibPulse       = slsDetectorDefs::CALIBRATION_PULSE,
	TrimBitSize      = slsDetectorDefs::TRIMBIT_SIZE,
	PreAmp           = slsDetectorDefs::PREAMP,
	Shaper1          = slsDetectorDefs::SHAPER1,
	Shaper2          = slsDetectorDefs::SHAPER2,
	Humidity         = slsDetectorDefs::HUMIDITY,
	DetectorBias     = slsDetectorDefs::DETECTOR_BIAS,
	PowerVa          = slsDetectorDefs::VA_POT,
	PowerVdd         = slsDetectorDefs::VDD_POT,
	PowerVsh         = slsDetectorDefs::VSH_POT,
	PowerVio         = slsDetectorDefs::VIO_POT,
	PowerHV          = slsDetectorDefs::HV_POT,
	GotthardVrefDS   = slsDetectorDefs::G_VREF_DS,
	GotthardVcascnPB = slsDetectorDefs::G_VCASCN_PB,
	GotthardVcascpPB = slsDetectorDefs::G_VCASCP_PB,
	GotthardVoutCM   = slsDetectorDefs::G_VOUT_CM,
	GotthardVcascOut = slsDetectorDefs::G_VCASC_OUT,
	GotthardVinCM    = slsDetectorDefs::G_VIN_CM,
	GotthardVrefComp = slsDetectorDefs::G_VREF_COMP,
	GotthardIBTestC  = slsDetectorDefs::G_IB_TESTC,
	VoltDAC0         = slsDetectorDefs::V_DAC0,
	VoltDAC1         = slsDetectorDefs::V_DAC1,
	VoltDAC2         = slsDetectorDefs::V_DAC2,
	VoltDAC3         = slsDetectorDefs::V_DAC3,
	VoltDAC4         = slsDetectorDefs::V_DAC4,
	VoltDAC5         = slsDetectorDefs::V_DAC5,
	VoltDAC6         = slsDetectorDefs::V_DAC6,
	VoltDAC7         = slsDetectorDefs::V_DAC7,
	EigerSvP         = slsDetectorDefs::E_SvP,
	EigerSvN         = slsDetectorDefs::E_SvN,
	EigerVtr         = slsDetectorDefs::E_Vtr,
	EigerVrf         = slsDetectorDefs::E_Vrf,
	EigerVrs         = slsDetectorDefs::E_Vrs,
	EigerVtgstv      = slsDetectorDefs::E_Vtgstv,
	EigerVcmpLL      = slsDetectorDefs::E_Vcmp_ll,
	EigerVcmpLR      = slsDetectorDefs::E_Vcmp_lr,
	EigerVcal        = slsDetectorDefs::E_cal,
	EigerVcmpRL      = slsDetectorDefs::E_Vcmp_rl,
	EigerVcmpRR      = slsDetectorDefs::E_Vcmp_rr,
	EigerRxbRB       = slsDetectorDefs::E_rxb_rb,
	EigerRxbLB       = slsDetectorDefs::E_rxb_lb,
	EigerVcp         = slsDetectorDefs::E_Vcp,
	EigerVcn         = slsDetectorDefs::E_Vcn,
	EigerVis         = slsDetectorDefs::E_Vis,
	IODelay          = slsDetectorDefs::IO_DELAY,
	ADCVpp           = slsDetectorDefs::ADC_VPP,
	HVNew            = slsDetectorDefs::HV_NEW,
	PowerA           = slsDetectorDefs::V_POWER_A,
	PowerB           = slsDetectorDefs::V_POWER_B,
	PowerC           = slsDetectorDefs::V_POWER_C,
	PowerD           = slsDetectorDefs::V_POWER_D,
	PowerIO          = slsDetectorDefs::V_POWER_IO,
	PowerChip        = slsDetectorDefs::V_POWER_CHIP,
};

#define MultiSlsDetectorErr (-100)

enum ADCIndex {
	TempADC          = slsDetectorDefs::TEMPERATURE_ADC,
	TempFPGA         = slsDetectorDefs::TEMPERATURE_FPGA,
	TempFPGAExt      = slsDetectorDefs::TEMPERATURE_FPGAEXT,
	Temp10GE         = slsDetectorDefs::TEMPERATURE_10GE,
	TempDCDC         = slsDetectorDefs::TEMPERATURE_DCDC,
	TempSODL         = slsDetectorDefs::TEMPERATURE_SODL,
	TempSODR         = slsDetectorDefs::TEMPERATURE_SODR,
	TempFPGAFL       = slsDetectorDefs::TEMPERATURE_FPGA2,
	TempFPGAFR       = slsDetectorDefs::TEMPERATURE_FPGA3,
};

enum ClockDiv {
	FullSpeed,
	HalfSpeed,
	QuarterSpeed,
	SuperSlowSpeed, 
};

enum ReadoutFlags {
	Normal       = slsDetectorDefs::NORMAL_READOUT,
	StoreInRAM   = slsDetectorDefs::STORE_IN_RAM,
	ReadHits     = slsDetectorDefs::READ_HITS,
	ZeroCompress = slsDetectorDefs::ZERO_COMPRESSION,
	PumpProbe    = slsDetectorDefs::PUMP_PROBE_MODE,
	BackgndCorr  = slsDetectorDefs::BACKGROUND_CORRECTIONS,
	TOTMode      = slsDetectorDefs::TOT_MODE,
	Continous    = slsDetectorDefs::CONTINOUS_RO,
	Parallel     = slsDetectorDefs::PARALLEL,
	NonParallel  = slsDetectorDefs::NONPARALLEL,
	Safe         = slsDetectorDefs::SAFE,
};

std::ostream& operator <<(std::ostream& os, TrigMode trig_mode);
std::ostream& operator <<(std::ostream& os, Settings settings);
std::ostream& operator <<(std::ostream& os, DACIndex dac_idx);
std::ostream& operator <<(std::ostream& os, ADCIndex adc_idx);
std::ostream& operator <<(std::ostream& os, ClockDiv clock_div);
std::ostream& operator <<(std::ostream& os, ReadoutFlags flags);

} // namespace Defs

} // namespace SlsDetector

} // namespace lima



#endif // __SLS_DETECTOR_DEFS_H
