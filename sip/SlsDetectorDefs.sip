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

enum DACIndex {
	Threshold        = THRESHOLD,
	CalibPulse       = CALIBRATION_PULSE,
	TrimBitSize      = TRIMBIT_SIZE,
	PreAmp           = PREAMP,
	Shaper1          = SHAPER1,
	Shaper2          = SHAPER2,
	Humidity         = HUMIDITY,
	DetectorBias     = DETECTOR_BIAS,
	PowerVa          = VA_POT,
	PowerVdd         = VDD_POT,
	PowerVsh         = VSH_POT,
	PowerVio         = VIO_POT,
	PowerHV          = HV_POT,
	GotthardVrefDS   = G_VREF_DS,
	GotthardVcascnPB = G_VCASCN_PB,
	GotthardVcascpPB = G_VCASCP_PB,
	GotthardVoutCM   = G_VOUT_CM,
	GotthardVcascOut = G_VCASC_OUT,
	GotthardVinCM    = G_VIN_CM,
	GotthardVrefComp = G_VREF_COMP,
	GotthardIBTestC  = G_IB_TESTC,
	VoltDAC0         = V_DAC0,
	VoltDAC1         = V_DAC1,
	VoltDAC2         = V_DAC2,
	VoltDAC3         = V_DAC3,
	VoltDAC4         = V_DAC4,
	VoltDAC5         = V_DAC5,
	VoltDAC6         = V_DAC6,
	VoltDAC7         = V_DAC7,
	EigerSvP         = E_SvP,
	EigerSvN         = E_SvN,
	EigerVtr         = E_Vtr,
	EigerVrf         = E_Vrf,
	EigerVrs         = E_Vrs,
	EigerVtgstv      = E_Vtgstv,
	EigerVcmpLL      = E_Vcmp_ll,
	EigerVcmpLR      = E_Vcmp_lr,
	EigerVcal        = E_cal,
	EigerVcmpRL      = E_Vcmp_rl,
	EigerVcmpRR      = E_Vcmp_rr,
	EigerRxbRB       = E_rxb_rb,
	EigerRxbLB       = E_rxb_lb,
	EigerVcp         = E_Vcp,
	EigerVcn         = E_Vcn,
	EigerVis         = E_Vis,
	IODelay          = IO_DELAY,
	ADCVpp           = ADC_VPP,
	HVNew            = HV_NEW,
	PowerA           = V_POWER_A,
	PowerB           = V_POWER_B,
	PowerC           = V_POWER_C,
	PowerD           = V_POWER_D,
	PowerIO          = V_POWER_IO,
	PowerChip        = V_POWER_CHIP,
};

enum ADCIndex {
	TempADC          = TEMPERATURE_ADC,
	TempFPGA         = TEMPERATURE_FPGA,
	TempFPGAExt      = TEMPERATURE_FPGAEXT,
	Temp10GE         = TEMPERATURE_10GE,
	TempDCDC         = TEMPERATURE_DCDC,
	TempSODL         = TEMPERATURE_SODL,
	TempSODR         = TEMPERATURE_SODR,
	TempFPGAFL       = TEMPERATURE_FPGA2,
	TempFPGAFR       = TEMPERATURE_FPGA3,
};

enum ClockDiv {
	FullSpeed,
	HalfSpeed,
	QuarterSpeed,
	SuperSlowSpeed, 
};

enum ReadoutFlags {
	Normal       = NORMAL_READOUT,
	StoreInRAM   = STORE_IN_RAM,
	ReadHits     = READ_HITS,
	ZeroCompress = ZERO_COMPRESSION,
	PumpProbe    = PUMP_PROBE_MODE,
	BackgndCorr  = BACKGROUND_CORRECTIONS,
	TOTMode      = TOT_MODE,
	Continous    = CONTINOUS_RO,
	Parallel     = PARALLEL,
	NonParallel  = NONPARALLEL,
	Safe         = SAFE,
};

enum DetStatus {
	Idle         = IDLE,
	Error        = ERROR,
	Waiting      = WAITING,
	RunFinished  = RUN_FINISHED,
	Transmitting = TRANSMITTING,
	Running      = RUNNING,
	Stopped      = STOPPED,
};

}; // namespace Defs


enum State {
	Idle, Init, Starting, Running, StopReq, Stopping, Stopped,
};

enum Type {
	GenericDet, EigerDet, JungfrauDet,
};

enum PixelDepth {
	PixelDepth4 = 4, 
	PixelDepth8 = 8, 
	PixelDepth16 = 16, 
	PixelDepth32 = 32,
};

struct TimeRanges {
	TimeRanges();
	double min_exp_time;
	double max_exp_time;
	double min_lat_time;
	double max_lat_time;
	double min_frame_period;
	double max_frame_period;
};


bool isValidFrame(unsigned long frame);
unsigned long latestFrame(unsigned long a, unsigned long b);
unsigned long oldestFrame(unsigned long a, unsigned long b);


struct SimpleStat {
	double xmin;
	double xmax;
	double xacc;
	double xacc2;
	int xn;
	double factor;
	Mutex lock;

//	std::map<int, int> hist;
//	int hist_bin;

	SimpleStat(double f = 1, int b = 5);
	void reset();
	void add(double x, bool do_hist=1);
	//SimpleStat& operator =(const SimpleStat& o);

	int n() const;
	double min() const;
	double max() const;
	double ave() const;
	double std() const;
};


class FrameMap
{
public:
	typedef std::set<int> List;
	typedef std::map<unsigned long, std::set<int> > Map;

	~FrameMap();

	void setNbItems(int nb_items);
	void setBufferSize(int buffer_size);
	void clear();

	unsigned long getLastItemFrame() const;
	unsigned long getLastFinishedFrame() const;

private:
	FrameMap();
};


struct Stats {
	SlsDetector::SimpleStat cb_period;
	SlsDetector::SimpleStat new_finish;
	SlsDetector::SimpleStat cb_exec;
	SlsDetector::SimpleStat recv_exec;
	Stats();
	void reset();
};


}; // namespace SlsDetector
