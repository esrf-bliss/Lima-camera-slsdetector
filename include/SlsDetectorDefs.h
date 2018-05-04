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

#include "lima/Debug.h"
#include "lima/RegExUtils.h"
#include "lima/Timestamp.h"

#include <set>

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

#define MultiSlsDetectorErr	(-100)
#define SlsDetectorBadIndexErr	(-9999)

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

typedef std::map<DACIndex, std::string> DACCmdMapType;
extern DACCmdMapType DACCmdMap;

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

typedef std::map<ADCIndex, std::string> ADCCmdMapType;
extern ADCCmdMapType ADCCmdMap;

enum ClockDiv {
	FullSpeed,
	HalfSpeed,
	QuarterSpeed,
	SuperSlowSpeed, 
};

enum DetStatus {
	Idle         = slsDetectorDefs::IDLE,
	Error        = slsDetectorDefs::ERROR,
	Waiting      = slsDetectorDefs::WAITING,
	RunFinished  = slsDetectorDefs::RUN_FINISHED,
	Transmitting = slsDetectorDefs::TRANSMITTING,
	Running      = slsDetectorDefs::RUNNING,
	Stopped      = slsDetectorDefs::STOPPED,
};

enum NetworkParameter {
	DetectorMAC     = slsDetectorDefs::DETECTOR_MAC,
	DetectorIP      = slsDetectorDefs::DETECTOR_IP,
	RecvHostName    = slsDetectorDefs::RECEIVER_HOSTNAME,
	RecvUDPIP       = slsDetectorDefs::RECEIVER_UDP_IP,
	RecvUDPPort     = slsDetectorDefs::RECEIVER_UDP_PORT,
	RecvUDPMAC      = slsDetectorDefs::RECEIVER_UDP_MAC,
	RecvUDPPort2    = slsDetectorDefs::RECEIVER_UDP_PORT2,
	DetTxDelayLeft  = slsDetectorDefs::DETECTOR_TXN_DELAY_LEFT,
	DetTxDelayRight = slsDetectorDefs::DETECTOR_TXN_DELAY_RIGHT,
	DetTxDelayFrame = slsDetectorDefs::DETECTOR_TXN_DELAY_FRAME,
	FlowCtrl10G     = slsDetectorDefs::FLOW_CONTROL_10G,
	FlowCtrlWrPtr   = slsDetectorDefs::FLOW_CONTROL_WR_PTR,
	FlowCtrlRdPtr   = slsDetectorDefs::FLOW_CONTROL_RD_PTR,
};

std::ostream& operator <<(std::ostream& os, TrigMode trig_mode);
std::ostream& operator <<(std::ostream& os, Settings settings);
std::ostream& operator <<(std::ostream& os, DACIndex dac_idx);
std::ostream& operator <<(std::ostream& os, ADCIndex adc_idx);
std::ostream& operator <<(std::ostream& os, ClockDiv clock_div);
std::ostream& operator <<(std::ostream& os, DetStatus status);
std::ostream& operator <<(std::ostream& os, NetworkParameter net_param);

} // namespace Defs


template <class T>
class PrettyList
{
 public:
	typedef typename T::const_iterator const_iterator;

	PrettyList(const T& l) : begin(l.begin()), end(l.end()) {}
	PrettyList(const_iterator b, const_iterator e) : begin(b), end(e) {}

	std::ostream& print(std::ostream& os) const
	{
		os << "[";
		int prev;
		bool in_seq = false;
		bool first = true;
		for (const_iterator it = begin; it != end; ++it) {
			int val = *it;
			bool seq = (!first && (val == prev + 1));
			if (!seq) {
				if (in_seq)
					os << "-" << prev;
				os << (first ? "" : ",") << val;
			}
			prev = val;
			in_seq = seq;
			first = false;
		}
		if (in_seq)
			os << "-" << prev;
		return os << "]";
	}

 private:
	const_iterator begin, end;
};

template <class T>
std::ostream& operator <<(std::ostream& os, const PrettyList<T>& pl)
{
	return pl.print(os);
}


enum State {
	Idle, Init, Starting, Running, StopReq, Stopping, Stopped,
};

enum Type {
	UnknownDet, GenericDet, EigerDet, JungfrauDet,
};
 
enum PixelDepth {
	PixelDepth4 = 4, 
	PixelDepth8 = 8, 
	PixelDepth16 = 16, 
	PixelDepth32 = 32,
};

std::ostream& operator <<(std::ostream& os, State state);
std::ostream& operator <<(std::ostream& os, Type type);


typedef uint64_t FrameType;
typedef std::vector<std::string> StringList;
typedef StringList NameList;
typedef std::vector<int> IntList;
typedef std::vector<double> FloatList;
typedef std::set<int> SortedIntList;
typedef std::vector<FrameType> FrameArray;
typedef IntList ProcList;

std::ostream& operator <<(std::ostream& os, const SortedIntList& l);
std::ostream& operator <<(std::ostream& os, const FrameArray& a);

typedef PrettyList<IntList> PrettyIntList;
typedef PrettyList<SortedIntList> PrettySortedList;


struct TimeRanges {
	TimeRanges() :
		min_exp_time(-1.), 
		max_exp_time(-1.),
		min_lat_time(-1.),
		max_lat_time(-1.),
		min_frame_period(-1.),
		max_frame_period(-1.)
	{}

	double min_exp_time;
	double max_exp_time;
	double min_lat_time;
	double max_lat_time;
	double min_frame_period;
	double max_frame_period;
};


class Glob
{
	DEB_CLASS_NAMESPC(DebModCamera, "Glob", "SlsDetector");
 public:
	Glob(std::string pattern = "");
	Glob& operator =(std::string pattern);

	static StringList find(std::string pattern);
	static StringList split(std::string path);

	int getNbEntries() const
	{ return m_found_list.size(); }

	StringList getPathList() const
	{ return m_found_list; }

	StringList getSubPathList(int idx) const;

 private:
	std::string m_pattern;
	StringList m_found_list;
};

class NumericGlob
{
	 DEB_CLASS_NAMESPC(DebModCamera, "NumericGlob", "SlsDetector");
 public:
	 typedef std::pair<int, std::string> IntString;
	 typedef std::vector<IntString> IntStringList;

	 NumericGlob(std::string pattern_prefix, 
		     std::string pattern_suffix = "");

	 int getNbEntries() const
	 { return m_glob.getNbEntries(); }

	 IntStringList getIntPathList() const;

 private:
	 int m_nb_idx;
	 int m_prefix_len;
	 int m_suffix_len;
	 Glob m_glob;
};

inline int operator <(const NumericGlob::IntString& a,
		      const NumericGlob::IntString& b)
{
	return (a.first < b.first);
}


inline bool isValidFrame(FrameType frame)
{ return (frame != FrameType(-1)); }

inline FrameType latestFrame(FrameType a, FrameType b)
{
	 if (!isValidFrame(a))
		 return b;
	 if (!isValidFrame(b))
		 return a;
	 return std::max(a, b);
}

inline FrameType oldestFrame(FrameType a, FrameType b)
{
	 if (!isValidFrame(a))
		 return a;
	 if (!isValidFrame(b))
		 return b;
	 return std::min(a, b);
}

inline FrameType updateLatestFrame(FrameType& a, FrameType b)
{
	 a = latestFrame(a, b);
	 return a;
}

inline FrameType updateOldestFrame(FrameType& a, FrameType b)
{
	 a = oldestFrame(a, b);
	 return a;
}

FrameType getLatestFrame(const FrameArray& l);
FrameType getOldestFrame(const FrameArray& l);


struct SimpleStat {
	typedef std::map<int, int> Histogram;

	static void setDoHist(bool do_hist);
	static bool getDoHist();

	double xmin, xmax, xacc, xacc2;
	int xn;
	double factor;
	mutable Mutex lock;
	Histogram hist;
	int hist_bin;

	SimpleStat(double f = 1, int b = 5);
	void reset();
	void add(double x, bool do_hist = false);
	SimpleStat& operator =(const SimpleStat& o);
	SimpleStat& operator += (const SimpleStat& o);

	int n() const;
	double min() const;
	double max() const;
	double ave() const;
	double std() const;

private:
	static bool DoHist;
};
 
std::ostream& operator <<(std::ostream& os, const SimpleStat::Histogram& s);
std::ostream& operator <<(std::ostream& os, const SimpleStat& s);


class Camera;

class TimeRangesChangedCallback {
	DEB_CLASS_NAMESPC(DebModCamera, "TimeRangesChangedCallback", 
			  "SlsDetector");
 public:
	TimeRangesChangedCallback();
	virtual ~TimeRangesChangedCallback();

 protected:
	virtual void timeRangesChanged(TimeRanges time_ranges) = 0;

 private:
	friend class Camera;
	Camera *m_cam;
};


class FrameMap
{
	DEB_CLASS_NAMESPC(DebModCamera, "FrameMap", "SlsDetector");

 public:
	class Item
	{
		DEB_CLASS_NAMESPC(DebModCamera, "Item", "SlsDetector");
	
	public:
		struct FinishInfo {
			FrameType first_lost;
			int nb_lost;
			SortedIntList finished;
		};
		typedef std::vector<FinishInfo> FinishInfoList;
	
		Item();
		~Item();
	
		void checkFinishedFrame(FrameType frame);
		void frameFinished(FrameType frame, bool no_check, bool valid);
		FinishInfoList pollFrameFinished();
		void stopPollFrameFinished();
	
	private:
		friend class FrameMap;
	
		typedef std::pair<int, bool> FrameData;
		typedef std::vector<FrameData> FrameDataList;
	
		class FrameQueue 
		{
		public:
			FrameQueue(int size = 1000);
			void clear();
			void push(FrameData data);
			FrameDataList pop_all();
			void stop();
	
		private:
			int index(int i)
			{ return i % m_size; }
	
			FrameDataList m_array;
			int m_size;
			volatile int m_write_idx;
			volatile int m_read_idx;
			volatile bool m_stopped;
		};
	
		friend bool SlsDetector::operator <(FrameData a, FrameData b);
	
		void setFrameMap(FrameMap *map);
		void clear();
	
		FrameMap *m_map;
		FrameQueue m_frame_queue;
		FrameType m_last_pushed_frame;
		FrameType m_last_frame;
	};
	typedef std::vector<Item> ItemList;

	
	FrameMap();
		
	void setNbItems(int nb_items);
	void setBufferSize(int buffer_size);
	void clear();

	Item& getItem(int item)
	{ return m_item_list[item]; }

	FrameArray getItemFrameArray() const;

	FrameType getLastItemFrame() const
	{ return getLatestFrame(getItemFrameArray()); }

	FrameType getLastFinishedFrame() const
	{ return getOldestFrame(getItemFrameArray()); }

 private:
	friend class Item;

	struct AtomicCounter {
		int count;
		Mutex mutex;
	
		void set(int reset)
		{ count = reset; }
	
		bool dec_test_and_reset(int reset)
		{
			mutex.lock();
			bool zero = (--count == 0);
			if (zero)
				set(reset);
			mutex.unlock();
			return zero;
		}
	};
	typedef std::vector<AtomicCounter> CounterList;
	
	int m_nb_items;
	int m_buffer_size;
	CounterList m_frame_item_count_list;
	ItemList m_item_list;
};

std::ostream& operator <<(std::ostream& os, const FrameMap& m);


class SeqFilter {
 public:
	struct Range {
		int first;
		int nb;

		Range()
		{
			reset();
		}

		void reset(int new_first = 0)
		{
			first = new_first;
			nb = 0;
		}

		int end()
		{
			return first + nb;
		}

		bool tryExtend(int val)
		{
			bool ok = (val == end());
			if (ok)
				nb++;
			return ok;
		}
	};

	void addVal(int new_val)
	{
		SortedIntList& w = m_waiting;
		if (!m_seq_range.tryExtend(new_val)) {
			w.insert(new_val);
		} else {
			while (!w.empty()) {
				SortedIntList::iterator wit = w.begin();
				if (!m_seq_range.tryExtend(*wit))
					break;
				w.erase(wit);
			}
		}
	}

	Range getSeqRange() 
	{
		Range ret = m_seq_range;
		m_seq_range.reset(m_seq_range.end());
		return ret;
	}

 private:
	Range m_seq_range;
	SortedIntList m_waiting;
};

struct Stats {
	SimpleStat cb_period;
	SimpleStat new_finish;
	SimpleStat cb_exec;
	SimpleStat recv_exec;
	Stats();
	void reset();
	Stats& operator +=(const Stats& o);
};

std::ostream& operator <<(std::ostream& os, const Stats& s);


typedef RegEx::SingleMatchType SingleMatch;
typedef RegEx::FullMatchType FullMatch;
typedef RegEx::MatchListType MatchList;
typedef MatchList::const_iterator MatchListIt;


} // namespace SlsDetector

} // namespace lima



#endif // __SLS_DETECTOR_DEFS_H
