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

#include "sls/sls_detector_defs.h"

#include "lima/Debug.h"
#include "lima/Exceptions.h"
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
	Gated           = slsDetectorDefs::GATED,
	BurstTrigger    = slsDetectorDefs::BURST_TRIGGER,
	TriggeredGated  = slsDetectorDefs::TRIGGER_GATED,
	SoftTriggerExposure,
};

enum Settings {
	Standard      = slsDetectorDefs::STANDARD,
	Fast          = slsDetectorDefs::FAST,
	HighGain      = slsDetectorDefs::HIGHGAIN,
	DynamicGain   = slsDetectorDefs::DYNAMICGAIN,
	LowGain       = slsDetectorDefs::LOWGAIN,
	MediumGain    = slsDetectorDefs::MEDIUMGAIN,
	VeryHighGain  = slsDetectorDefs::VERYHIGHGAIN,
	FixGain1      = slsDetectorDefs::FIXGAIN1,
	FixGain2      = slsDetectorDefs::FIXGAIN2,
	VeryLowGain   = slsDetectorDefs::VERYLOWGAIN,
	Undefined     = slsDetectorDefs::UNDEFINED,
	Unitialized   = slsDetectorDefs::UNINITIALIZED,
};

enum GainMode {
	Dynamic       = slsDetectorDefs::DYNAMIC,
        ForceSwitchG1 = slsDetectorDefs::FORCE_SWITCH_G1,
        ForceSwitchG2 = slsDetectorDefs::FORCE_SWITCH_G2,
        FixG1         = slsDetectorDefs::FIX_G1,
        FixG2         = slsDetectorDefs::FIX_G2,
        FixG0         = slsDetectorDefs::FIX_G0,
};

#define SlsDetectorBadIndexErr	(-9999)

enum DACIndex {
	EigerVcmpLL = slsDetectorDefs::VCMP_LL,
	EigerVcmpLR = slsDetectorDefs::VCMP_LR,
	EigerVcmpRL = slsDetectorDefs::VCMP_RL,
	EigerVcmpRR = slsDetectorDefs::VCMP_RR,
	Threshold   = slsDetectorDefs::VTHRESHOLD,
};

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
	FullSpeed = slsDetectorDefs::FULL_SPEED,
	HalfSpeed = slsDetectorDefs::HALF_SPEED,
	QuarterSpeed = slsDetectorDefs::QUARTER_SPEED,
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

typedef slsDetectorDefs::xy xy;

std::ostream& operator <<(std::ostream& os, TrigMode trig_mode);
std::ostream& operator <<(std::ostream& os, Settings settings);
std::ostream& operator <<(std::ostream& os, DACIndex dac_idx);
std::ostream& operator <<(std::ostream& os, ADCIndex adc_idx);
std::ostream& operator <<(std::ostream& os, ClockDiv clock_div);
std::ostream& operator <<(std::ostream& os, DetStatus status);

} // namespace Defs

typedef std::vector<int> Positions;

inline Positions Idx2Pos(int idx)
{
	if (idx != -1)
		return {idx};
	else
		return {};
}

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
		int prev = -1;
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


enum AcqState {
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

std::ostream& operator <<(std::ostream& os, AcqState state);
std::ostream& operator <<(std::ostream& os, Type type);
std::ostream& operator <<(std::ostream& os, PixelDepth pixel_depth);
std::istream& operator >>(std::istream& is, PixelDepth& pixel_depth);

typedef uint64_t FrameType;
typedef std::vector<std::string> StringList;
typedef StringList NameList;
typedef std::vector<int> IntList;
typedef std::vector<double> FloatList;
typedef std::set<int> SortedIntList;
typedef std::vector<FrameType> FrameArray;
typedef IntList ProcList;

std::ostream& operator <<(std::ostream& os, const StringList& l);
std::ostream& operator <<(std::ostream& os, const SortedIntList& l);
std::ostream& operator <<(std::ostream& os, const FrameArray& a);

typedef PrettyList<IntList> PrettyIntList;
typedef PrettyList<SortedIntList> PrettySortedList;

StringList SplitString(const std::string& s, const std::string& sep = ",");

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
	 IntStringList getIntSubPathList(int idx) const;

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
 
struct XYStat {
	double xacc, xacc2, yacc, xyacc;
	int xn;
	double factor;
	mutable Mutex lock;

	XYStat(double f = 1);
	void reset();
	void add(double x, double y);
	XYStat& operator =(const XYStat& o);
	XYStat& operator += (const XYStat& o);

	// Linear Regression
	struct LinRegress {
		int n;
		double slope;
		double offset;

		LinRegress() : n(0), slope(0), offset(0)
		{}
	};

	int n() const;
	LinRegress calcLinRegress() const;
};

std::ostream& operator <<(std::ostream& os, const SimpleStat::Histogram& s);
std::ostream& operator <<(std::ostream& os, const SimpleStat& s);
std::ostream& operator <<(std::ostream& os, const XYStat::LinRegress& r);


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


inline void CheckLockedAutoMutex(Cond& cond, AutoMutex& l, DebObj *deb_ptr)
{
	DEB_FROM_PTR(deb_ptr);
	if (&l.mutex() != &cond.mutex())
		THROW_HW_ERROR(Error) << "Bad AutoMutex";
	else if (!l.locked())
		THROW_HW_ERROR(Error) << "Mutex not locked";
}

template <typename T>
class StateCleanUpHelper
{
public:
  StateCleanUpHelper(T& state, T final_state, Cond& cond, AutoMutex& l,
		     DebObj *deb_ptr)
	  : m_state(state), m_final_state(final_state), m_cond(cond), m_lock(l),
	    m_deb_ptr(deb_ptr)
	{
		DEB_FROM_PTR(m_deb_ptr);
		CheckLockedAutoMutex(m_cond, m_lock, DEB_PTR());
		DEB_TRACE() << DEB_VAR1(m_state);
	}

	~StateCleanUpHelper()
	{
		DEB_FROM_PTR(m_deb_ptr);
		DEB_TRACE() << DEB_VAR1(m_state);
		m_state = m_final_state;
		DEB_TRACE() << DEB_VAR1(m_state);
		m_cond.broadcast();
	}

protected:
	T& m_state;
	T m_final_state;
	Cond& m_cond;
	AutoMutex& m_lock;
	DebObj *m_deb_ptr;
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

#define EXC_CHECK(x)						\
	try {							\
		x;						\
	} catch (std::exception& e) {				\
		THROW_HW_ERROR(Error) << e.what();		\
	}

} // namespace SlsDetector

} // namespace lima



#endif // __SLS_DETECTOR_DEFS_H
