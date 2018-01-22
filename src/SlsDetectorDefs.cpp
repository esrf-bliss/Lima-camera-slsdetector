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

#include <glob.h>
#include <sys/syscall.h>

using namespace std;
using namespace lima;
using namespace lima::SlsDetector;
using namespace lima::SlsDetector::Defs;

DEB_GLOBAL_NAMESPC(DebModCamera, "SlsDetector");

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

ostream& lima::SlsDetector::Defs::operator <<(ostream& os, DetStatus status)
{
	const char *name = "Unknown";
	switch (status) {
	case Idle:		name = "Idle";			break;
	case Error:		name = "Error";			break;
	case Waiting:		name = "Waiting";		break;
	case RunFinished:	name = "RunFinished";		break;
	case Transmitting:	name = "Transmitting";		break;
	case Running:		name = "Running";		break;
	case Stopped:		name = "Stopped";		break;
	}
	return os << name;
}


Glob::Glob(string pattern)
	: m_pattern(pattern)
{
	DEB_CONSTRUCTOR();
	if (!m_pattern.empty())
		m_found_list = find(m_pattern);
}

Glob& Glob::operator =(std::string pattern)
{
	if (!pattern.empty())
		m_found_list = find(pattern);
	else
		m_found_list.clear();
	m_pattern = pattern;
	return *this;
}

StringList Glob::find(string pattern)
{
	DEB_STATIC_FUNCT();
	DEB_PARAM() << DEB_VAR1(pattern);

	glob_t glob_data;
	int ret = glob(pattern.c_str(), 0, NULL, &glob_data);
	if (ret == GLOB_NOMATCH)
		return StringList();
	else if (ret != 0)
		THROW_HW_ERROR(Error) << "Error in glob: " << ret;

	StringList found;
	try {
		char **begin = glob_data.gl_pathv;
		found.assign(begin, begin + glob_data.gl_pathc);
	} catch (...) {
		globfree(&glob_data);
		throw;
	}
	globfree(&glob_data);
	return found;
}

StringList Glob::split(string path)
{
	DEB_STATIC_FUNCT();
	DEB_PARAM() << DEB_VAR1(path);

	StringList split;
	if (path.empty())
		return split;
	size_t i = 0;
	if (path[i] == '/') {
		split.push_back("/");
		++i;
	}
	size_t end = path.size();
	while (i < end) {
		size_t sep = path.find("/", i);
		split.push_back(path.substr(i, sep - i));
		i = (sep == string::npos) ? end : (sep + 1);
	}
	if (DEB_CHECK_ANY(DebTypeReturn)) {
		ostringstream os;
		os << "[";
		for (unsigned int i = 0; i < split.size(); ++i)
			os << (i ? "," : "") << '"' << split[i] << '"';
		os << "]";
		DEB_RETURN() << "split=" << os.str();
	}
	return split;
}

StringList Glob::getSubPathList(int idx) const
{
	StringList sub_path_list;
	StringList::const_iterator it, end = m_found_list.end();
	for (it = m_found_list.begin(); it != end; ++it)
		sub_path_list.push_back(split(*it).at(idx));
	return sub_path_list;
}

NumericGlob::NumericGlob(string pattern_prefix, string pattern_suffix)
	: m_prefix_len(0), m_suffix_len(0)
{
	DEB_CONSTRUCTOR();
	if (pattern_prefix.empty())
		THROW_HW_ERROR(InvalidValue) << "Empty pattern prefix";

	StringList prefix_dir_list = Glob::split(pattern_prefix);
	m_nb_idx = prefix_dir_list.size();
	if (*pattern_prefix.rbegin() != '/') {
		--m_nb_idx;
		m_prefix_len = prefix_dir_list.back().size();
	}
	if (!pattern_suffix.empty() && (pattern_suffix[0] != '/'))
		m_suffix_len = Glob::split(pattern_suffix)[0].size();
	DEB_TRACE() << DEB_VAR3(m_nb_idx, m_prefix_len, m_suffix_len);
	m_glob = pattern_prefix + "[0-9]*" + pattern_suffix;
}

NumericGlob::IntStringList NumericGlob::getIntPathList() const
{
	DEB_MEMBER_FUNCT();
	IntStringList list;
	StringList path_list = m_glob.getPathList();
	StringList::const_iterator pit = path_list.begin();
	StringList sub_path_list = m_glob.getSubPathList(m_nb_idx);
	StringList::const_iterator it, end = sub_path_list.end();
	for (it = sub_path_list.begin(); it != end; ++it, ++pit) {
		size_t l = (*it).size() - m_prefix_len - m_suffix_len;
		string s = (*it).substr(m_prefix_len, l);
		int nb;
		istringstream(s) >> nb;
		DEB_TRACE() << DEB_VAR2(nb, *pit);
		list.push_back(IntString(nb, *pit));
	}
	sort(list.begin(), list.end());
	return list;
}

SimpleStat::SimpleStat(double f)
	: factor(f)
{
	reset();
}

void SimpleStat::reset()
{
	AutoMutex l(lock);
	xmin = xmax = xacc = xacc2 = 0;
	xn = 0;
}

void SimpleStat::add(double x) {
	AutoMutex l(lock);
	x *= factor;
	xmin = xn ? std::min(xmin, x) : x;
	xmax = xn ? std::max(xmax, x) : x;
	xacc += x;
	xacc2 += pow(x, 2);
	++xn;
}

SimpleStat& SimpleStat::operator =(const SimpleStat& o)
{
	if (&o == this)
		return *this;

	AutoMutex l(o.lock);
	xmin = o.xmin;
	xmax = o.xmax;
	xacc = o.xacc;
	xacc2 = o.xacc2;
	xn = o.xn;
	factor = o.factor;
	return *this;
}

int SimpleStat::n() const
{ 
	AutoMutex l(lock);
	return xn; 
}

double SimpleStat::min() const
{ 
	AutoMutex l(lock);
	return xmin;
}

double SimpleStat::max() const
{
	AutoMutex l(lock);
	return xmax; 
}

double SimpleStat::ave() const
{ 
	AutoMutex l(lock);
	return xn ? (xacc / xn) : 0; 
}

double SimpleStat::std() const
{ 
	AutoMutex l(lock);
	return xn ? sqrt(xacc2 / xn - pow(ave(), 2)) : 0; 
}

FrameType lima::SlsDetector::getLatestFrame(const FrameArray& l)
{
	DEB_STATIC_FUNCT();

	FrameType last_frame = !l.empty() ? l[0] : -1;
	for (unsigned int i = 1; i < l.size(); ++i)
		updateLatestFrame(last_frame, l[i]);

	DEB_RETURN() << DEB_VAR1(last_frame);
	return last_frame;
}

FrameType lima::SlsDetector::getOldestFrame(const FrameArray& l)
{
	DEB_STATIC_FUNCT();

	FrameType first_frame = !l.empty() ? l[0] : -1;
	for (unsigned int i = 1; i < l.size(); ++i)
		updateOldestFrame(first_frame, l[i]);

	DEB_RETURN() << DEB_VAR1(first_frame);
	return first_frame;
}

ostream& lima::SlsDetector::operator <<(ostream& os, State state)
{
	const char *name = "Unknown";
	switch (state) {
	case Idle:	name = "Idle";		break;
	case Init:	name = "Init";		break;
	case Starting:	name = "Starting";	break;
	case Running:	name = "Running";	break;
	case StopReq:	name = "StopReq";	break;
	case Stopping:	name = "Stopping";	break;
	case Stopped:	name = "Stopped";	break;
	}
	return os << name;
}

ostream& lima::SlsDetector::operator <<(ostream& os, Type type)
{
	const char *name = "Invalid";
	switch (type) {
	case UnknownDet:	name = "Unknown";	break;
	case GenericDet:	name = "Generic";	break;
	case EigerDet:		name = "Eiger";		break;
	case JungfrauDet:	name = "Jungfrau";	break;
	}
	return os << name;
}

ostream& lima::SlsDetector::operator <<(ostream& os, const SortedIntList& l)
{
	return os << PrettySortedList(l);
}

ostream& lima::SlsDetector::operator <<(ostream& os, const FrameArray& a)
{
	os << "[";
	for (unsigned int i = 0; i < a.size(); ++i)
		os << (i ? ", " : "") << a[i];
	return os << "]";
}

ostream& lima::SlsDetector::operator <<(ostream& os, const SimpleStat& s)
{
	os << "<";
	os << "min=" << int(s.min()) << ", max=" << int(s.max()) << ", "
	   << "ave=" << int(s.ave()) << ", std=" << int(s.std()) << ", "
	   << "n=" << s.n();
	return os << ">";
}

TimeRangesChangedCallback::TimeRangesChangedCallback()
	: m_cam(NULL)
{
	DEB_CONSTRUCTOR();
}

TimeRangesChangedCallback::~TimeRangesChangedCallback()
{
	DEB_DESTRUCTOR();

	if (m_cam)
		m_cam->unregisterTimeRangesChangedCallback(*this);
}

FrameMap::FrameMap()
	: m_nb_items(0), m_buffer_size(0)
{
	DEB_CONSTRUCTOR();
}

FrameMap::~FrameMap()
{
	DEB_DESTRUCTOR();
}

void FrameMap::setNbItems(int nb_items)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR1(nb_items);
	if (nb_items == m_nb_items)
		return;

	m_last_item_frame.resize(nb_items);
	m_nb_items = nb_items;
}

void FrameMap::setBufferSize(int buffer_size)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR1(buffer_size);
	if (buffer_size == m_buffer_size)
		return;

	m_frame_item_count.resize(buffer_size);
	m_buffer_size = buffer_size;
}

void FrameMap::clear()
{
	DEB_MEMBER_FUNCT();
	for (int i = 0; i < m_nb_items; ++i)
		m_last_item_frame[i] = -1;
	for (int i = 0; i < m_buffer_size; ++i)
		m_frame_item_count[i].set(m_nb_items);
}

void FrameMap::checkFinishedFrameItem(FrameType frame, int item)
{
	DEB_MEMBER_FUNCT();

	if (m_nb_items == 0)		
		THROW_HW_ERROR(InvalidValue) << "No items defined";
	else if (m_buffer_size == 0)
		THROW_HW_ERROR(InvalidValue) << "No buffer size defined";
	else if ((item < 0) || (item >= m_nb_items))
		THROW_HW_ERROR(InvalidValue) << DEB_VAR2(item, m_nb_items);

	FrameType &last = m_last_item_frame[item];
	if (isValidFrame(last) && (frame <= last))
		THROW_HW_ERROR(Error) << DEB_VAR1(frame) << " finished already";
}

FrameMap::FinishInfo FrameMap::frameItemFinished(FrameType frame, int item, 
						 bool no_check, bool valid)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR4(frame, item, no_check, m_nb_items);

	if (!no_check)
		checkFinishedFrameItem(frame, item);

	FinishInfo finfo;
	FrameType &last = m_last_item_frame[item];
	finfo.first_lost = last + 1;
	finfo.nb_lost = frame - finfo.first_lost + (!valid ? 1 : 0);
	for (FrameType f = last + 1; f != (frame + 1); ++f) {
		int idx = f % m_buffer_size;
		AtomicCounter& count = m_frame_item_count[idx];
		bool frame_finished = count.dec_test_and_reset(m_nb_items);
		if (!frame_finished)
			continue;
		finfo.finished.insert(f);
	}
	last = frame;

	if (DEB_CHECK_ANY(DebTypeReturn))
		DEB_RETURN() << DEB_VAR3(finfo.first_lost, finfo.nb_lost,
					 PrettySortedList(finfo.finished));
	return finfo;
}

ostream& lima::SlsDetector::operator <<(ostream& os, const FrameMap& m)
{
	os << "<";
	os << "LastFinishedFrame=" << m.getLastFinishedFrame() << ", "
	   << "LastItemFrame=" << m.getLastItemFrame() << ", "
	   << "ItemFrameArray=" << m.getItemFrameArray();
	return os << ">";
}

Stats::Stats()
	: cb_period(1e6), new_finish(1e6), cb_exec(1e6), recv_exec(1e6)
{}

void Stats::reset()
{
	cb_period.reset();
	new_finish.reset();
	cb_exec.reset();
	recv_exec.reset();
}

ostream& lima::SlsDetector::operator <<(ostream& os, const Stats& s)
{
	os << "<";
	os << "cb_period=" << s.cb_period << ", "
	   << "new_finish=" << s.new_finish << ", "
	   << "cb_exec=" << s.cb_exec << ", "
	   << "recv_exec=" << s.recv_exec;
	return os << ">";
}


pid_t lima::SlsDetector::gettid() {
	return syscall(SYS_gettid);
}

