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
#include <cmath>

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
	case Gated:		name = "Gated";			break;
	case BurstTrigger:	name = "BurstTrigger";		break;
	case TriggeredGated:	name = "TriggeredGated";	break;
	case SoftTriggerExposure: name = "SoftTriggerExposure";	break;
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
	case FixGain1:		name = "FixGain1";		break;
	case FixGain2:		name = "FixGain2";		break;
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
	case EigerVcmpLL:      name = "EigerVcmpLL";      break;
	case EigerVcmpLR:      name = "EigerVcmpLR";      break;
	case EigerVcmpRL:      name = "EigerVcmpRL";      break;
	case EigerVcmpRR:      name = "EigerVcmpRR";      break;
	case Threshold:        name = "Threshold";        break;
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
	}
	return os << name;
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

StringList lima::SlsDetector::SplitString(const std::string& s,
					  const std::string& sep)
{
	StringList list;
	string::size_type i, p, n;
	for (i = 0; (i != string::npos) && (i != s.size()); i = p) {
		p = s.find(sep, i);
		n = (p == string::npos) ? p : (p - i);
		list.push_back(string(s, i, n));
		if (p != string::npos)
			++p;
	}
	return list;
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
	if (idx == -1)
		return getPathList();

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
	return getIntSubPathList(-1);
}

NumericGlob::IntStringList NumericGlob::getIntSubPathList(int idx) const
{
	DEB_MEMBER_FUNCT();
	IntStringList list;
	StringList ret_path_list = m_glob.getSubPathList(idx);
	StringList::const_iterator rit = ret_path_list.begin();
	StringList sub_path_list = m_glob.getSubPathList(m_nb_idx);
	StringList::const_iterator it, end = sub_path_list.end();
	for (it = sub_path_list.begin(); it != end; ++it, ++rit) {
		size_t l = (*it).size() - m_prefix_len - m_suffix_len;
		string s = (*it).substr(m_prefix_len, l);
		int nb;
		istringstream(s) >> nb;
		DEB_TRACE() << DEB_VAR2(nb, *rit);
		list.push_back(IntString(nb, *rit));
	}
	sort(list.begin(), list.end());
	return list;
}

bool SimpleStat::DoHist = false;

void SimpleStat::setDoHist(bool do_hist)
{
	DoHist = do_hist;
}

bool SimpleStat::getDoHist()
{
	return DoHist;
}

SimpleStat::SimpleStat(double f, int b)
	: factor(f), hist_bin(b)
{
	reset();
}

void SimpleStat::reset()
{
	AutoMutex l(lock);
	xmin = xmax = xacc = xacc2 = 0;
	xn = 0;
	hist.clear();
}

void SimpleStat::add(double x, bool do_hist) {
	AutoMutex l(lock);
	x *= factor;
	xmin = xn ? std::min(xmin, x) : x;
	xmax = xn ? std::max(xmax, x) : x;
	xacc += x;
	xacc2 += pow(x, 2);
	++xn;

	if (!(do_hist || DoHist))
		return;

	int i = x;
	i -= i % hist_bin;
	Histogram::iterator it = hist.find(i);
	if (it == hist.end())
		hist[i] = 1;
	else
		it->second++;
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
	hist = o.hist;
	hist_bin = o.hist_bin;
	return *this;
}

SimpleStat& SimpleStat::operator +=(const SimpleStat& o)
{
	if ((o.factor != factor) || (o.hist_bin != hist_bin))
		throw LIMA_HW_EXC(Error, "Cannot add different SimpleStats");

	AutoMutex l(o.lock);
	xmin = xn ? (o.xn ? std::min(xmin, o.xmin) : xmin) : o.xmin;
	xmax = xn ? (o.xn ? std::max(xmax, o.xmax) : xmin) : o.xmax;
	xacc += o.xacc;
	xacc2 += o.xacc2;
	xn += o.xn;
	Histogram::const_iterator oit, oend = o.hist.end();
	for (oit = o.hist.begin(); oit != oend; ++oit) {
		Histogram::iterator it = hist.find(oit->first);
		if (it == hist.end())
			hist[oit->first] = oit->second;
		else
			it->second += oit->second;
	}

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

XYStat::XYStat(double f)
	: factor(f)
{
	reset();
}

void XYStat::reset()
{
	AutoMutex l(lock);
	xacc = xacc2 = yacc = xyacc = 0;
	xn = 0;
}

void XYStat::add(double x, double y) {
	AutoMutex l(lock);
	y *= factor;
	xacc += x;
	xacc2 += pow(x, 2);
	yacc += y;
	xyacc += x * y;
	++xn;
}

XYStat& XYStat::operator =(const XYStat& o)
{
	if (&o == this)
		return *this;

	AutoMutex l(o.lock);
	xacc = o.xacc;
	xacc2 = o.xacc2;
	yacc = o.yacc;
	xyacc = o.xyacc;
	xn = o.xn;
	factor = o.factor;
	return *this;
}

XYStat& XYStat::operator +=(const XYStat& o)
{
	if (o.factor != factor)
		throw LIMA_HW_EXC(Error, "Cannot add different XYStats");

	AutoMutex l(o.lock);
	xacc += o.xacc;
	xacc2 += o.xacc2;
	yacc += o.yacc;
	xyacc += o.xyacc;
	xn += o.xn;
	return *this;
}

int XYStat::n() const
{ 
	AutoMutex l(lock);
	return xn;
}

XYStat::LinRegress XYStat::calcLinRegress() const
{ 
	AutoMutex l(lock);
	LinRegress r;
	if (!xn)
		return r;
	r.n = xn;
	r.slope = (xn * xyacc - xacc * yacc) / (xn * xacc2 - xacc * xacc);
	r.offset = (yacc - r.slope * xacc) / xn;
	return r;
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

ostream& lima::SlsDetector::operator <<(ostream& os, AcqState state)
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

ostream& lima::SlsDetector::operator <<(ostream& os, const StringList& l)
{
	os << "[";
	StringList::const_iterator it, end = l.end();
	bool first = true;
	for (it = l.begin(); it != end; ++it, first = false)
		os << (first ? "" : ",") << *it;
	return os << "]";
}

ostream& lima::SlsDetector::operator <<(ostream& os, const SortedIntList& l)
{
	return os << PrettySortedList(l);
}

ostream& lima::SlsDetector::operator <<(ostream& os, const FrameArray& a)
{
	os << "[";
	for (unsigned int i = 0; i < a.size(); ++i)
		os << (!i ? "" : ", ") << a[i];
	return os << "]";
}

ostream& lima::SlsDetector::operator <<(ostream& os, const 
					SimpleStat::Histogram& h)
{
	os << "{";
	SimpleStat::Histogram::const_iterator it, end = h.end();
	bool first = true;
	for (it = h.begin(); it != end; ++it, first = false)
		os << (first ? "" : ",") << it->first << ":" << it->second;
	return os << "}";
}

ostream& lima::SlsDetector::operator <<(ostream& os, const SimpleStat& s)
{
	os << "<";
	os << "min=" << int(s.min()) << ", max=" << int(s.max()) << ", "
	   << "ave=" << int(s.ave()) << ", std=" << int(s.std()) << ", "
	   << "n=" << s.n() << ", hist=" << s.hist;
	return os << ">";
}

ostream& lima::SlsDetector::operator <<(ostream& os,
					const XYStat::LinRegress& r)
{
	os << "<";
	os << "slope=" << r.slope << ", "
	   << "offset=" << r.offset << ", "
	   << "n=" << r.n;
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

Stats& Stats::operator +=(const Stats& o)
{
	cb_period += o.cb_period;
	new_finish += o.new_finish;
	cb_exec += o.cb_exec;
	recv_exec += o.recv_exec;
	return *this;
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

