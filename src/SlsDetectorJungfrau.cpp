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

#include "SlsDetectorJungfrau.h"
#include "lima/MiscUtils.h"

#include <emmintrin.h>
#include <sched.h>

using namespace std;
using namespace lima;
using namespace lima::SlsDetector;
using namespace lima::SlsDetector::Defs;

const int Jungfrau::ChipSize = 256;
const int Jungfrau::ChipGap = 2;
const int Jungfrau::HalfModuleChips = 4;

Jungfrau::Thread::Thread(Jungfrau *jungfrau, int idx)
	: m_jungfrau(jungfrau), m_idx(idx)
{
	DEB_MEMBER_FUNCT();

	AutoMutex l = lock();
	m_state = Init;

	start();

	struct sched_param param;
	param.sched_priority = 50;
	int ret = pthread_setschedparam(m_thread, SCHED_RR, &param);
	if (ret != 0)
		DEB_ERROR() << "Could not set real-time priority!!";

	while (m_state == Init)
		wait();
}

Jungfrau::Thread::~Thread()
{
	DEB_DESTRUCTOR();

	AutoMutex l = lock();
	m_state = Quitting;
	broadcast();
	while (m_state != End)
		wait();
}

void Jungfrau::Thread::threadFunction()
{
	DEB_MEMBER_FUNCT();

	State& s = m_state;

	AutoMutex l = lock();
	s = Ready;
	broadcast();

	while (s != Quitting) {
		while ((s == Ready) || (s == Stopping)
		       || ((s == Running) && m_jungfrau->allFramesAcquired())) {
			if (s == Stopping) {
				s = Ready;
				broadcast();
			}
			wait();
		}
		if (s == Running)
			m_jungfrau->processOneFrame(l);
	}

	s = End;
	broadcast();
}

void Jungfrau::Thread::setCPUAffinity(CPUAffinity aff)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR1(aff);

        aff.applyToTask(getThreadID(), false);
}

void Jungfrau::Thread::prepareAcq()
{
	DEB_MEMBER_FUNCT();
}

Jungfrau::Jungfrau(Camera *cam)
	: Model(cam, JungfrauDet)
{
	DEB_CONSTRUCTOR();

	int nb_det_modules = getNbDetModules();
	if (nb_det_modules > 1)
		THROW_HW_ERROR(NotSupported) << "Multimodule not supported yet";

	DEB_TRACE() << "Using Jungfrau detector, " << DEB_VAR1(nb_det_modules);

	m_recv = cam->getRecv(0);

	setNbProcessingThreads(1);

	updateCameraModel();
}

Jungfrau::~Jungfrau()
{
	DEB_DESTRUCTOR();
}

void Jungfrau::getFrameDim(FrameDim& frame_dim, bool raw)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR1(raw);
	Size size = Point(ChipSize, ChipSize) * Point(HalfModuleChips, 2);
	if (!raw)
		size += Point(ChipGap, ChipGap) * Point(3, 1);
	frame_dim = FrameDim(size, Bpp16);
	DEB_RETURN() << DEB_VAR1(frame_dim);
}

string Jungfrau::getName()
{
	DEB_MEMBER_FUNCT();
	ostringstream os;
	os << "PSI/Jungfrau-";
	int nb_modules = getNbDetModules();
	if (nb_modules == 1) {
		os << "500k";
	} else if (nb_modules % 2 == 0) {
		os << (nb_modules / 2) << "M";
	} else {
		os << nb_modules << "-Modules";
	}
	string name = os.str();
	DEB_RETURN() << DEB_VAR1(name);
	return name;
}

void Jungfrau::getPixelSize(double& x_size, double& y_size)
{
	DEB_MEMBER_FUNCT();
	x_size = y_size = 75e-6;
	DEB_RETURN() << DEB_VAR2(x_size, y_size);
}

void Jungfrau::getDACInfo(NameList& name_list, IntList& idx_list,
		       IntList& milli_volt_list)
{
	DEB_MEMBER_FUNCT();

#define JUNGFRAU_DAC(x)			{x, 0}
#define JUNGFRAU_DAC_MV(x)			{x, 1}

	static struct DACData {
		DACIndex idx;
		int milli_volt;
	} JungfrauDACList[] = {
		JUNGFRAU_DAC(Threshold),
	};
	const unsigned int size = C_LIST_SIZE(JungfrauDACList);

	name_list.resize(size);
	idx_list.resize(size);
	milli_volt_list.resize(size);
	struct DACData *data = JungfrauDACList;
	for (unsigned int i = 0; i < size; ++i, ++data) {
		ostringstream os;
		os << data->idx;
		name_list[i] = os.str();
		idx_list[i] = int(data->idx);
		milli_volt_list[i] = data->milli_volt;
		DEB_RETURN() << DEB_VAR2(name_list[i], idx_list[i]);
	}
}

void Jungfrau::getADCInfo(NameList& name_list, IntList& idx_list,
		       FloatList& factor_list, FloatList& min_val_list)
{
	DEB_MEMBER_FUNCT();

#define JUNGFRAU_TEMP_FACTOR		(1 / 1000.0)

#define JUNGFRAU_TEMP(x)			{x, JUNGFRAU_TEMP_FACTOR, 0}

	static struct ADCData {
		ADCIndex idx;
		double factor, min_val;
	} JungfrauADCList[] = {
		JUNGFRAU_TEMP(TempFPGA),
		JUNGFRAU_TEMP(TempFPGAExt),
		JUNGFRAU_TEMP(Temp10GE),
		JUNGFRAU_TEMP(TempDCDC),
		JUNGFRAU_TEMP(TempSODL),
		JUNGFRAU_TEMP(TempSODR),
		JUNGFRAU_TEMP(TempFPGAFL),
		JUNGFRAU_TEMP(TempFPGAFR),
	};
	const unsigned int size = C_LIST_SIZE(JungfrauADCList);

	name_list.resize(size);
	idx_list.resize(size);
	factor_list.resize(size);
	min_val_list.resize(size);
	struct ADCData *data = JungfrauADCList;
	for (unsigned int i = 0; i < size; ++i, ++data) {
		ostringstream os;
		os << data->idx;
		name_list[i] = os.str();
		idx_list[i] = int(data->idx);
		factor_list[i] = data->factor;
		min_val_list[i] = data->min_val;
		DEB_RETURN() << DEB_VAR4(name_list[i], idx_list[i], 
					 factor_list[i], min_val_list[i]);
	}
}

void Jungfrau::getTimeRanges(TimeRanges& time_ranges)
{
	DEB_MEMBER_FUNCT();

	double min_exp = 10;
	double min_period = 1000;
	double min_lat = 500;

	time_ranges.min_exp_time = min_exp * 1e-6;
	time_ranges.max_exp_time = 1e3;
	time_ranges.min_lat_time = min_lat * 1e-6;
	time_ranges.max_lat_time = 1e3;
	time_ranges.min_frame_period = min_period * 1e-6;
	time_ranges.max_frame_period = 1e3;
}

int Jungfrau::getNbFrameMapItems()
{
	DEB_MEMBER_FUNCT();
	int nb_items = 1;
	DEB_RETURN() << DEB_VAR1(nb_items);
	return nb_items;
}

void Jungfrau::updateFrameMapItems(FrameMap *map)
{
	DEB_MEMBER_FUNCT();
	m_frame_map_item = map->getItem(0);
}

void Jungfrau::processBadItemFrame(FrameType frame, int item, char *bptr)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR2(frame, item);
	DEB_ERROR() << "Bad frames not supported yet";
}

void Jungfrau::updateImageSize()
{
	DEB_MEMBER_FUNCT();

	Camera *cam = getCamera();
	bool raw;
	cam->getRawMode(raw);
	DEB_TRACE() << DEB_VAR1(raw);
}

bool Jungfrau::checkSettings(Settings settings)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR1(settings);
	bool ok;
	switch (settings) {
	case Defs::Standard:
		ok = true;
		break;
	default:
		ok = false;
	}

	DEB_RETURN() << DEB_VAR1(ok);
	return ok;
}

void Jungfrau::setHighVoltage(int hvolt)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR1(hvolt);
	DEB_ALWAYS() << "Setting high voltage (" << DEB_VAR1(hvolt) << ") ...";
	EXC_CHECK(m_det->setHighVoltage(hvolt));
}

void Jungfrau::getHighVoltage(int& hvolt)
{
	DEB_MEMBER_FUNCT();
	Positions pos = Idx2Pos(0);
	EXC_CHECK(hvolt = m_det->getHighVoltage(pos).front());
	DEB_RETURN() << DEB_VAR1(hvolt);
}

void Jungfrau::setThresholdEnergy(int thres)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR1(thres);
	EXC_CHECK(m_det->setThresholdEnergy(thres));
}

void Jungfrau::getThresholdEnergy(int& thres)
{
	DEB_MEMBER_FUNCT();
	EXC_CHECK(thres = m_det->getThresholdEnergy().squash(-1));
	DEB_RETURN() << DEB_VAR1(thres);
}

void Jungfrau::setNbProcessingThreads(int nb_proc_threads)
{
	DEB_MEMBER_FUNCT();
	int curr_nb_proc_threads = m_thread_list.size();
	DEB_PARAM() << DEB_VAR2(curr_nb_proc_threads, nb_proc_threads);

	if (nb_proc_threads < 1)
		THROW_HW_ERROR(InvalidValue) << DEB_VAR1(nb_proc_threads);
	if (nb_proc_threads == curr_nb_proc_threads)
		return;

	if (nb_proc_threads < curr_nb_proc_threads)
		m_thread_list.resize(nb_proc_threads);

	for (int i = curr_nb_proc_threads; i < nb_proc_threads; ++i) {
		Thread *t = new Thread(this, i);
		m_thread_list.push_back(t);
	}
}

int Jungfrau::getNbProcessingThreads()
{
	DEB_MEMBER_FUNCT();
	int nb_proc_threads = m_thread_list.size();
	DEB_RETURN() << DEB_VAR1(nb_proc_threads);
	return nb_proc_threads;
}

void Jungfrau::setThreadCPUAffinity(const CPUAffinityList& aff_list)
{
	DEB_MEMBER_FUNCT();

	setNbProcessingThreads(aff_list.size());

	CPUAffinityList::const_iterator rit = aff_list.begin();
	ThreadList::iterator it, end = m_thread_list.end();
	for (it = m_thread_list.begin(); it != end; ++it, ++rit)
		(*it)->setCPUAffinity(*rit);
}

void Jungfrau::prepareAcq()
{
	DEB_MEMBER_FUNCT();

	m_in_process.clear();

	Camera *cam = getCamera();
	cam->getNbFrames(m_nb_frames);
	m_next_frame = 0;
	m_last_frame = -1;

	ThreadList::iterator tit, tend = m_thread_list.end();
	for (tit = m_thread_list.begin(); tit != tend; ++tit)
		(*tit)->prepareAcq();
}

void Jungfrau::startAcq()
{
	DEB_MEMBER_FUNCT();
	ThreadList::iterator it, end = m_thread_list.end();
	for (it = m_thread_list.begin(); it != end; ++it)
		(*it)->startAcq();
}

void Jungfrau::stopAcq()
{
	DEB_MEMBER_FUNCT();
	ThreadList::iterator it, end = m_thread_list.end();
	for (it = m_thread_list.begin(); it != end; ++it)
		(*it)->stopAcq();
}

void Jungfrau::processOneFrame(AutoMutex& l)
{
	DEB_MEMBER_FUNCT();

	FrameType frame = m_next_frame++;

	class Sync
	{
	public:
		Sync(FrameType f, Jungfrau& e)
			: frame(f), jungfrau(e), in_proc(e.m_in_process)
		{
			in_proc.insert(frame);
		}

		~Sync()
		{
			in_proc.erase(frame);
			jungfrau.broadcast();
		}

		void startFinish()
		{
			while (!in_proc.empty()
			       && (*in_proc.begin() < int(frame)))
				jungfrau.wait();
		}

	private:
		FrameType frame;
		Jungfrau& jungfrau;
		SortedIntList& in_proc;
	} sync(frame, *this);

	{
		AutoMutexUnlock u(l);
		Receiver::ImageData data;
		data.frame = frame;
		data.buffer = getFrameBufferPtr(frame);
		if (!m_recv->getImage(data))
			return;
	}

	if ((int(m_last_frame) == -1) || (frame > m_last_frame))
		m_last_frame = frame;

	sync.startFinish();

	{
		AutoMutexUnlock u(l);
		FrameMap::Item& mi = *m_frame_map_item;
		FinishInfo finfo = mi.frameFinished(frame, true, true);
		processFinishInfo(finfo);
	}
}

bool Jungfrau::isXferActive()
{
	DEB_MEMBER_FUNCT();
	bool xfer_active = 0;
	DEB_RETURN() << DEB_VAR1(xfer_active);
	return xfer_active;
}

