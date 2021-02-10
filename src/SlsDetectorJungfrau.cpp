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

#include "sls/Geometry.h"

#include <emmintrin.h>
#include <sched.h>

using namespace std;
using namespace lima;
using namespace lima::SlsDetector;
using namespace lima::SlsDetector::Defs;


#define applyDetGeom(j, f, raw)						\
	using namespace sls::Geom::Jungfrau;				\
	int ifaces;							\
	j->getNbUDPInterfaces(ifaces);					\
	auto any_nb_ifaces = AnyNbUDPIfacesFromNbUDPIfaces(ifaces);	\
	std::visit([&](auto nb) {					\
	    constexpr int nb_udp_ifaces = nb;				\
	    Defs::xy det_size = j->m_det->getDetectorSize();		\
	    auto any_geom = AnyDetGeomFromDetSize<nb_udp_ifaces>({det_size.x, \
								  det_size.y});\
	    std::visit([&](auto geom) {					\
	        if (raw)						\
			f(geom.raw_geom);				\
		else							\
			f(geom.asm_wg_geom);				\
	      }, any_geom);						\
	  }, any_nb_ifaces);

/*
 * Jungfrau::Recv class
 */

Jungfrau::Recv::Recv(Jungfrau *jungfrau, int idx)
	: m_jungfrau(jungfrau), m_idx(idx)
{
	DEB_CONSTRUCTOR();
	DEB_PARAM() << DEB_VAR1(m_idx);

	Camera *cam = m_jungfrau->getCamera(); 
	m_recv = cam->getRecv(m_idx);
}

void Jungfrau::Recv::prepareAcq()
{
	DEB_MEMBER_FUNCT();
	m_raw = m_jungfrau->getRawMode();
	m_frame_dim = m_jungfrau->getModuleFrameDim(m_idx, m_raw);
	m_data_offset = m_jungfrau->getModuleDataOffset(m_idx, m_raw);
	DEB_TRACE() << DEB_VAR3(m_idx, m_frame_dim, m_data_offset);
}

bool Jungfrau::Recv::processOneFrame(FrameType frame, char *bptr)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR2(m_idx, frame);
	RecvImageData data;
	data.frame = frame;
	data.buffer = bptr;
	return m_recv->getImage(data);
}

void Jungfrau::Recv::processBadFrame(FrameType frame, char *bptr)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR2(m_idx, frame);
	if (!m_raw)
		THROW_HW_ERROR(NotSupported) << "Bad Frames in assembled mode";
	memset(bptr + m_data_offset, 0xff, m_frame_dim.getMemSize());
}


/*
 * Jungfrau::Thread class
 */

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


/*
 * Jungfrau::ImgProcTask class
 */

Jungfrau::ImgProcTask::ImgProcTask(Jungfrau *jungfrau)
	: m_jungfrau(jungfrau)
{
	DEB_CONSTRUCTOR();
}

void Jungfrau::ImgProcTask::setConfig(string config)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR1(config);
	m_jungfrau->setImgProcConfig(config);
}

void Jungfrau::ImgProcTask::getConfig(string &config)
{
	DEB_MEMBER_FUNCT();
	m_jungfrau->getImgProcConfig(config);
	DEB_RETURN() << DEB_VAR1(config);
}

void Jungfrau::ImgProcTask::process(Data& data)
{
	DEB_MEMBER_FUNCT();

	DEB_PARAM() << DEB_VAR2(data.frameNumber, data.data());
	
	ImgProcList& img_proc_list = m_jungfrau->m_img_proc_list;
	ImgProcList::iterator it, end = img_proc_list.end();
	for (it = img_proc_list.begin(); it != end; ++it)
		(*it)->processFrame(data);
}


/*
 * Jungfrau::ImgProcBase class
 */

Jungfrau::ImgProcBase::ImgProcBase(Jungfrau *jungfrau, std::string name)
	: m_jungfrau(jungfrau), m_name(name)
{
	DEB_CONSTRUCTOR();
	DEB_PARAM() << DEB_VAR1(m_name);
	m_nb_jungfrau_modules = m_jungfrau->getNbJungfrauModules();
	auto f = [&](auto det_geom) {
		m_det_mods = Point(det_geom.det_mods.x, det_geom.det_mods.y);
		DEB_TRACE() << DEB_VAR2(m_det_mods, m_nb_jungfrau_modules);
	};
	applyDetGeom(m_jungfrau, f, false);
}

Jungfrau::ImgProcBase::~ImgProcBase()
{
	DEB_DESTRUCTOR();
	if (m_jungfrau)
		m_jungfrau->removeImgProc(this);
}

void Jungfrau::ImgProcBase::updateImageSize(Size size, bool raw)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR3(m_name, size, raw);
	m_pixels = size.getWidth() * size.getHeight();
	m_frame_size = size;
	m_raw = raw;
}

void Jungfrau::ImgProcBase::prepareAcq()
{
	DEB_MEMBER_FUNCT();

	if (!m_jungfrau)
		THROW_HW_ERROR(InvalidValue) << "ImgProc already removed";
}

/*
 * Jungfrau::ImgProcBase class
 */

Jungfrau::GainADCMapImgProc::GainADCMapImgProc(Jungfrau *jungfrau)
	: ImgProcBase(jungfrau, "gain_adc_map"), m_reader(new ReaderHelper)
{
	DEB_CONSTRUCTOR();
}

void Jungfrau::GainADCMapImgProc::updateImageSize(Size size, bool raw)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR3(m_name, size, raw);
	ImgProcBase::updateImageSize(size, raw);

	for (auto& d : m_buffer)
		d.updateSize(size);
}

void Jungfrau::GainADCMapImgProc::prepareAcq()
{
	DEB_MEMBER_FUNCT();
	ImgProcBase::prepareAcq();

	for (auto& d : m_buffer)
		d.clear();
	m_buffer.reset();
}

void Jungfrau::GainADCMapImgProc::processFrame(Data& data)
{
	DEB_MEMBER_FUNCT();
	long frame = data.frameNumber;
	DEB_PARAM() << DEB_VAR1(frame);

	DoubleBufferWriter<MapData> w(m_buffer);
	MapData& m = w.getBuffer();
	DEB_TRACE() << DEB_VAR1(&m);
	unsigned short *src;
	{
		src = (unsigned short *) data.data();
		unsigned char *dst = (unsigned char *) m.gain_map.data();
		for (int i = 0; i < m_pixels; ++i)
			*dst++ = *src++ >> 14;
		m.gain_map.frameNumber = frame;
	}
	{
		src = (unsigned short *) data.data();
		unsigned short *dst = (unsigned short *) m.adc_map.data();
		for (int i = 0; i < m_pixels; ++i)
			*dst++ = *src++ & 0x3fff;
		m.adc_map.frameNumber = frame;
	}
	w.setCounter(frame);
}

void Jungfrau::GainADCMapImgProc::readGainADCMaps(Data& gain_map, Data& adc_map,
						  FrameType& frame)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR1(frame);
	m_reader->addRead(m_buffer, gain_map, adc_map, frame);
	DEB_RETURN() << DEB_VAR3(gain_map, adc_map, frame);
}

/*
 * Jungfrau detector class
 */

Jungfrau::Jungfrau(Camera *cam)
	: Model(cam, JungfrauDet)
{
	DEB_CONSTRUCTOR();

	int nb_det_modules = getNbDetModules();

	DEB_TRACE() << "Using Jungfrau detector, " << DEB_VAR1(nb_det_modules);

	for (int i = 0; i < nb_det_modules; ++i) {
		Recv *r = new Recv(this, i);
		m_recv_list.push_back(r);
	}

	setNbProcessingThreads(1);

	updateCameraModel();
}

Jungfrau::~Jungfrau()
{
	DEB_DESTRUCTOR();
	getCamera()->waitAcqState(Idle);
	removeAllImgProc();
}

void Jungfrau::getFrameDim(FrameDim& frame_dim, bool raw)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR1(raw);
	Size size;
	auto f = [&](auto det_geom) {
		size = Size(det_geom.size.x, det_geom.size.y);
		DEB_TRACE() << DEB_VAR1(size);
	};
	applyDetGeom(this, f, raw);
	frame_dim = FrameDim(size, Bpp16);
	DEB_RETURN() << DEB_VAR1(frame_dim);
}

template <class DG>
Defs::xy Jungfrau::getModulePosition(const DG& det_geom, int idx)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR1(idx);
	Defs::xy mod_pos;
	auto det_mods = det_geom.det_mods;
	// module ID is column-wise
	mod_pos.x = idx / det_mods.y;
	mod_pos.y = idx % det_mods.y;
	DEB_RETURN() << DEB_VAR2(mod_pos.x, mod_pos.y);
	return mod_pos;
}

FrameDim Jungfrau::getModuleFrameDim(int idx, bool raw)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR2(idx, raw);
	Size size;
	auto f = [&](auto det_geom) {
		auto mod_geom = det_geom.getModGeom({0, 0});
		bool last_x = true;  // only raw mode supported
		bool last_y = (idx == getNbJungfrauModules() - 1);
		size = Size(!last_x ? det_geom.mod_step.x : mod_geom.size.x,
			    !last_y ? det_geom.mod_step.y : mod_geom.size.y);
		DEB_TRACE() << DEB_VAR1(size);
	};
	applyDetGeom(this, f, raw);
	FrameDim frame_dim(size, Bpp16);
	DEB_RETURN() << DEB_VAR1(frame_dim);
	return frame_dim;
}

int Jungfrau::getModuleDataOffset(int idx, bool raw)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR2(idx, raw);
	int data_offset;
	auto f = [&](auto det_geom) {
		auto mod_pos = getModulePosition(det_geom, idx);
		auto mod_view = det_geom.getModView({mod_pos.x,
					mod_pos.y});
		auto origin = mod_view.calcViewOrigin();
		int pixel_offset = mod_view.calcMapPixelIndex(origin);
		data_offset = pixel_offset * sls::Geom::Pixel16::depth();
		DEB_TRACE() << DEB_VAR1(data_offset);
	};
	applyDetGeom(this, f, raw);
	DEB_RETURN() << DEB_VAR1(data_offset);
	return data_offset;
}

void Jungfrau::getDetMap(Data& det_map)
{
	DEB_MEMBER_FUNCT();
	bool raw = getRawMode();
	FrameDim frame_dim;
	getFrameDim(frame_dim, raw);
	Buffer *b = new Buffer(frame_dim.getMemSize() * 2);
	det_map.type = Data::UINT32;
	Size size = frame_dim.getSize();
	std::vector<int> dims = {size.getWidth(), size.getHeight()};
	det_map.dimensions = dims;
	det_map.setBuffer(b);
	b->unref();
	DEB_ALWAYS() << DEB_VAR1(det_map.size());

	auto f = [&](auto det_geom) {
		using namespace sls::Geom;

		// gap pixels are set to -1
		unsigned int *p = (unsigned int *) det_map.data();
		memset(p, 0xff, det_map.size());

		int chip_idx = 0;
		det_for_each_mod(det_geom, mod, mod_geom,
		    mod_for_each_recv(mod_geom, recv, recv_geom,
		        recv_for_each_iface(recv_geom, iface, iface_geom,
		            iface_for_each_chip(iface_geom, chip, chip_view,
			        view_for_each_pixel(chip_view, pixel,
				    int i = chip_view.calcMapPixelIndex(pixel);
				    p[i] = chip_idx;
				  );
				++chip_idx;
			      );
		          );
		      );
		  );
	};
	applyDetGeom(this, f, raw);
	DEB_ALWAYS() << DEB_VAR1(det_map);
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
#define JUNGFRAU_DAC_MV(x)		{x, 1}

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

	int nb_udp_ifaces;
	getNbUDPInterfaces(nb_udp_ifaces);

	double min_exp = 0.1;
	double min_period = 1000 / nb_udp_ifaces;
	double min_lat = 0.1;

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

	bool raw = getRawMode();
	FrameDim frame_dim;
	getFrameDim(frame_dim, raw);
	DEB_TRACE() << DEB_VAR2(frame_dim, raw);

	ImgProcList::iterator it, end = m_img_proc_list.end();
	for (it = m_img_proc_list.begin(); it != end; ++it)
		(*it)->updateImageSize(frame_dim.getSize(), raw);
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

int Jungfrau::getNbRecvs()
{
	DEB_MEMBER_FUNCT();
	int nb_recvs = m_recv_list.size();
	DEB_RETURN() << DEB_VAR1(nb_recvs);
	return nb_recvs;
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

	FrameType nb_frames;
	Camera *cam = getCamera();
	cam->getNbFrames(nb_frames);

	{
		AutoMutex l = lock();
		m_in_process.clear();
		m_nb_frames = nb_frames;
		m_next_frame = 0;
		m_last_frame = -1;
	}

	RecvList::iterator rit, rend = m_recv_list.end();
	for (rit = m_recv_list.begin(); rit != rend; ++rit)
		(*rit)->prepareAcq();

	ThreadList::iterator tit, tend = m_thread_list.end();
	for (tit = m_thread_list.begin(); tit != tend; ++tit)
		(*tit)->prepareAcq();

	ImgProcList::iterator pit, pend = m_img_proc_list.end();
	for (pit = m_img_proc_list.begin(); pit != pend; ++pit)
		(*pit)->prepareAcq();
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
		std::bitset<64> ok;
		int nb_recvs = getNbRecvs();
		if (nb_recvs > 64)
			THROW_HW_ERROR(Error) << "Too many receivers";
		char *bptr = getFrameBufferPtr(frame);
		for (int i = 0; i < nb_recvs; ++i)
			ok[i] = m_recv_list[i]->processOneFrame(frame, bptr);

		if (ok.none())
			return;

		for (int i = 0; i < nb_recvs; ++i)
			if (!ok[i])
				m_recv_list[i]->processBadFrame(frame, bptr);
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

Jungfrau::ImgProcTask *Jungfrau::createImgProcTask()
{
	DEB_MEMBER_FUNCT();
	return new ImgProcTask(this);
}

void Jungfrau::addImgProc(ImgProcBase *img_proc)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR1(img_proc);
	m_img_proc_list.push_back(img_proc);
	bool raw = getRawMode();
	FrameDim frame_dim;
	getFrameDim(frame_dim, raw);
	img_proc->updateImageSize(frame_dim.getSize(), raw);
}

void Jungfrau::removeImgProc(ImgProcBase *img_proc)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR1(img_proc);

	ImgProcList::iterator it, end = m_img_proc_list.end();
	it = find(m_img_proc_list.begin(), end, img_proc);
	if (it != end)
		m_img_proc_list.erase(it);
	img_proc->m_jungfrau = NULL;
}

void Jungfrau::removeAllImgProc()
{
	DEB_MEMBER_FUNCT();
	while (m_img_proc_list.size() > 0)
		delete *m_img_proc_list.rbegin();
}

Jungfrau::ImgProcBase *Jungfrau::createGainADCMapImgProc()
{
	DEB_MEMBER_FUNCT();
	ImgProcBase *map_img_proc = new GainADCMapImgProc(this);
	addImgProc(map_img_proc);
	DEB_RETURN() << DEB_VAR1(map_img_proc);
	return map_img_proc;
}

void Jungfrau::setImgProcConfig(std::string config)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR1(config);
	if (config == m_img_proc_config)
		return;
	if (config.empty()) {
		removeAllImgProc();
	} else if (config == "gain_adc_map") {
		createGainADCMapImgProc();
	} else {
		THROW_HW_ERROR(InvalidValue) << "Invalid " << DEB_VAR1(config);
	}
	m_img_proc_config = config;
}

void Jungfrau::getImgProcConfig(std::string &config)
{
	DEB_MEMBER_FUNCT();
	config = m_img_proc_config;
	DEB_RETURN() << DEB_VAR1(config);
}

void Jungfrau::readGainADCMaps(Data& gain_map, Data& adc_map, FrameType& frame)
{
	DEB_MEMBER_FUNCT();
	ImgProcList::iterator it, end = m_img_proc_list.end();
	for (it = m_img_proc_list.begin(); it != end; ++it) {
		if ((*it)->m_name == "gain_adc_map")
			break;
	}
	if (it == end)
		THROW_HW_ERROR(Error) << "ImgProc gain_adc_map not found";
	GainADCMapImgProc *img_proc = static_cast<GainADCMapImgProc *>(*it);
	img_proc->readGainADCMaps(gain_map, adc_map, frame);
}

