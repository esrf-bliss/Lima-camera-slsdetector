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

#include "multiSlsDetectorCommand.h"

#include <limits.h>
#include <algorithm>
#include <cmath>

using namespace std;
using namespace lima;
using namespace lima::SlsDetector;


Camera::AppInputData::AppInputData(string cfg_fname) 
	: config_file_name(cfg_fname)
{
	DEB_CONSTRUCTOR();
	parseConfigFile();
}

void Camera::AppInputData::parseConfigFile()
{
	DEB_MEMBER_FUNCT();

	ifstream config_file(config_file_name.c_str());
	while (config_file) {
		string s;
		config_file >> s;
		RegEx re;
		FullMatch full_match;
		MatchList match_list;
		MatchListIt lit, lend;

		re = "hostname";
		if (re.match(s, full_match)) {
			string host_name;
			config_file >> host_name;
			RegEx re("([A-Za-z0-9]+)\\+?");
			re.multiSearch(host_name, match_list);
			lend = match_list.end();
			for (lit = match_list.begin(); lit != lend; ++lit) {
				const FullMatch& full_match = *lit;
				const SingleMatch& single_match = full_match[1];
				host_name_list.push_back(single_match);
			}
			continue;
		}

		re = "([0-9]+):rx_tcpport";
		if (re.match(s, full_match)) {
			istringstream is(full_match[1]);
			int id;
			is >> id;
			if (id < 0)
				THROW_HW_FATAL(InvalidValue) << 
					"Invalid detector id: " << id;
			int rx_tcpport;
			config_file >> rx_tcpport;
			recv_port_map[id] = rx_tcpport;
			continue;
		}
	}
}


Camera::Receiver::Receiver(Camera *cam, int idx, int rx_port)
	: m_cam(cam), m_idx(idx), m_rx_port(rx_port)
{
	DEB_CONSTRUCTOR();

	ostringstream os;
	os << "slsReceiver"
	   << " --rx_tcpport " << m_rx_port;
	m_args.set(os.str());

	start();

	m_recv->registerCallBackStartAcquisition(fileStartCallback, this);
	m_recv->registerCallBackRawDataReady(portCallback, this);
}

Camera::Receiver::~Receiver()
{
	DEB_DESTRUCTOR();
	m_recv->stop();
}

void Camera::Receiver::start()
{	
	DEB_MEMBER_FUNCT();
	int init_ret;
	m_recv = new slsReceiverUsers(m_args.size(), m_args, init_ret);
	if (init_ret == slsReceiverDefs::FAIL)
		THROW_HW_ERROR(Error) << "Error creating slsReceiver";
	if (m_recv->start() == slsReceiverDefs::FAIL) 
		THROW_HW_ERROR(Error) << "Error starting slsReceiver";
}

int Camera::Receiver::fileStartCallback(char *fpath, char *fname, 
					uint64_t fidx, uint32_t dsize, 
					void *priv)
{
	DEB_STATIC_FUNCT();
	Receiver *recv = static_cast<Receiver *>(priv);
	return recv->fileStartCallback(fpath, fname, fidx, dsize);
}

void Camera::Receiver::portCallback(FrameType frame, 
				    uint32_t exp_len,
				    uint32_t recv_packets,
				    uint64_t bunch_id,
				    uint64_t timestamp,
				    uint16_t mod_id,
				    uint16_t x, uint16_t y, uint16_t z,
				    uint32_t debug,
				    uint16_t rr_nb,
				    uint8_t det_type,
				    uint8_t cb_version,
				    char *dptr, 
				    uint32_t dsize, 
				    void *priv)
{
	DEB_STATIC_FUNCT();
	Receiver *recv = static_cast<Receiver *>(priv);
	int port = (x % 2);
	FrameType lima_frame = frame - 1;
	DEB_PARAM() << DEB_VAR2(frame, lima_frame);
	recv->portCallback(lima_frame, port, dptr, dsize);
}

int Camera::Receiver::fileStartCallback(char *fpath, char *fname, 
					uint64_t fidx, uint32_t dsize)
{
	DEB_MEMBER_FUNCT();
	if (m_cam->m_model)
		m_cam->processRecvFileStart(m_idx, dsize);
	return 0;
}

void Camera::Receiver::portCallback(FrameType frame, int port, char *dptr, 
				    uint32_t dsize)
{
	DEB_MEMBER_FUNCT();

	if (!m_cam->m_model || (m_cam->getState() == Stopping))
		return;

	Timestamp t0 = Timestamp::now();

	int port_idx = m_cam->getPortIndex(m_idx, port);
	PortStats& port_stats = m_cam->m_port_stats[port_idx];
	if (port_stats.last_t0.isSet())
		port_stats.stats.cb_period.add(t0 - port_stats.last_t0);
	if (port_stats.last_t1.isSet())
		port_stats.stats.recv_exec.add(t0 - port_stats.last_t1);
	port_stats.last_t0 = t0;

	try {
		if (frame >= m_cam->m_nb_frames)
			THROW_HW_ERROR(Error) << "Invalid " 
					      << DEB_VAR2(frame, DebHex(frame));
		m_cam->processRecvPort(port_idx, frame, dptr, dsize);
	} catch (Exception& e) {
		ostringstream err_msg;
		err_msg << "Receiver::frameCallback: " << e << ": "
			<< DEB_VAR3(m_idx, frame, port);
		Event::Code err_code = Event::CamOverrun;
		Event *event = new Event(Hardware, Event::Error, Event::Camera, 
					 err_code, err_msg.str());
		DEB_EVENT(*event) << DEB_VAR1(*event);
		m_cam->reportEvent(event);
	}

	Timestamp t1 = Timestamp::now();
	port_stats.stats.cb_exec.add(t1 - t0);
	port_stats.last_t1 = t1;
}

Camera::BufferThread::BufferThread()
	: m_cam(NULL), m_port_idx(-1), m_frame_map_item(NULL)
{
	DEB_CONSTRUCTOR();
}

void Camera::BufferThread::init(Camera *cam, int port_idx)
{
	DEB_MEMBER_FUNCT();

	m_cam = cam;
	m_port_idx = port_idx;
	m_frame_map_item = &m_cam->m_frame_map.getItem(m_port_idx);

	start();
}

void Camera::BufferThread::prepareAcq()
{
	DEB_MEMBER_FUNCT();
	m_bad_frame_list.clear();
	m_bad_frame_list.reserve(16 * 1024);
}

Camera::BufferThread::~BufferThread()
{
	DEB_DESTRUCTOR();

	if (!hasStarted())
		return;

	{
		AutoMutex l = lock();
		m_end = true;
		m_cond.broadcast();
	}

	m_frame_map_item->stopPollFrameFinished();
}

void Camera::BufferThread::start()
{
	DEB_MEMBER_FUNCT();

	m_end = true;
	Thread::start();

	struct sched_param param;
	param.sched_priority = 50;
	int ret = pthread_setschedparam(m_thread, SCHED_RR, &param);
	if (ret != 0)
		DEB_ERROR() << "Could not set real-time priority!!";

	AutoMutex l = lock();
	while (m_end)
		m_cond.wait();
}

void Camera::BufferThread::threadFunction()
{
	DEB_MEMBER_FUNCT();

	m_tid = gettid();

	AutoMutex l = lock();

	m_end = false;
	m_cond.broadcast();
	while (!m_end) {
		AutoMutexUnlock u(l);
		FinishInfoList finfo_list;
		finfo_list = m_frame_map_item->pollFrameFinished();
		FinishInfoList::const_iterator it, end = finfo_list.end();
		for (it = finfo_list.begin(); it != end; ++it) {
			const FinishInfo& finfo = *it;
			if ((finfo.nb_lost != 0) || !finfo.finished.empty())
				processFinishInfo(finfo);
		}
	}
}

void Camera::BufferThread::processFinishInfo(const FinishInfo& finfo)
{
	DEB_MEMBER_FUNCT();

	try {
		if ((finfo.nb_lost > 0) && !m_cam->m_tol_lost_packets)
			THROW_HW_ERROR(Error) << "lost frames: "
					      << "port_idx=" << m_port_idx
					      << ", first=" << finfo.first_lost
					      << ", nb=" << finfo.nb_lost;
		{
			AutoMutex l = lock();
			FrameType f = finfo.first_lost;
			for (int i = 0; i < finfo.nb_lost; ++i, ++f)
				m_bad_frame_list.push_back(f);
		}
		SortedIntList::const_iterator it, end = finfo.finished.end();
		for (it = finfo.finished.begin(); it != end; ++it)
			m_cam->m_acq_thread->queueFinishedFrame(*it);
	} catch (Exception& e) {
		ostringstream err_msg;
		err_msg << "BufferThread::processRecvPort: " << e;
		Event::Code err_code = Event::CamOverrun;
		Event *event = new Event(Hardware, Event::Error, Event::Camera, 
					 err_code, err_msg.str());
		DEB_EVENT(*event) << DEB_VAR1(*event);
		m_cam->reportEvent(event);
	}
}

bool Camera::BufferThread::isBadFrame(FrameType frame)
{ 
	AutoMutex l = lock();
	IntList::iterator end = m_bad_frame_list.end();
	return (find(m_bad_frame_list.begin(), end, frame) != end); 
}

Camera::AcqThread::ExceptionCleanUp::ExceptionCleanUp(AcqThread& thread)
	: Thread::ExceptionCleanUp(thread)
{
	DEB_CONSTRUCTOR();
}

Camera::AcqThread::ExceptionCleanUp::~ExceptionCleanUp()
{
	DEB_DESTRUCTOR();
	AcqThread *thread = static_cast<AcqThread *>(&m_thread);
	thread->cleanUp();
}

Camera::AcqThread::AcqThread(Camera *cam)
	: m_cam(cam), m_cond(m_cam->m_cond), m_state(m_cam->m_state)
{
	DEB_CONSTRUCTOR();
}

void Camera::AcqThread::start()
{
	DEB_MEMBER_FUNCT();

	m_state = Starting;
	Thread::start();

	struct sched_param param;
	param.sched_priority = sched_get_priority_min(SCHED_RR);
	int ret = pthread_setschedparam(m_thread, SCHED_RR, &param);
	if (ret != 0)
		DEB_ERROR() << "Could not set AcqThread real-time priority!!";

	while (m_state != Running)
		m_cond.wait();
}

void Camera::AcqThread::stop(bool wait)
{
	DEB_MEMBER_FUNCT();
	m_state = StopReq;
	m_cond.broadcast();
	while (wait && (m_state != Stopped) && (m_state != Idle))
		m_cond.wait();
}

void Camera::AcqThread::threadFunction()
{
	DEB_MEMBER_FUNCT();

	ExceptionCleanUp cleanup(*this);

	AutoMutex l = m_cam->lock();

	GlobalCPUAffinityMgr& affinity_mgr = m_cam->m_global_cpu_affinity_mgr;
	{
		AutoMutexUnlock u(l);
		affinity_mgr.startAcq();
		startAcq();
	}
	m_state = Running;
	m_cond.broadcast();

	SeqFilter seq_filter;
	bool had_frames = false;
	bool cont_acq = true;
	do {
		while ((m_state != StopReq) && m_frame_queue.empty()) {
			if (!m_cond.wait(m_cam->m_new_frame_timeout)) {
				AutoMutexUnlock u(l);
				m_cam->checkLostPackets();
			}
		}
		if (!m_frame_queue.empty()) {
			FrameType frame = m_frame_queue.front();
			m_frame_queue.pop();
			DEB_TRACE() << DEB_VAR1(frame);
			seq_filter.addVal(frame);
			SeqFilter::Range frames = seq_filter.getSeqRange();
			if (frames.nb > 0) {
				AutoMutexUnlock u(l);
				int f = frames.first;
				do {
					DEB_TRACE() << DEB_VAR1(f);
					cont_acq = newFrameReady(f);
					had_frames = true;
				} while ((++f != frames.end()) && cont_acq);
			}
		}
	} while ((m_state != StopReq) && cont_acq);

	m_state = Stopping;
	{
		AutoMutexUnlock u(l);
		stopAcq();

		IntList bfl;
		m_cam->getSortedBadFrameList(bfl);
		DEB_ALWAYS() << "bad_frames=" << bfl.size() << ": "
			     << PrettyIntList(bfl);

		Stats stats;
		m_cam->getStats(stats);
		DEB_ALWAYS() << DEB_VAR1(stats);

		if (had_frames) {
			affinity_mgr.recvFinished();
			affinity_mgr.waitLimaFinished();
		} else {
			affinity_mgr.cleanUp();
		}
	}

	m_state = Stopped;
	m_cond.broadcast();
}

void Camera::AcqThread::queueFinishedFrame(FrameType frame)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR1(frame);
	AutoMutex l = m_cam->lock();
	m_frame_queue.push(frame);
	m_cond.broadcast();
}

void Camera::AcqThread::cleanUp()
{
	DEB_MEMBER_FUNCT();

	AutoMutex l = m_cam->lock();

	if ((m_state == Stopped) || (m_state == Idle))
		return;

	if ((m_state == Running) || (m_state == StopReq)) {
		m_state = Stopping;
		AutoMutexUnlock u(l);
		stopAcq();
	}

	{
		AutoMutexUnlock u(l);
		string err_msg = "AcqThread: exception thrown";
		Event::Code err_code = Event::CamFault;
		Event *event = new Event(Hardware, Event::Error, Event::Camera, 
					 err_code, err_msg);
		DEB_EVENT(*event) << DEB_VAR1(*event);
		m_cam->reportEvent(event);
	}

	m_state = Stopped;
	m_cond.broadcast();
}

void Camera::AcqThread::startAcq()
{
	DEB_MEMBER_FUNCT();
	DEB_TRACE() << "calling startReceiver";
	multiSlsDetector *det = m_cam->m_det;
	det->startReceiver();
	DEB_TRACE() << "calling startAcquisition";
	det->startAcquisition();
}

void Camera::AcqThread::stopAcq()
{
	DEB_MEMBER_FUNCT();
	multiSlsDetector *det = m_cam->m_det;
	if (m_cam->getDetStatus() == Defs::Running) {
		DEB_TRACE() << "calling stopAcquisition";
		det->stopAcquisition();
		Timestamp t0 = Timestamp::now();
		while (m_cam->getDetStatus() != Defs::Idle)
			Sleep(m_cam->m_abort_sleep_time);
		double milli_sec = (Timestamp::now() - t0) * 1e3;
		DEB_TRACE() << "Abort -> Idle: " << DEB_VAR1(milli_sec);
	}
	DEB_TRACE() << "calling stopReceiver";
	det->stopReceiver();
}

bool Camera::AcqThread::newFrameReady(FrameType frame)
{
	DEB_MEMBER_FUNCT();
	HwFrameInfoType frame_info;
	frame_info.acq_frame_nb = frame;
	bool cont_acq = m_cam->m_buffer_cb_mgr->newFrameReady(frame_info);
	return cont_acq && (frame < m_cam->m_nb_frames - 1);
}

Camera::Camera(string config_fname) 
	: m_model(NULL),
	  m_recv_fifo_depth(1000),
	  m_nb_frames(1),
	  m_lat_time(0),
	  m_recv_ports(0),
	  m_pixel_depth(PixelDepth16), 
	  m_image_type(Bpp16), 
	  m_raw_mode(false),
	  m_state(Idle),
	  m_new_frame_timeout(0.5),
	  m_abort_sleep_time(0.1),
	  m_tol_lost_packets(true),
	  m_time_ranges_cb(NULL),
	  m_global_cpu_affinity_mgr(this)
{
	DEB_CONSTRUCTOR();

	CPUAffinity::getNbCPUs();

	m_input_data = new AppInputData(config_fname);

	removeSharedMem();
	createReceivers();

	DEB_TRACE() << "Creating the multiSlsDetector object";
	m_det = new multiSlsDetector(0);
	DEB_TRACE() << "Reading configuration file";
	const char *fname = m_input_data->config_file_name.c_str();
	m_det->readConfigurationFile(fname);

	m_det->setReceiverFifoDepth(m_recv_fifo_depth);

	m_pixel_depth = PixelDepth(m_det->setDynamicRange(-1));

	setSettings(Defs::Standard);
	setTrigMode(Defs::Auto);
	setNbFrames(1);
	setExpTime(0.99);
	setFramePeriod(1.0);

	if (m_det->enableTenGigabitEthernet()) {
		DEB_TRACE() << "Forcing 10G Ethernet flow control";
		string val = "1";
		setNetworkParameter(Defs::FlowCtrl10G, val);
	}
}

Camera::~Camera()
{
	DEB_DESTRUCTOR();

	if (m_time_ranges_cb)
		unregisterTimeRangesChangedCallback(*m_time_ranges_cb);

	if (!m_model)
		return;

	stopAcq();
	m_model->m_cam = NULL;
}

Type Camera::getType()
{
	DEB_MEMBER_FUNCT();
	string type_resp = getCmd("type");
	ostringstream os;
	os << "(([^+]+)\\+){" << getNbDetModules() << "}";
	DEB_TRACE() << DEB_VAR1(os.str());
	RegEx re(os.str());
	FullMatch full_match;
	if (!re.match(type_resp, full_match))
		THROW_HW_ERROR(Error) << "Invalid type response: " << type_resp;
	string type_str = full_match[2];
	Type det_type = UnknownDet;
	if (type_str == "Generic") {
		det_type = GenericDet;
	} else if (type_str == "Eiger") {
		det_type = EigerDet;
	} else if (type_str == "Jungfrau") {
		det_type = JungfrauDet;
	}
	DEB_RETURN() << DEB_VAR1(det_type);
	return det_type;
}

void Camera::setModel(Model *model)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR2(model, m_model);
	
	if (model && (model->getType() != getType()))
		THROW_HW_ERROR(InvalidValue) << DEB_VAR2(getType(), 
							 model->getType());
	if (m_model == model)
		return;
	if (m_model)
		m_model->m_cam = NULL;
	m_model = model;
	if (!m_model)
		return;

	m_recv_ports = m_model->getRecvPorts();
	int nb_ports = getTotNbPorts();
	m_frame_map.setNbItems(nb_ports);
	if (!m_buffer_thread) {
		m_buffer_thread = new BufferThread[nb_ports];
		for (int i = 0; i < nb_ports; ++i)
			m_buffer_thread[i].init(this, i);
	}
	if (m_port_stats.empty())
		m_port_stats.resize(nb_ports);

	setPixelDepth(m_pixel_depth);
	setSettings(m_settings);
}

char *Camera::getFrameBufferPtr(FrameType frame_nb)
{
	DEB_MEMBER_FUNCT();

	StdBufferCbMgr *cb_mgr = m_buffer_cb_mgr;
	if (!cb_mgr)
		THROW_HW_ERROR(InvalidValue) << "No BufferCbMgr defined";
	void *ptr = cb_mgr->getFrameBufferPtr(frame_nb);
	return static_cast<char *>(ptr);
}

void Camera::removeSharedMem()
{
	DEB_MEMBER_FUNCT();
	const char *cmd = "ipcs -m | "
		"grep -E '^0x000016[0-9a-z]{2}' | "
		"awk '{print $2}' | while read m; do ipcrm -m $m; done";
	system(cmd);
}

void Camera::createReceivers()
{
	DEB_MEMBER_FUNCT();

	DEB_TRACE() << "Receivers:";
	const RecvPortMap& recv_port_map = m_input_data->recv_port_map;
	RecvPortMap::const_iterator mit, mend = recv_port_map.end();
	int idx = 0;
	for (mit = recv_port_map.begin(); mit != mend; ++mit, ++idx) {
		unsigned int id = mit->first;
		if (id >= m_input_data->host_name_list.size())
			THROW_HW_FATAL(InvalidValue) << DEB_VAR1(id) 
						     << "too high";
		const string& host_name = m_input_data->host_name_list[id];
		int rx_port = mit->second;
		DEB_TRACE() << "  " << host_name << ": " << DEB_VAR1(rx_port);

		AutoPtr<Receiver> recv_obj = new Receiver(this, idx, rx_port);
		m_recv_list.push_back(recv_obj);
	}
}

void Camera::putCmd(const string& s, int idx)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << "s=\"" << s << "\"";
	Args args(s);
	multiSlsDetectorCommand cmd(m_det);
	cmd.putCommand(args.size(), args, idx);
}

string Camera::getCmd(const string& s, int idx)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << "s=\"" << s << "\"";
	Args args(s);
	multiSlsDetectorCommand cmd(m_det);
	string r = cmd.getCommand(args.size(), args, idx);
	DEB_RETURN() << "r=\"" << r << "\"";
	return r;
}

void Camera::setTrigMode(TrigMode trig_mode)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR1(trig_mode);
	waitState(Idle);
	typedef slsDetectorDefs::externalCommunicationMode ExtComMode;
	ExtComMode mode = static_cast<ExtComMode>(trig_mode);
	m_det->setExternalCommunicationMode(mode);
	m_trig_mode = trig_mode;
	setNbFrames(m_nb_frames);
}

void Camera::getTrigMode(TrigMode& trig_mode)
{
	DEB_MEMBER_FUNCT();
	trig_mode = m_trig_mode;
	DEB_RETURN() << DEB_VAR1(trig_mode);
}

void Camera::setNbFrames(FrameType nb_frames)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR1(nb_frames);

	// Lima frame numbers are (signed) int
	const FrameType MaxFrames = INT_MAX;
	if (nb_frames >= MaxFrames)
		THROW_HW_ERROR(InvalidValue) << "too high " 
					     <<	DEB_VAR2(nb_frames, MaxFrames);

	waitState(Idle);
	bool trig_exp = (m_trig_mode == Defs::TriggerExposure);
	int cam_frames = trig_exp ? 1 : nb_frames;
	int cam_triggers = trig_exp ? nb_frames : 1;
	m_det->setNumberOfFrames(cam_frames);
	m_det->setNumberOfCycles(cam_triggers);
	m_nb_frames = nb_frames;
}

void Camera::getNbFrames(FrameType& nb_frames)
{
	DEB_MEMBER_FUNCT();
	nb_frames = m_nb_frames;
	DEB_RETURN() << DEB_VAR1(nb_frames);
}

void Camera::setExpTime(double exp_time)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR1(exp_time);
	waitState(Idle);
	m_det->setExposureTime(NSec(exp_time));
	m_exp_time = exp_time;
}

void Camera::getExpTime(double& exp_time)
{ 
	DEB_MEMBER_FUNCT();
	exp_time = m_exp_time;
	DEB_RETURN() << DEB_VAR1(exp_time);
}

void Camera::setLatTime(double lat_time)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR1(lat_time);
	m_lat_time = lat_time;
}

void Camera::getLatTime(double& lat_time)
{ 
	DEB_MEMBER_FUNCT();
	lat_time = m_lat_time;
	DEB_RETURN() << DEB_VAR1(lat_time);
}

void Camera::setFramePeriod(double frame_period)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR1(frame_period);

	if (m_model) {
		TimeRanges time_ranges;
		double e = 1e-6;
		m_model->getTimeRanges(time_ranges);
		if ((frame_period < time_ranges.min_frame_period - e) ||
		    (frame_period > time_ranges.max_frame_period + e))
			THROW_HW_ERROR(InvalidValue) 
				<< DEB_VAR3(frame_period,
					    time_ranges.min_frame_period, 
					    time_ranges.max_frame_period);
	}

	waitState(Idle);
	m_det->setExposurePeriod(NSec(frame_period));
	m_frame_period = frame_period;
}

void Camera::getFramePeriod(double& frame_period)
{
	DEB_MEMBER_FUNCT();
	frame_period = m_frame_period;
	DEB_RETURN() << DEB_VAR1(frame_period);
}

void Camera::updateImageSize()
{
	DEB_MEMBER_FUNCT();
	m_model->updateImageSize();
	FrameDim frame_dim;
	getFrameDim(frame_dim, m_raw_mode);
	DEB_TRACE() << "MaxImageSizeChanged: " << DEB_VAR1(frame_dim);
	maxImageSizeChanged(frame_dim.getSize(), frame_dim.getImageType());
}

void Camera::updateTimeRanges()
{
	DEB_MEMBER_FUNCT();
	TimeRanges time_ranges;
	m_model->getTimeRanges(time_ranges);
	m_exp_time = max(m_exp_time, time_ranges.min_exp_time);
	m_frame_period = max(m_frame_period, time_ranges.min_frame_period);
	DEB_TRACE() << "TimeRangesChanged: " 
		    << DEB_VAR6(time_ranges.min_exp_time, 
				time_ranges.max_exp_time,
				time_ranges.min_lat_time,
				time_ranges.max_lat_time,
				time_ranges.min_frame_period,
				time_ranges.max_frame_period);
	if (m_time_ranges_cb)
		m_time_ranges_cb->timeRangesChanged(time_ranges);
}

void Camera::updateCPUAffinity(bool recv_restarted)
{
	DEB_MEMBER_FUNCT();

	// receiver threads are restarted after DR change
	if (recv_restarted)
		m_global_cpu_affinity_mgr.updateRecvRestart();

	// apply the corresponding GlobalCPUAffinity
	GlobalCPUAffinity global_affinity = m_cpu_affinity_map[m_pixel_depth];
	m_global_cpu_affinity_mgr.applyAndSet(global_affinity);
}

void Camera::setRecvCPUAffinity(const RecvCPUAffinity& recv_affinity)
{
	DEB_MEMBER_FUNCT();

	CPUAffinity listeners_affinity = recv_affinity.listeners;
	CPUAffinity writers_affinity = recv_affinity.writers;
	cpu_set_t list_cpu_set;
	listeners_affinity.initCPUSet(list_cpu_set);
	cpu_set_t writ_cpu_set;
	writers_affinity.initCPUSet(writ_cpu_set);
	for (unsigned int i = 0; i < m_recv_list.size(); ++i) {
		DEB_TRACE() << "setting recv " << i << " "
			     << "listeners CPU mask to " << listeners_affinity;
		DEB_TRACE() << "setting recv " << i << " "
			     << "writers CPU mask to " << writers_affinity;
		slsReceiverUsers *recv = m_recv_list[i]->m_recv;
		recv->setThreadCPUAffinity(sizeof(list_cpu_set),
					   &list_cpu_set, &writ_cpu_set);
	}
	for (int i = 0; i < getTotNbPorts(); ++i) {
		pid_t tid = m_buffer_thread[i].getTID();
		writers_affinity.applyToTask(tid, false);
	}
}

void Camera::setPixelDepth(PixelDepth pixel_depth)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR1(pixel_depth);

	if (getState() != Idle)
		THROW_HW_FATAL(Error) << "Camera is not idle";

	waitState(Idle);
	switch (pixel_depth) {
	case PixelDepth4:
	case PixelDepth8:
		m_image_type = Bpp8;	break;
	case PixelDepth16:
		m_image_type = Bpp16;	break;
	case PixelDepth32:
		m_image_type = Bpp32;	break;
	default:
		THROW_HW_ERROR(InvalidValue) << DEB_VAR1(pixel_depth);
	}
	m_det->setDynamicRange(pixel_depth);
	m_pixel_depth = pixel_depth;

	if (m_model) {
		updateImageSize();
		updateTimeRanges();
		updateCPUAffinity(true);
	}
}

void Camera::getPixelDepth(PixelDepth& pixel_depth)
{
	DEB_MEMBER_FUNCT();
	pixel_depth = m_pixel_depth; 
	DEB_RETURN() << DEB_VAR1(pixel_depth);
}

void Camera::setRawMode(bool raw_mode)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR1(raw_mode);

	if (raw_mode == m_raw_mode)
		return;
	m_raw_mode = raw_mode;

	updateImageSize();
}

void Camera::getRawMode(bool& raw_mode)
{
	DEB_MEMBER_FUNCT();
	raw_mode = m_raw_mode; 
	DEB_RETURN() << DEB_VAR1(raw_mode);
}

State Camera::getState()
{
	DEB_MEMBER_FUNCT();
	AutoMutex l = lock();
	State state = getEffectiveState();
	DEB_RETURN() << DEB_VAR1(state);
	return state;
}

State Camera::getEffectiveState()
{
	if (m_state == Stopped) {
		m_acq_thread = NULL;
		m_state = Idle;
	}
	return m_state;
}

void Camera::waitState(State state)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR1(state);
	AutoMutex l = lock();
	while (getEffectiveState() != state)
		m_cond.wait();
}

State Camera::waitNotState(State state)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR1(state);
	AutoMutex l = lock();
	while (getEffectiveState() == state)
		m_cond.wait();
	state = getEffectiveState();
	DEB_RETURN() << DEB_VAR1(state);
	return state;
}

void Camera::prepareAcq()
{
	DEB_MEMBER_FUNCT();

	if (!m_buffer_cb_mgr)
		THROW_HW_ERROR(Error) << "No BufferCbMgr defined";
	if (!m_model)
		THROW_HW_ERROR(Error) << "No BufferCbMgr defined";

	waitNotState(Stopping);
	if (getState() != Idle)
		THROW_HW_ERROR(Error) << "Camera is not idle";

	bool need_period = !m_nb_frames || (m_nb_frames > 1);
	need_period &= ((m_trig_mode == Defs::Auto) || 
			(m_trig_mode == Defs::BurstTrigger));
	if (need_period && (m_lat_time > 0))
		setFramePeriod(m_exp_time + m_lat_time);

	int nb_buffers;
	m_buffer_cb_mgr->getNbBuffers(nb_buffers);
	int nb_ports = getTotNbPorts();

	{
		AutoMutex l = lock();
		m_frame_map.setBufferSize(nb_buffers);
		m_frame_map.clear();
		m_prev_ifa.clear();
		for (int i = 0; i < nb_ports; ++i) {
			m_port_stats[i].reset();
			m_buffer_thread[i].prepareAcq();
		}
	}

	m_model->prepareAcq();
	m_global_cpu_affinity_mgr.prepareAcq();

	// recv->resetAcquisitionCount()
	m_det->resetFramesCaught();
	m_det->enableWriteToFile(0);
}

void Camera::startAcq()
{
	DEB_MEMBER_FUNCT();

	AutoMutex l = lock();
	if (m_acq_thread)
		THROW_HW_ERROR(Error) << "Must call prepareAcq first";

	m_buffer_cb_mgr->setStartTimestamp(Timestamp::now());

	m_acq_thread = new AcqThread(this);
	m_acq_thread->start();
}

void Camera::stopAcq()
{
	DEB_MEMBER_FUNCT();

	m_global_cpu_affinity_mgr.stopAcq();

	AutoMutex l = lock();
	if (getEffectiveState() != Running)
		return;

	m_acq_thread->stop(true);
	if (getEffectiveState() != Idle)
		THROW_HW_ERROR(Error) << "Camera not Idle";
}

void Camera::processRecvFileStart(int recv_idx, uint32_t dsize)
{
	DEB_MEMBER_FUNCT();
	for (int i = 0; i < m_model->getRecvPorts(); ++i) {
		int port_idx = getPortIndex(recv_idx, i);
		m_model->processRecvFileStart(port_idx, dsize);
	}
}

void Camera::processRecvPort(int port_idx, FrameType frame, char *dptr, 
			     uint32_t dsize)
{
	DEB_MEMBER_FUNCT();

	FrameMap::Item& frame_map_item = m_frame_map.getItem(port_idx);
	frame_map_item.checkFinishedFrame(frame);
	bool valid = (dptr != NULL);
	if (valid) {
		char *bptr = getFrameBufferPtr(frame);
		m_model->processRecvPort(port_idx, frame, dptr, dsize, bptr);
	}
	Timestamp t0 = Timestamp::now();
	frame_map_item.frameFinished(frame, true, valid);
	Timestamp t1 = Timestamp::now();
	PortStats& port_stats = m_port_stats[port_idx];
	port_stats.stats.new_finish.add(t1 - t0);
}

bool Camera::checkLostPackets()
{
	DEB_MEMBER_FUNCT();

	FrameArray ifa = m_frame_map.getItemFrameArray();
	if (ifa != m_prev_ifa) {
		m_prev_ifa = ifa;
		return false;
	}

	FrameType last_frame = getLatestFrame(ifa);
	if (getOldestFrame(ifa) == last_frame) {
		DEB_RETURN() << DEB_VAR1(false);
		return false;
	}

	if (!m_tol_lost_packets) {
		ostringstream err_msg;
		err_msg << "checkLostPackets: frame_map=" << m_frame_map;
		Event::Code err_code = Event::CamOverrun;
		Event *event = new Event(Hardware, Event::Error, Event::Camera, 
					 err_code, err_msg.str());
		DEB_EVENT(*event) << DEB_VAR1(*event);
		reportEvent(event);
		DEB_RETURN() << DEB_VAR1(true);
		return true;
	}

	int nb_ports = getTotNbPorts();
	IntList first_bad(nb_ports);
	if (DEB_CHECK_ANY(DebTypeWarning)) {
		for (int i = 0; i < nb_ports; ++i) 
			first_bad[i] = m_buffer_thread[i].getNbBadFrames();
	}
	for (int i = 0; i < nb_ports; ++i) {
		if (ifa[i] != last_frame)
			processRecvPort(i, last_frame, NULL, 0);
	}
	if (DEB_CHECK_ANY(DebTypeWarning)) {
		IntList last_bad(nb_ports);
		for (int i = 0; i < nb_ports; ++i)
			last_bad[i] = m_buffer_thread[i].getNbBadFrames();
		IntList bfl;
		getSortedBadFrameList(first_bad, last_bad, bfl);
		DEB_WARNING() << "bad_frames=" << bfl.size() << ": " 
			      << PrettyIntList(bfl);
	}

	DEB_RETURN() << DEB_VAR1(false);
	return false;
}

FrameType Camera::getLastReceivedFrame()
{
	DEB_MEMBER_FUNCT();
	FrameType last_frame = m_frame_map.getLastItemFrame();
	DEB_RETURN() << DEB_VAR1(last_frame);
	return last_frame;
}

int Camera::getFramesCaught()
{
	DEB_MEMBER_FUNCT();
	// recv->getTotalFramesCaught()
	int frames_caught = m_det->getFramesCaughtByReceiver();
	DEB_RETURN() << DEB_VAR1(frames_caught);
	return frames_caught;
}

Camera::DetStatus Camera::getDetStatus()
{
	DEB_MEMBER_FUNCT();
	DetStatus status = DetStatus(m_det->getRunStatus());
	DEB_RETURN() << DEB_VAR1(status);
	return status;
}

void Camera::setDAC(int sub_mod_idx, DACIndex dac_idx, int val, bool milli_volt)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR4(sub_mod_idx, dac_idx, val, milli_volt);

	if ((sub_mod_idx < -1) || (sub_mod_idx >= getNbDetSubModules()))
		THROW_HW_ERROR(InvalidValue) << DEB_VAR1(sub_mod_idx);

	typedef slsDetectorDefs::dacIndex SlsDACIndex;
	SlsDACIndex idx = static_cast<SlsDACIndex>(dac_idx);
	dacs_t ret = m_det->setDAC(val, idx, milli_volt, sub_mod_idx);
	if (ret == MultiSlsDetectorErr)
		THROW_HW_ERROR(Error) << "Error setting DAC " << dac_idx 
				      << " on (sub)module " << sub_mod_idx;
}

void Camera::getDAC(int sub_mod_idx, DACIndex dac_idx, int& val, bool milli_volt)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR3(sub_mod_idx, dac_idx, milli_volt);

	if ((sub_mod_idx < 0) || (sub_mod_idx >= getNbDetSubModules()))
		THROW_HW_ERROR(InvalidValue) << DEB_VAR1(sub_mod_idx);

	typedef slsDetectorDefs::dacIndex SlsDACIndex;
	SlsDACIndex idx = static_cast<SlsDACIndex>(dac_idx);
	dacs_t ret = m_det->setDAC(-1, idx, milli_volt, sub_mod_idx);
	if (ret == MultiSlsDetectorErr)
		THROW_HW_ERROR(Error) << "Error getting DAC " << dac_idx 
				      << " on (sub)module " << sub_mod_idx;
	val = ret;
	DEB_RETURN() << DEB_VAR1(val);
}

void Camera::getDACList(DACIndex dac_idx, IntList& val_list, bool milli_volt)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR2(dac_idx, milli_volt);

	int nb_sub_modules = getNbDetSubModules();
	val_list.resize(nb_sub_modules);
	for (int i = 0; i < nb_sub_modules; ++i)
		getDAC(i, dac_idx, val_list[i], milli_volt);
}

void Camera::getADC(int sub_mod_idx, ADCIndex adc_idx, int& val)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR2(sub_mod_idx, adc_idx);

	if ((sub_mod_idx < 0) || (sub_mod_idx >= getNbDetSubModules()))
		THROW_HW_ERROR(InvalidValue) << DEB_VAR1(sub_mod_idx);

	typedef slsDetectorDefs::dacIndex SlsDACIndex;
	SlsDACIndex idx = static_cast<SlsDACIndex>(adc_idx);
	dacs_t ret = m_det->getADC(idx, sub_mod_idx);
	if (ret == MultiSlsDetectorErr)
		THROW_HW_ERROR(Error) << "Error getting ADC " << adc_idx 
				      << " on (sub)module " << sub_mod_idx;
	val = ret;
	DEB_RETURN() << DEB_VAR1(val);
}

void Camera::getADCList(ADCIndex adc_idx, IntList& val_list)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR1(adc_idx);

	int nb_sub_modules = getNbDetSubModules();
	val_list.resize(nb_sub_modules);
	for (int i = 0; i < nb_sub_modules; ++i)
		getADC(i, adc_idx, val_list[i]);
}

void Camera::setAllTrimBits(int sub_mod_idx, int val)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR2(sub_mod_idx, val);

	if ((sub_mod_idx < -1) || (sub_mod_idx >= getNbDetSubModules()))
		THROW_HW_ERROR(InvalidValue) << DEB_VAR1(sub_mod_idx);

	int ret = m_det->setAllTrimbits(val, sub_mod_idx);
	if (ret == MultiSlsDetectorErr)
		THROW_HW_ERROR(Error) << "Error setting all trim bits"
				      << " on (sub)module " << sub_mod_idx;
}

void Camera::getAllTrimBits(int sub_mod_idx, int& val)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR1(sub_mod_idx);

	if ((sub_mod_idx < 0) || (sub_mod_idx >= getNbDetSubModules()))
		THROW_HW_ERROR(InvalidValue) << DEB_VAR1(sub_mod_idx);

	int ret = m_det->setAllTrimbits(-1, sub_mod_idx);
	if (ret == MultiSlsDetectorErr)
		THROW_HW_ERROR(Error) << "Error getting all trim bits"
				      << " on (sub)module " << sub_mod_idx;
	val = ret;
	DEB_RETURN() << DEB_VAR1(val);
}

void Camera::getAllTrimBitsList(IntList& val_list)
{
	DEB_MEMBER_FUNCT();
	int nb_sub_modules = getNbDetSubModules();
	val_list.resize(nb_sub_modules);
	for (int i = 0; i < nb_sub_modules; ++i)
		getAllTrimBits(i, val_list[i]);
}

void Camera::setSettings(Settings settings)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR1(settings);
	if (m_model) {
		if (!m_model->checkSettings(settings))
			THROW_HW_ERROR(InvalidValue) << DEB_VAR1(settings);
		typedef slsDetectorDefs::detectorSettings  DetSettings;
		DetSettings cam_settings = DetSettings(settings);
		m_det->setSettings(cam_settings);
	}
	m_settings = settings;
}

void Camera::getSettings(Settings& settings)
{
	DEB_MEMBER_FUNCT();
	settings = m_settings;
	DEB_RETURN() << DEB_VAR1(settings);
}

void Camera::setThresholdEnergy(int thres)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR1(thres);
	m_det->setThresholdEnergy(thres);
}

void Camera::getThresholdEnergy(int& thres)
{
	DEB_MEMBER_FUNCT();
	thres = m_det->getThresholdEnergy();
	DEB_RETURN() << DEB_VAR1(thres);
}

void Camera::setClockDiv(ClockDiv clock_div)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR1(clock_div);
	m_det->setSpeed(slsDetectorDefs::CLOCK_DIVIDER, clock_div);
	if (m_model)
		updateTimeRanges();
}

void Camera::getClockDiv(ClockDiv& clock_div)
{
	DEB_MEMBER_FUNCT();
	int ret = m_det->setSpeed(slsDetectorDefs::CLOCK_DIVIDER, -1);
	if (ret == MultiSlsDetectorErr)
		THROW_HW_ERROR(Error) << "Error getting clock divider";
	clock_div = ClockDiv(ret);
	DEB_RETURN() << DEB_VAR1(clock_div);
}

void Camera::setReadoutFlags(ReadoutFlags flags)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR1(flags);

	if (!m_model)
		return;

	IntList flags_list;
	if (!m_model->checkReadoutFlags(flags, flags_list))
		THROW_HW_ERROR(InvalidValue) << DEB_VAR1(flags);

	IntList::const_iterator it, end = flags_list.end();
	for (it = flags_list.begin(); it != end; ++it) {
		typedef slsDetectorDefs::readOutFlags DetFlags;
		DetFlags det_flags = static_cast<DetFlags>(*it);
		m_det->setReadOutFlags(det_flags);
	}

	updateTimeRanges();
}

void Camera::getReadoutFlags(ReadoutFlags& flags)
{
	DEB_MEMBER_FUNCT();
	typedef slsDetectorDefs::readOutFlags DetFlags;
	DetFlags det_flags = static_cast<DetFlags>(-1);
	int ret = m_det->setReadOutFlags(det_flags);
	if (ret == MultiSlsDetectorErr)
		THROW_HW_ERROR(Error) << "Error getting flags";
	flags = ReadoutFlags(ret);
	DEB_RETURN() << DEB_VAR1(flags);
}

void Camera::addValidReadoutFlags(DebObj *deb_ptr, ReadoutFlags flags, 
				  IntList& flag_list, NameList& flag_name_list)
{
	DEB_FROM_PTR(deb_ptr);
	ostringstream os;
	os << flags;
	DEB_RETURN() << DEB_VAR2(flags, os.str());
	flag_list.push_back(flags);
	flag_name_list.push_back(os.str());
}

void Camera::getValidReadoutFlags(IntList& flag_list, NameList& flag_name_list)
{
	DEB_MEMBER_FUNCT();
	flag_list.clear();
	flag_name_list.clear();

	if (!m_model)
		return;

	IntList aux_list;
	ReadoutFlags flags = Defs::Normal;
	if (m_model->checkReadoutFlags(flags, aux_list, true))
		addValidReadoutFlags(DEB_PTR(), flags, flag_list, 
				     flag_name_list);

	ReadoutFlags flag_mask = m_model->getReadoutFlagsMask();
	IntList det_flags;
	const unsigned int nb_bits = sizeof(ReadoutFlags) * 8;
	for (unsigned int i = 0; i < nb_bits; ++i) {
		int flags = (1 << i);
		if (flag_mask & flags)
			det_flags.push_back(flags);
	}

	int max_flags = (1 << det_flags.size());
	for (int n = 0; n < max_flags; ++n) {
		flags = ReadoutFlags(0);
		for (unsigned int i = 0; i < nb_bits; ++i) {
			if (n & (1 << i))
				flags = ReadoutFlags(flags | det_flags[i]);
		}
		if (m_model->checkReadoutFlags(flags, aux_list, true))
			addValidReadoutFlags(DEB_PTR(), flags, flag_list, 
					     flag_name_list);
	}
}

void Camera::setNetworkParameter(NetworkParameter net_param, string& val)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR2(net_param, val);
	typedef slsDetectorDefs::networkParameter NetParam;
	NetParam param = static_cast<NetParam>(net_param);
	string res = m_det->setNetworkParameter(param, val);
	val = res;
	DEB_RETURN() << DEB_VAR1(val);
}

void Camera::getNetworkParameter(NetworkParameter net_param, string& val)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR1(net_param);
	typedef slsDetectorDefs::networkParameter NetParam;
	NetParam param = static_cast<NetParam>(net_param);
	val = m_det->getNetworkParameter(param);
	DEB_RETURN() << DEB_VAR1(val);
}

void Camera::setTolerateLostPackets(bool tol_lost_packets)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR1(tol_lost_packets);
	m_tol_lost_packets = tol_lost_packets;
}

void Camera::getTolerateLostPackets(bool& tol_lost_packets)
{
	DEB_MEMBER_FUNCT();
	tol_lost_packets = m_tol_lost_packets;
	DEB_RETURN() << DEB_VAR1(tol_lost_packets);
}

int Camera::getNbBadFrames(int port_idx)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR1(port_idx);
	if ((port_idx < -1) || (port_idx >= getTotNbPorts()))
		THROW_HW_ERROR(InvalidValue) << DEB_VAR1(port_idx);
	int nb_bad_frames;
	if (port_idx == -1) {
		IntList bfl;
		getBadFrameList(port_idx, bfl);
		nb_bad_frames = bfl.size();
	} else {
		nb_bad_frames = m_buffer_thread[port_idx].getNbBadFrames();
	}
	DEB_RETURN() << DEB_VAR1(nb_bad_frames);
	return nb_bad_frames;
}

void Camera::getSortedBadFrameList(IntList first_idx, IntList last_idx,
				   IntList& bad_frame_list)
{
	int nb_ports = getTotNbPorts();
	bool all = first_idx.empty();
	IntList bfl;
	BufferThread *buffer_thread = m_buffer_thread;
	for (int i = 0; i < nb_ports; ++i, ++buffer_thread) {
		int first = all ? 0 : first_idx[i];
		int last = all ? buffer_thread->getNbBadFrames() : last_idx[i];
		IntList l;
		buffer_thread->getBadFrameList(first, last, l);
		bfl.insert(bfl.end(), l.begin(), l.end());
	}
	IntList::iterator first = bfl.begin();
	IntList::iterator last = bfl.end();
	sort(first, last);
	bad_frame_list.resize(last - first);
	IntList::iterator bfl_end, bfl_begin = bad_frame_list.begin();
	bfl_end = unique_copy(first, last, bfl_begin);
	bad_frame_list.resize(bfl_end - bfl_begin);
}

void Camera::getBadFrameList(int port_idx, int first_idx, int last_idx, 
			     IntList& bad_frame_list)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR3(port_idx, first_idx, last_idx);
	if ((port_idx < 0) || (port_idx >= getTotNbPorts()))
		THROW_HW_ERROR(InvalidValue) << DEB_VAR1(port_idx);

	BufferThread *buffer_thread = &m_buffer_thread[port_idx];
	buffer_thread->getBadFrameList(first_idx, last_idx, bad_frame_list);

	DEB_RETURN() << DEB_VAR1(PrettyIntList(bad_frame_list));
}

void Camera::getBadFrameList(int port_idx, IntList& bad_frame_list)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR1(port_idx);
	if (port_idx == -1)
		getSortedBadFrameList(bad_frame_list);
	else
		getBadFrameList(port_idx, 0, getNbBadFrames(port_idx), 
				bad_frame_list);
}

void Camera::registerTimeRangesChangedCallback(TimeRangesChangedCallback& cb)
{
	DEB_MEMBER_FUNCT();

	if (m_time_ranges_cb)
		THROW_HW_ERROR(InvalidValue) << "a cb is already registered";

	cb.m_cam = this;
	m_time_ranges_cb = &cb;
}

void Camera::unregisterTimeRangesChangedCallback(TimeRangesChangedCallback& cb)
{
	DEB_MEMBER_FUNCT();

	if (&cb != m_time_ranges_cb)
		THROW_HW_ERROR(InvalidValue) << "the cb is not registered";

	m_time_ranges_cb = NULL;
	cb.m_cam = NULL;
}

void Camera::getStats(Stats& stats, int port_idx)
{
	DEB_MEMBER_FUNCT();
	if ((port_idx < -1) || (port_idx >= int(m_port_stats.size())))
		THROW_HW_ERROR(InvalidValue) << DEB_VAR1(port_idx);

	if (port_idx < 0) {
		stats.reset();
		for (unsigned int i = 0; i < m_port_stats.size(); ++i)
			stats += m_port_stats[i].stats;
	} else {
		stats = m_port_stats[port_idx].stats;
	}
	DEB_RETURN() << DEB_VAR1(stats);
}

void Camera::setPixelDepthCPUAffinityMap(PixelDepthCPUAffinityMap aff_map)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR1(aff_map);
	m_cpu_affinity_map = aff_map;
	updateCPUAffinity(false);
}

void Camera::getPixelDepthCPUAffinityMap(PixelDepthCPUAffinityMap& aff_map)
{
	DEB_MEMBER_FUNCT();
	aff_map = m_cpu_affinity_map;
	DEB_RETURN() << DEB_VAR1(aff_map);
}

GlobalCPUAffinityMgr::
ProcessingFinishedEvent *Camera::getProcessingFinishedEvent()
{
	DEB_MEMBER_FUNCT();
	return m_global_cpu_affinity_mgr.getProcessingFinishedEvent();
}
