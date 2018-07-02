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
	DEB_TRACE() << DEB_VAR1(m_state);
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
	State prev_state = m_state;

	m_state = Stopping;
	DEB_TRACE() << DEB_VAR2(prev_state, m_state);
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
	DEB_TRACE() << DEB_VAR1(m_state);
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

	State prev_state = m_state;
	if ((m_state == Running) || (m_state == StopReq)) {
		m_state = Stopping;
		AutoMutexUnlock u(l);
		stopAcq();
	}

	{
		AutoMutexUnlock u(l);
		ostringstream err_msg;
		err_msg << "AcqThread: exception thrown: "
			<< "m_state=" << prev_state;
		Event::Code err_code = Event::CamFault;
		Event *event = new Event(Hardware, Event::Error, Event::Camera, 
					 err_code, err_msg.str());
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
	slsDetectorUsers *det = m_cam->m_det;
	det->startReceiver();
	DEB_TRACE() << "calling startAcquisition";
	det->startAcquisition();
}

void Camera::AcqThread::stopAcq()
{
	DEB_MEMBER_FUNCT();
	slsDetectorUsers *det = m_cam->m_det;
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
	  m_recv_nb_ports(0),
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

	CPUAffinity::getNbSystemCPUs();

	m_input_data = new AppInputData(config_fname);

	removeSharedMem();
	createReceivers();

	DEB_TRACE() << "Creating the slsDetectorUsers object";
	m_det = new slsDetectorUsers(0);
	DEB_TRACE() << "Reading configuration file";
	const char *fname = m_input_data->config_file_name.c_str();
	m_det->readConfigurationFile(fname);

	m_det->setReceiverSilentMode(1);
	setReceiverFifoDepth(m_recv_fifo_depth);

	m_pixel_depth = PixelDepth(m_det->setBitDepth(-1));

	setSettings(Defs::Standard);
	setTrigMode(Defs::Auto);
	setNbFrames(1);
	setExpTime(0.99);
	setFramePeriod(1.0);

	if (isTenGigabitEthernetEnabled()) {
		DEB_TRACE() << "Forcing 10G Ethernet flow control";
		setFlowControl10G(true);
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

	m_recv_nb_ports = m_model->getRecvPorts();
	int nb_ports = getTotNbPorts();
	m_frame_map.setNbItems(nb_ports);

	RecvList::iterator it, end = m_recv_list.end();
	for (it = m_recv_list.begin(); it != end; ++it)
		(*it)->setNbPorts(m_recv_nb_ports);

	setPixelDepth(m_pixel_depth);
	setSettings(m_settings);
}

Camera::RecvPortList Camera::getRecvPortList()
{
	DEB_MEMBER_FUNCT();
	RecvPortList port_list;
	RecvList::iterator it, end = m_recv_list.end();
	for (it = m_recv_list.begin(); it != end; ++it)
		port_list.insert(port_list.end(), (*it)->m_port_list.begin(),
						  (*it)->m_port_list.end());
	return port_list;
}

Receiver::Port *Camera::getRecvPort(int port_idx)
{
	pair<int, int> recv_port = splitPortIndex(port_idx);
	Receiver *recv = m_recv_list[recv_port.first];
	return recv->m_port_list[recv_port.second];
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
	m_det->putCommand(args.size(), args, idx);
}

string Camera::getCmd(const string& s, int idx)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << "s=\"" << s << "\"";
	Args args(s);
	string r = m_det->getCommand(args.size(), args, idx);
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
	m_det->setTimingMode(mode);
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

	CPUAffinityList::const_iterator lit = recv_affinity.listeners.begin();
	CPUAffinityList::const_iterator wit = recv_affinity.writers.begin();
	RecvList::iterator it, end = m_recv_list.end();
	for (it = m_recv_list.begin(); it != end; ++it) {
		Receiver *recv = *it;
		slsReceiverUsers::CPUMaskList list_cpu_mask;
		slsReceiverUsers::CPUMaskList writ_cpu_mask;
		slsReceiverUsers::NodeMaskList fifo_node_mask;
		int max_node;
		string deb_head;
		if (DEB_CHECK_ANY(DebTypeTrace) ||
		    DEB_CHECK_ANY(DebTypeWarning)) {
			ostringstream os;
			os << "setting recv " << recv->m_idx << " ";
			deb_head = os.str();
		}
		for (int i = 0; i < m_recv_nb_ports; ++i, ++lit, ++wit) {
			cpu_set_t cpu_set;
			const CPUAffinity& listener = *lit;
			DEB_TRACE() << deb_head << "listener " << i << " "
				    << "CPU mask to " << listener;
			listener.initCPUSet(cpu_set);
			list_cpu_mask.push_back(cpu_set);

			const CPUAffinity& writer = *wit;
			DEB_TRACE() << deb_head << "writer " << i  << " "
				    << "CPU mask to " << writer;
			writer.initCPUSet(cpu_set);
			writ_cpu_mask.push_back(cpu_set);

			CPUAffinity both = listener | writer;
			vector<unsigned long> mlist;
			both.getNUMANodeMask(mlist, max_node);
			if (mlist.size() != 1)
				THROW_HW_ERROR(Error) << DEB_VAR1(mlist.size());
			unsigned long& nmask = mlist[0];
			DEB_TRACE() << deb_head << "Fifo " << i  << " "
				    << "NUMA node mask mask to "
				    << DEB_HEX(nmask);
			int c = bitset<64>(nmask).count();
			if (c != 1)
				DEB_WARNING() << deb_head << "Fifo " << i << " "
					      << "NUMA node mask has "
					      << c << " nodes";
			fifo_node_mask.push_back(nmask);
		}
		slsReceiverUsers *recv_users = recv->m_recv;
		recv_users->setThreadCPUAffinity(list_cpu_mask, writ_cpu_mask);

		recv_users->setFifoNodeAffinity(fifo_node_mask, max_node);
	}

	CPUAffinityList::const_iterator tit;
	tit = recv_affinity.port_threads.begin();
	RecvPortList port_list = getRecvPortList();
	RecvPortList::iterator pit, pend = port_list.end();
	for (pit = port_list.begin(); pit != pend; ++pit, ++tit) {
		pid_t tid = (*pit)->getThreadID();
		(*tit).applyToTask(tid, false);
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
	m_det->setBitDepth(pixel_depth);
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

	{
		AutoMutex l = lock();
		m_frame_map.setBufferSize(nb_buffers);
		m_frame_map.clear();
		m_prev_ifa.clear();
		RecvList::iterator it, end = m_recv_list.end();
		for (it = m_recv_list.begin(); it != end; ++it)
			(*it)->prepareAcq();
	}

	m_model->prepareAcq();
	m_global_cpu_affinity_mgr.prepareAcq();

	resetFramesCaught();
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

	RecvPortList port_list = getRecvPortList();
	int nb_ports = port_list.size();
	IntList first_bad(nb_ports);
	if (DEB_CHECK_ANY(DebTypeWarning)) {
		for (int i = 0; i < nb_ports; ++i) 
			first_bad[i] = port_list[i]->getNbBadFrames();
	}
	for (int i = 0; i < nb_ports; ++i) {
		if (ifa[i] != last_frame)
			port_list[i]->processFrame(last_frame, NULL, 0);
	}
	if (DEB_CHECK_ANY(DebTypeWarning)) {
		IntList last_bad(nb_ports);
		for (int i = 0; i < nb_ports; ++i)
			last_bad[i] = port_list[i]->getNbBadFrames();
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
	int frames_caught = getNbCmd<int>("framescaught");
	DEB_RETURN() << DEB_VAR1(frames_caught);
	return frames_caught;
}

Camera::DetStatus Camera::getDetStatus()
{
	DEB_MEMBER_FUNCT();
	DetStatus status = DetStatus(m_det->getDetectorStatus());
	DEB_RETURN() << DEB_VAR1(status);
	return status;
}

void Camera::setDAC(int sub_mod_idx, DACIndex dac_idx, int val, bool milli_volt)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR4(sub_mod_idx, dac_idx, val, milli_volt);

	if (milli_volt)
		THROW_HW_ERROR(InvalidValue) << "milli-volt not supported";
	if ((sub_mod_idx < -1) || (sub_mod_idx >= getNbDetSubModules()))
		THROW_HW_ERROR(InvalidValue) << DEB_VAR1(sub_mod_idx);

	Defs::DACCmdMapType::const_iterator it = Defs::DACCmdMap.find(dac_idx);
	if (it == Defs::DACCmdMap.end())
		THROW_HW_ERROR(InvalidValue) << DEB_VAR1(dac_idx);
	const string& dac_cmd = it->second;
	dacs_t ret = m_det->setDAC(dac_cmd, val, sub_mod_idx);
	if (ret == MultiSlsDetectorErr)
		THROW_HW_ERROR(Error) << "Error setting DAC " << dac_idx 
				      << " on (sub)module " << sub_mod_idx;
	else if (ret == SlsDetectorBadIndexErr)
		THROW_HW_ERROR(Error) << "Bad value: " << DEB_VAR1(dac_idx);
}

void Camera::getDAC(int sub_mod_idx, DACIndex dac_idx, int& val, bool milli_volt)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR3(sub_mod_idx, dac_idx, milli_volt);

	if (milli_volt)
		THROW_HW_ERROR(InvalidValue) << "milli-volt not supported";
	if ((sub_mod_idx < 0) || (sub_mod_idx >= getNbDetSubModules()))
		THROW_HW_ERROR(InvalidValue) << DEB_VAR1(sub_mod_idx);

	Defs::DACCmdMapType::const_iterator it = Defs::DACCmdMap.find(dac_idx);
	if (it == Defs::DACCmdMap.end())
		THROW_HW_ERROR(InvalidValue) << DEB_VAR1(dac_idx);
	const string& dac_cmd = it->second;
	dacs_t ret = m_det->setDAC(dac_cmd, -1, sub_mod_idx);
	if (ret == MultiSlsDetectorErr)
		THROW_HW_ERROR(Error) << "Error getting DAC " << dac_idx 
				      << " on (sub)module " << sub_mod_idx;
	else if (ret == SlsDetectorBadIndexErr)
		THROW_HW_ERROR(Error) << "Bad value: " << DEB_VAR1(dac_idx);
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

	Defs::ADCCmdMapType::const_iterator it = Defs::ADCCmdMap.find(adc_idx);
	if (it == Defs::ADCCmdMap.end())
		THROW_HW_ERROR(InvalidValue) << DEB_VAR1(adc_idx);
	const string& adc_cmd = it->second;
	dacs_t ret = m_det->getADC(adc_cmd, sub_mod_idx);
	if (ret == MultiSlsDetectorErr)
		THROW_HW_ERROR(Error) << "Error getting ADC " << adc_idx 
				      << " on (sub)module " << sub_mod_idx;
	else if (ret == SlsDetectorBadIndexErr)
		THROW_HW_ERROR(Error) << "Bad value: " << DEB_VAR1(adc_idx);
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

void Camera::setNetworkParameter(NetworkParameter net_param, string& val)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR2(net_param, val);
	THROW_HW_ERROR(NotSupported) << "Not implemented by slsDetector API";
}

void Camera::getNetworkParameter(NetworkParameter net_param, string& val)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR1(net_param);
	THROW_HW_ERROR(NotSupported) << "Not implemented by slsDetector API";
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
		Receiver::Port *port = getRecvPort(port_idx);
		nb_bad_frames = port->getNbBadFrames();
	}
	DEB_RETURN() << DEB_VAR1(nb_bad_frames);
	return nb_bad_frames;
}

void Camera::getSortedBadFrameList(IntList first_idx, IntList last_idx,
				   IntList& bad_frame_list)
{
	bool all = first_idx.empty();
	IntList bfl;
	RecvPortList port_list = getRecvPortList();
	RecvPortList::iterator it = port_list.begin();
	int nb_ports = port_list.size();
	for (int i = 0; i < nb_ports; ++i, ++it) {
		Receiver::Port *port = *it;
		int first = all ? 0 : first_idx[i];
		int last = all ? port->getNbBadFrames() : last_idx[i];
		IntList l;
		port->getBadFrameList(first, last, l);
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

	Receiver::Port *port = getRecvPort(port_idx);
	port->getBadFrameList(first_idx, last_idx, bad_frame_list);

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
	if ((port_idx < -1) || (port_idx >= getTotNbPorts()))
		THROW_HW_ERROR(InvalidValue) << DEB_VAR1(port_idx);

	if (port_idx < 0) {
		stats.reset();
		RecvPortList port_list = getRecvPortList();
		RecvPortList::iterator it, end = port_list.end();
		for (it = port_list.begin(); it != end; ++it)
			stats += (*it)->getStats().stats;
	} else {
		Receiver::Port *port = getRecvPort(port_idx);
		stats = port->getStats().stats;
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

void Camera::setReceiverFifoDepth(int fifo_depth)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR1(fifo_depth);
	// recv->setFifoDepth()
	putNbCmd<int>("rx_fifodepth", fifo_depth);
}

bool Camera::isTenGigabitEthernetEnabled()
{
	DEB_MEMBER_FUNCT();
	bool enabled = getNbCmd<int>("tengiga");
	DEB_RETURN() << DEB_VAR1(enabled);
	return enabled;
}

void Camera::setFlowControl10G(bool enabled)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR1(enabled);
	putNbCmd<int>("flowcontrol_10g", enabled);
}

void Camera::resetFramesCaught()
{
	DEB_MEMBER_FUNCT();
	// recv->resetAcquisitionCount()
	putCmd("resetframescaught");
}
