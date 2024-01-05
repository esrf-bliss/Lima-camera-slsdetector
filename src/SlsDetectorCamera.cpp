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

#include <limits.h>
#include <algorithm>
#include <numeric>
#include <cmath>
#include <iostream>
#include <fstream>

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

	ifstream config_file(config_file_name);
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
			DEB_TRACE() << DEB_VAR1(host_name_list);
			continue;
		}

		re = "(([0-9]+):)?rx_tcpport";
		if (re.match(s, full_match)) {
			int id = 0;
			if (full_match[2].found()) {
				istringstream is(full_match[2]);
				is >> id;
				if (id < 0)
					THROW_HW_FATAL(InvalidValue) <<
						"Invalid detector id: " << id;
			}
			int rx_tcpport;
			config_file >> rx_tcpport;
			recv_port_map[id] = rx_tcpport;
			continue;
		}

		re = "(([0-9]+):)?rx_hostname";
		if (re.match(s, full_match)) {
			int id = 0;
			if (full_match[2].found()) {
				istringstream is(full_match[2]);
				is >> id;
				if (id < 0)
					THROW_HW_FATAL(InvalidValue) <<
						"Invalid detector id: " << id;
			}
			string host_port;
			config_file >> host_port;
			re = "([^:]+):([0-9]+)\\+?";
			if (re.match(host_port, full_match)) {
				istringstream is(full_match[2]);
				int rx_tcpport;
				is >> rx_tcpport;
				recv_port_map[id] = rx_tcpport;
			}
			continue;
		}
	}
}

Camera::AcqThread::ExceptionCleanUp::ExceptionCleanUp(AcqThread& thread,
						      AutoMutex& l)
	: Thread::ExceptionCleanUp(thread), m_lock(l), m_finished(false)
{
	DEB_CONSTRUCTOR();
}

Camera::AcqThread::ExceptionCleanUp::~ExceptionCleanUp()
{
	DEB_DESTRUCTOR();
	if (m_finished)
		return;
	AcqThread *thread = static_cast<AcqThread *>(&m_thread);
	thread->onError(m_lock);
}

void Camera::AcqThread::ExceptionCleanUp::threadFinished()
{
	DEB_MEMBER_FUNCT();
	m_finished = true;
}

Camera::AcqThread::AcqThread(Camera *cam)
	: m_cam(cam), m_cond(m_cam->m_cond), m_state(m_cam->m_state),
	  m_stop_state(NoStop)
{
	DEB_CONSTRUCTOR();
}

void Camera::AcqThread::start(AutoMutex& l)
{
	DEB_MEMBER_FUNCT();

	CheckLockedAutoMutex(m_cond, l, DEB_PTR());

	m_state = Starting;
	Thread::start();

	if (false) {
		struct sched_param param;
		param.sched_priority = sched_get_priority_min(SCHED_RR);
		int ret = pthread_setschedparam(m_thread, SCHED_RR, &param);
		if (ret != 0)
			DEB_ERROR() << "Could not set AcqThread real-time priority!!";
	}

	while (m_state == Starting)
		m_cond.wait();

	const CPUAffinity& acq_affinity = m_cam->m_acq_thread_cpu_affinity;
	if (!acq_affinity.isDefault()) {
		DEB_ALWAYS() << "Setting CPUAffinity for AcqThread to "
			     << acq_affinity;
		acq_affinity.applyToTask(getThreadID(), false, false);
	}
}

void Camera::AcqThread::stop(AutoMutex& l, bool wait)
{
	DEB_MEMBER_FUNCT();

	CheckLockedAutoMutex(m_cond, l, DEB_PTR());

	m_state = StopReq;
	m_cond.broadcast();

	{
		AutoMutexUnlock u(l);
		stopAcq();
	}

	while (wait && (m_state != Stopped) && (m_state != Idle))
		m_cond.wait();
}

inline Camera::AcqThread::Status
Camera::AcqThread::newFrameReady(DetFrameImagePackets&& packets)
{
	DEB_MEMBER_FUNCT();
	HwFrameInfoType frame_info;
	FrameType frame = packets.first;
	frame_info.acq_frame_nb = frame;
	static const std::string key = "packet_data";
	HwAddData(key, frame_info,
		  std::make_shared<PacketData>(std::move(packets)));
	DEB_TRACE() << DEB_VAR1(frame_info);
	StdBufferCbMgr *cb_mgr = m_cam->m_buffer.getBufferCbMgr();
	bool cont_acq = cb_mgr->newFrameReady(frame_info);
	bool acq_end = (frame == m_cam->m_lima_nb_frames - 1);
	cont_acq &= !acq_end;
	return Status(cont_acq, acq_end);
}

void Camera::AcqThread::threadFunction()
{
	DEB_MEMBER_FUNCT();

	AutoMutex l = m_cam->lock();

	// cleanup is executed with lock, once state goes to Stopped
	// thread will be deleted in getEffectiveState without releasing lock
	ExceptionCleanUp cleanup(*this, l);

	class Acq
	{
		DEB_CLASS_NAMESPC(DebModCamera, "Camera::AcqThread::Acq", 
				  "SlsDetector");
	public:
		Acq(AcqThread& thread, AutoMutex& l)
			: m_thread(thread), m_state(m_thread.m_state),
			  m_cond(m_thread.m_cond), m_lock(l)
		{
			DEB_CONSTRUCTOR();
			CheckLockedAutoMutex(m_cond, m_lock, DEB_PTR());
			{
				AutoMutexUnlock u(m_lock);
				m_thread.startAcq();
			}
			m_state = Running;
			DEB_TRACE() << DEB_VAR1(m_state);
			m_cond.broadcast();
		}

		~Acq()
		{
			DEB_DESTRUCTOR();
			DEB_TRACE() << DEB_VAR1(m_state);
			m_state = Stopping;

			typedef StateCleanUpHelper<AcqState> StateCleanUp;
			StateCleanUp state_cleanup(m_state, Stopped, m_cond,
						   m_lock, DEB_PTR());

			AutoMutexUnlock u(m_lock);
			m_thread.stopAcq();

			SlsDetector::Stats stats;
			m_thread.m_cam->getStats(stats);
			DEB_ALWAYS() << DEB_VAR1(stats);
		}

		bool stopReq() { return (m_state == StopReq); }

	private:
		AcqThread& m_thread;
		AcqState& m_state;
		Cond& m_cond;
		AutoMutex& m_lock;
	};

	class AffinityMgrProxy
	{
	public:
		AffinityMgrProxy(GlobalCPUAffinityMgr& mgr, AutoMutex& l)
			: m_mgr(mgr), m_lock(l), m_recv_finished(false)
		{
			AutoMutexUnlock u(m_lock);
			m_mgr.startAcq();
		}

		~AffinityMgrProxy()
		{
			AutoMutexUnlock u(m_lock);
			if (m_recv_finished)
				m_mgr.waitLimaFinished();
			else
				m_mgr.cleanUp();
		}

		bool needCheckRecv()
		{
			return !m_recv_finished;
		}

		void recvFinished()
		{
			AutoMutexUnlock u(m_lock);
			m_mgr.recvFinished();
			m_recv_finished = true;
		}

	private:
		GlobalCPUAffinityMgr& m_mgr;
		AutoMutex& m_lock;
		bool m_recv_finished;
	};

	Acq acq(*this, l);

	AffinityMgrProxy affinity_mgr(m_cam->m_global_cpu_affinity_mgr, l);

	Reconstruction *reconstruct = m_cam->m_model->getReconstruction();

	SeqFilter seq_filter;
	bool cont_acq = true;
	bool acq_end = false;
	FrameType next_frame = 0;
	FrameType acq_nb_frames = m_cam->m_lima_nb_frames;

	const double check_recv_finished_time = 1.0;
	Timestamp last_check_ts = Timestamp::now();

	auto need_check_recv = [&] { return affinity_mgr.needCheckRecv(); };
	auto check_recv_finished = [&] {
		int last_frame;
		{
			AutoMutexUnlock u(l);
			last_frame = m_cam->getLastFrameCaught();
		}
		if (last_frame == acq_nb_frames - 1)
			affinity_mgr.recvFinished();
	};

	while (!acq.stopReq() && cont_acq && (next_frame < acq_nb_frames)) {
		Timestamp t0 = Timestamp::now();
		FrameType frame = next_frame++;
		DetFrameImagePackets packets;
		{
			AutoMutexUnlock u(l);
			packets = std::move(readRecvPackets(frame));
		}
		Timestamp t1 = Timestamp::now();
		m_stats.read_packets.add(t1 - t0);
		if (packets.second.empty() || acq.stopReq())
			break;
		else if (packets.first != frame)
			THROW_HW_ERROR(Error) << "Invalid packets frame";
		while (!acq.stopReq()) {
			AutoMutexUnlock u(l);
			if (m_cam->m_buffer.waitFrame(frame))
				break;
		}
		Timestamp t2 = Timestamp::now();
		m_stats.wait_frame.add(t2 - t1);
		if (acq.stopReq())
			break;
		{
			AutoMutexUnlock u(l);
			Status status = newFrameReady(std::move(packets));
			cont_acq = status.first;
			acq_end = status.second;
		}
		Timestamp t3 = Timestamp::now();
		m_stats.new_frame.add(t3 - t2);
		if (need_check_recv() &&
		    (t3 - last_check_ts > check_recv_finished_time)) {
			check_recv_finished();
			last_check_ts = Timestamp::now();
			m_stats.check_recv.add(Timestamp::now() - t3);
		}
	}

	check_recv_finished();

	if (acq_end && m_cam->m_skip_frame_freq) {
		AutoMutexUnlock u(l);
		m_cam->waitLastSkippedFrame();
	}

	cleanup.threadFinished();

	DEB_ALWAYS() << DEB_VAR1(m_stats.read_packets);
	DEB_ALWAYS() << DEB_VAR1(m_stats.wait_frame);
	DEB_ALWAYS() << DEB_VAR1(m_stats.new_frame);
	DEB_ALWAYS() << DEB_VAR1(m_stats.check_recv);
}

DetFrameImagePackets Camera::AcqThread::readRecvPackets(FrameType frame)
{
	DEB_MEMBER_FUNCT();

	DetFrameImagePackets det_frame_packets{frame, {}};
	DetImagePackets& det_packets = det_frame_packets.second;

	auto stopped = [&]() { return (m_cam->getAcqState() == StopReq); };

	int nb_recvs = m_cam->getNbRecvs();
	for (int i = 0; i < nb_recvs; ++i) {
		if (stopped())
			return {};
		AutoPtr<Receiver::ImagePackets> image_packets;
		Receiver *recv = m_cam->m_recv_list[i];
		image_packets = recv->readImagePackets();
		if (stopped())
			return {};
		else if (!image_packets)
			THROW_HW_ERROR(Error) << "Recv. " << i << ": "
					      << "missing frame " << frame;
		FrameType f = image_packets->frame;
		if (f != frame)
			THROW_HW_ERROR(Error) << "Recv. " << i << ": "
					      << "unexpected frame " << f
					      << " instead of " << frame;
		typedef DetImagePackets::value_type MapEntry;
		det_packets.emplace(MapEntry(i, image_packets));
	}

	return det_frame_packets;
}

void Camera::AcqThread::onError(AutoMutex& l)
{
	DEB_MEMBER_FUNCT();

	CheckLockedAutoMutex(m_cond, l, DEB_PTR());

	if ((m_state != Stopped) && (m_state != Idle)) {
		DEB_ERROR() << DEB_VAR1(m_state);
		m_state = Stopped;
	}

	AutoMutexUnlock u(l);
	ostringstream err_msg;
	err_msg << "AcqThread: exception thrown!";
	Event::Code err_code = Event::CamFault;
	Event *event = new Event(Hardware, Event::Error, Event::Camera, 
				 err_code, err_msg.str());
	DEB_EVENT(*event) << DEB_VAR1(*event);
	m_cam->reportEvent(event);
}

void Camera::AcqThread::startAcq()
{
	DEB_MEMBER_FUNCT();
	sls::Detector *det = m_cam->m_det;
	DEB_TRACE() << "calling startReceiver";
	det->startReceiver();
	DEB_TRACE() << "calling Model::startAcq";
	m_cam->m_model->startAcq();
	DEB_TRACE() << "calling startDetector";
	det->startDetector();
}

void Camera::AcqThread::stopAcq()
{
	DEB_MEMBER_FUNCT();

	class Sync
	{
	public:
		Sync(StopState& stop_state, Cond& cond, DebObj *deb_ptr)
			: m_stop_state(stop_state), m_lock(cond.mutex()),
			  m_was_not_stopped(m_stop_state == NoStop),
			  m_cleanup(m_stop_state, StopFinished, cond, m_lock,
				    deb_ptr)
		{
			if (m_was_not_stopped) {
				m_stop_state = StopRunning;
			} else {
				while (m_stop_state != StopFinished)
					cond.wait();
			}
			m_unlock = new AutoMutexUnlock(m_lock);
		}

		bool stopDone() { return !m_was_not_stopped; }

	private:
		typedef StateCleanUpHelper<StopState> StateCleanUp;

		StopState& m_stop_state;
		AutoMutex m_lock;
		bool m_was_not_stopped;
		StateCleanUp m_cleanup;
		AutoPtr<AutoMutexUnlock> m_unlock;
	};

	Sync sync(m_stop_state, m_cond, DEB_PTR());
	if (sync.stopDone())
		return;

	sls::Detector *det = m_cam->m_det;
	DetStatus det_status = m_cam->getDetStatus();
	bool xfer_active = m_cam->m_model->isXferActive();
	DEB_ALWAYS() << DEB_VAR2(det_status, xfer_active);
	if ((det_status == Defs::Running) || (det_status == Defs::Waiting) ||
	    xfer_active) {
		DEB_TRACE() << "calling stopDetector";
		det->stopDetector();
		Timestamp t0 = Timestamp::now();
		while (m_cam->m_model->isAcqActive())
			Sleep(m_cam->m_abort_sleep_time);
		double milli_sec = (Timestamp::now() - t0) * 1e3;
		DEB_TRACE() << "Abort -> Idle: " << DEB_VAR1(milli_sec);
	}
	DEB_TRACE() << "calling stopReceiver";
	det->stopReceiver();
	DEB_TRACE() << "calling Model::stopAcq";
	m_cam->m_model->stopAcq();
}

Camera::Camera(string config_fname, int det_id) 
	: m_det_id(det_id),
	  m_model(NULL),
	  m_lima_nb_frames(1),
	  m_det_nb_frames(1),
	  m_skip_frame_freq(0),
	  m_last_skipped_frame_timeout(0.5),
	  m_lat_time(0),
	  m_buffer(this),
	  m_pixel_depth(PixelDepth16), 
	  m_raw_mode(false),
	  m_state(Idle),
	  m_new_frame_timeout(0.5),
	  m_abort_sleep_time(0.1),
	  m_tol_lost_packets(false),
	  m_time_ranges_cb(NULL),
	  m_global_cpu_affinity_mgr(this)
{
	DEB_CONSTRUCTOR();
	DEB_PARAM() << DEB_VAR1(m_det_id);

	CPUAffinity::getNbSystemCPUs();

	m_input_data = new AppInputData(config_fname);

	createReceivers();

	DEB_TRACE() << "Creating the sls::Detector object";
	m_det = new sls::Detector(m_det_id);
	try {
		// This should fail if the detector is not idle
		m_det->setHostname(m_input_data->host_name_list);
	} catch (...) {
		checkDetIdleStatus();
	}

	DEB_TRACE() << "Reading configuration file";
	const char *fname = m_input_data->config_file_name.c_str();
	EXC_CHECK(m_det->loadConfig(fname));

	EXC_CHECK(m_det->setRxSilentMode(1));

	sls::Result<int> dr_res;
	EXC_CHECK(dr_res = m_det->getDynamicRange());
	m_pixel_depth = PixelDepth(dr_res.squash(-1));

	setTrigMode(Defs::Auto);
	setNbFrames(1);
	setExpTime(0.99);
	setFramePeriod(1.0);
	setTolerateLostPackets(true);
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
	slsDetectorDefs::detectorType sls_type;
	const char *err_msg = "Detector types are different";
	EXC_CHECK(sls_type = m_det->getDetectorType().tsquash(err_msg));
	Type det_type;
	switch (sls_type) {
	case slsDetectorDefs::GENERIC:
		det_type = GenericDet;
		break;
	case slsDetectorDefs::EIGER:
		det_type = EigerDet;
		break;
	case slsDetectorDefs::JUNGFRAU:
		det_type = JungfrauDet;
		break;
	default:
		det_type = UnknownDet;
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

	int nb_udp_ifaces;
	m_model->getNbUDPInterfaces(nb_udp_ifaces);
	DEB_ALWAYS() << "Using " << m_model->getName()
		     << " with " << getNbDetModules() << "x" << nb_udp_ifaces
		     << " UDP interfaces (det_id=" << m_det_id << ")";

	setPixelDepth(m_pixel_depth);
}

void Camera::checkDetIdleStatus()
{
	DEB_MEMBER_FUNCT();

	const NameList& config_host_list = m_input_data->host_name_list;
	NameList shm_host_list;
	if (m_det->size() > 0)
		shm_host_list = m_det->getHostname();
	DEB_TRACE() << DEB_VAR1(config_host_list);
	DEB_TRACE() << DEB_VAR1(shm_host_list);
	if (shm_host_list != config_host_list) {
		DEB_WARNING() << "Cannot check detector status!";
		return;
	}

	DEB_TRACE() << "Checking that the detector is Idle";
	typedef slsDetectorDefs::runStatus RunStatus;
	sls::Result<RunStatus> det_status = m_det->getDetectorStatus();
	RunStatus mod0_status = det_status[0];
	bool stopped = ((mod0_status == Defs::Idle) ||
			(mod0_status == Defs::Stopped));
	if (!det_status.equal() || !stopped) {
		DEB_ALWAYS() << "Detector not (completely) idle: stopping";
		m_det->stopDetector();
	}
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

string Camera::execCmd(const string& s, bool put, int idx)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << "s=\"" << s << "\", " << DEB_VAR2(put, idx);

	string prog_name = string("sls_detector_") + (put ? "put" : "get");
	ostringstream os;
	os << prog_name << " " << m_det_id << '-';
	if (idx >= 0)
		os << idx << ':';
	os << s;
	SystemCmdPipe cmd(os.str(), prog_name, false);
	cmd.setPipe(SystemCmdPipe::StdOut, SystemCmdPipe::DoPipe);
	cmd.setPipe(SystemCmdPipe::StdErr, SystemCmdPipe::DoPipe);
	StringList out, err;
	int ret = cmd.execute(out, err);
	DEB_TRACE() << DEB_VAR3(ret, out, err);
	string err_str = accumulate(err.begin(), err.end(), string());
	if ((ret != 0) || (err_str.find("ERROR") != string::npos))
		THROW_HW_ERROR(Error) << prog_name << "(" << ret << "): "
				      << err_str;
	else if (!err_str.empty())
		cerr << err_str;
	string out_str = accumulate(out.begin(), out.end(), string());
	DEB_RETURN() << DEB_VAR1(out_str);
	return out_str;
}

void Camera::putCmd(const string& s, int idx)
{
	DEB_MEMBER_FUNCT();
	execCmd(s, true, idx);
}

string Camera::getCmd(const string& s, int idx)
{
	DEB_MEMBER_FUNCT();
	string r = execCmd(s, false, idx);
	string::size_type p = s.find(':');
	string raw_s = s.substr((p == string::npos) ? 0 : (p + 1));
	DEB_TRACE() << DEB_VAR2(s, raw_s);
	bool multi_line = ((s == "list") || (s == "versions"));
	if (!multi_line) {
		if (r.find(raw_s + ' ') != 0)
			THROW_HW_ERROR(Error) << "Invalid response: " << r;
		string::size_type e = raw_s.size() + 1;
		p = r.find('\n', e);
		r = r.substr(e, (p == string::npos) ? p : (p - e));
	}
	DEB_RETURN() << DEB_VAR1(r);
	return r;
}

void Camera::setTrigMode(TrigMode trig_mode)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR1(trig_mode);
	waitAcqState(Idle);
	TrigMode cam_trig_mode = trig_mode;
	if ((trig_mode == Defs::SoftTriggerExposure) ||
	    ((trig_mode == Defs::BurstTrigger) && (getType() != EigerDet)))
		cam_trig_mode = Defs::TriggerExposure;
	typedef slsDetectorDefs::timingMode TimingMode;
	TimingMode mode = static_cast<TimingMode>(cam_trig_mode);
	EXC_CHECK(m_det->setTimingMode(mode));
	m_trig_mode = trig_mode;
	setNbFrames(m_lima_nb_frames);
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

	waitAcqState(Idle);
	FrameType det_nb_frames = nb_frames;
	if (m_skip_frame_freq)
		det_nb_frames += nb_frames / m_skip_frame_freq;
	bool trig_exp = ((m_trig_mode == Defs::TriggerExposure) ||
			 (m_trig_mode == Defs::SoftTriggerExposure));
	int cam_frames = trig_exp ? 1 : det_nb_frames;
	int cam_triggers = trig_exp ? det_nb_frames : 1;
	EXC_CHECK(m_det->setNumberOfFrames(cam_frames));
	EXC_CHECK(m_det->setNumberOfTriggers(cam_triggers));
	m_lima_nb_frames = nb_frames;
	m_det_nb_frames = det_nb_frames;
}

void Camera::getNbFrames(FrameType& nb_frames)
{
	DEB_MEMBER_FUNCT();
	nb_frames = m_lima_nb_frames;
	DEB_RETURN() << DEB_VAR1(nb_frames);
}

void Camera::setSkipFrameFreq(FrameType skip_frame_freq)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR1(skip_frame_freq);
	m_skip_frame_freq = skip_frame_freq;
	setNbFrames(m_lima_nb_frames);
}

void Camera::getSkipFrameFreq(FrameType& skip_frame_freq)
{
	DEB_MEMBER_FUNCT();
	skip_frame_freq = m_skip_frame_freq;
	DEB_RETURN() << DEB_VAR1(skip_frame_freq);
}

void Camera::setExpTime(double exp_time)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR1(exp_time);
	waitAcqState(Idle);
	EXC_CHECK(m_det->setExptime(NSec(exp_time)));
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

	waitAcqState(Idle);
	EXC_CHECK(m_det->setPeriod(NSec(frame_period)));
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
	RecvList::iterator it, end = m_recv_list.end();
	for (it = m_recv_list.begin(); it != end; ++it)
		(*it)->setGapPixelsEnable(!m_raw_mode);
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

void Camera::setRecvCPUAffinity(const RecvCPUAffinityList& recv_affinity_list)
{
	DEB_MEMBER_FUNCT();
	unsigned int nb_aff = recv_affinity_list.size();
	DEB_PARAM() << DEB_VAR1(nb_aff);
	RecvCPUAffinityList::const_iterator ait = recv_affinity_list.begin();
	RecvList::iterator rit = m_recv_list.begin();
	for (unsigned int i = 0; i < nb_aff; ++i, ++ait, ++rit)
		(*rit)->setCPUAffinity(*ait);
}

void Camera::setModuleActive(int mod_idx, bool  active)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR2(mod_idx, active);

	if ((mod_idx < 0) || (mod_idx >= getNbDetModules()))
		THROW_HW_ERROR(InvalidValue) << DEB_VAR1(mod_idx);

	Positions pos = Idx2Pos(mod_idx);
	EXC_CHECK(m_det->setActive(active, pos));
}

void Camera::getModuleActive(int mod_idx, bool& active)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR1(mod_idx);

	if ((mod_idx < 0) || (mod_idx >= getNbDetModules()))
		THROW_HW_ERROR(InvalidValue) << DEB_VAR1(mod_idx);

	Positions pos = Idx2Pos(mod_idx);
	EXC_CHECK(active = m_det->getActive(pos).front());
	DEB_RETURN() << DEB_VAR1(active);
}

void Camera::setPixelDepth(PixelDepth pixel_depth)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR1(pixel_depth);

	if (getAcqState() != Idle)
		THROW_HW_FATAL(Error) << "Camera is not idle";

	waitAcqState(Idle);

	EXC_CHECK(m_det->setDynamicRange(pixel_depth));
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

AcqState Camera::getAcqState()
{
	DEB_MEMBER_FUNCT();
	AutoMutex l = lock();
	AcqState state = getEffectiveState(l);
	DEB_RETURN() << DEB_VAR1(state);
	return state;
}

AcqState Camera::getEffectiveState(AutoMutex& l)
{
	DEB_MEMBER_FUNCT();

	CheckLockedAutoMutex(m_cond, l, DEB_PTR());

	if (m_state == Stopped) {
		m_acq_thread = NULL;
		m_state = Idle;
	}
	return m_state;
}

void Camera::waitAcqState(AcqState state)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR1(state);
	AutoMutex l = lock();
	while (getEffectiveState(l) != state)
		m_cond.wait();
}

AcqState Camera::waitNotAcqState(AcqState state)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR1(state);
	AutoMutex l = lock();
	while (getEffectiveState(l) == state)
		m_cond.wait();
	state = getEffectiveState(l);
	DEB_RETURN() << DEB_VAR1(state);
	return state;
}

void Camera::prepareAcq()
{
	DEB_MEMBER_FUNCT();

	StdBufferCbMgr *cb_mgr = m_buffer.getBufferCbMgr();
	if (!cb_mgr)
		THROW_HW_ERROR(Error) << "No Acq BufferCbMgr defined";
	if (!m_model)
		THROW_HW_ERROR(Error) << "No Model defined";

	m_buffer.prepareAcq();

	waitNotAcqState(Stopping);
	if (getAcqState() != Idle)
		THROW_HW_ERROR(Error) << "Camera is not idle";

	bool need_period = !m_lima_nb_frames || (m_lima_nb_frames > 1);
	need_period &= ((m_trig_mode == Defs::Auto) || 
			(m_trig_mode == Defs::BurstTrigger));
	if (need_period && (m_lat_time > 0))
		setFramePeriod(m_exp_time + m_lat_time);

	int nb_buffers;
	cb_mgr->getNbBuffers(nb_buffers);

	{
		AutoMutex l = lock();
		m_prev_ifa.clear();
		RecvList::iterator it, end = m_recv_list.end();
		for (it = m_recv_list.begin(); it != end; ++it)
			(*it)->prepareAcq();

		m_missing_last_skipped_frame.clear();
		if (m_skip_frame_freq)
			for (int i = 0; i < getNbRecvs(); ++i)
				m_missing_last_skipped_frame.insert(i);

		m_next_ready_ts = Timestamp();
	}

	RecvList::iterator rit, rend = m_recv_list.end();
	for (rit = m_recv_list.begin(); rit != rend; ++rit)
		(*rit)->prepareAcq();

	m_model->prepareAcq();
	m_global_cpu_affinity_mgr.prepareAcq();
	m_model->getReconstruction()->prepare();

	resetFramesCaught();
	EXC_CHECK(m_det->setFileWrite(0));
	EXC_CHECK(m_det->setNextFrameNumber(1));
}

void Camera::startAcq()
{
	DEB_MEMBER_FUNCT();

	AutoMutex l = lock();
	if (m_acq_thread)
		THROW_HW_ERROR(Error) << "Must call prepareAcq first";

	StdBufferCbMgr *cb_mgr = m_buffer.getBufferCbMgr();
	cb_mgr->setStartTimestamp(Timestamp::now());

	m_acq_thread = new AcqThread(this);
	m_acq_thread->start(l);
}

void Camera::stopAcq()
{
	DEB_MEMBER_FUNCT();

	Reconstruction *r = m_model ? m_model->getReconstruction() : NULL;
	if (r)
		r->stop();
	m_global_cpu_affinity_mgr.stopAcq();

	AutoMutex l = lock();
	if (getEffectiveState(l) != Running)
		return;

	m_acq_thread->stop(l, true);
	if (getEffectiveState(l) != Idle)
		THROW_HW_ERROR(Error) << "Camera not Idle";
}

void Camera::triggerFrame()
{
	DEB_MEMBER_FUNCT();

	if (m_trig_mode != Defs::SoftTriggerExposure)
		THROW_HW_ERROR(InvalidValue) << "Wrong trigger mode";

	AutoMutex l = lock();
	if (getEffectiveState(l) != Running)
		THROW_HW_ERROR(Error) << "Camera not Running";

	EXC_CHECK(m_det->sendSoftwareTrigger());
	m_next_ready_ts = Timestamp::now();
	m_next_ready_ts += m_exp_time;
}

Camera::DetStatus Camera::getDetTrigStatus()
{
	DEB_MEMBER_FUNCT();

	if (m_trig_mode != Defs::SoftTriggerExposure)
		THROW_HW_ERROR(InvalidValue) << "Wrong trigger mode";

	Timestamp t = Timestamp::now();

	AutoMutex l = lock();
	bool ready = !m_next_ready_ts.isSet() || (m_next_ready_ts < t);
	DetStatus trig_status = ready ? Defs::Waiting : Defs::Running;
	DEB_RETURN() << DEB_VAR1(trig_status);
	return trig_status;
}

void Camera::waitLastSkippedFrame()
{
	DEB_MEMBER_FUNCT();
	AutoMutex l = lock();
	bool stopping = false;
	Timestamp t0;
	SortedIntList& recv_list = m_missing_last_skipped_frame;
	while (!recv_list.empty()) {
		double timeout = -1;
		if (!stopping && (m_state == StopReq)) {
			DEB_TRACE() << "stop requested";
			stopping = true;
			t0 = Timestamp::now();
		} 
		if (stopping) {
			double elapsed = Timestamp::now() - t0;
			timeout = m_last_skipped_frame_timeout - elapsed;
			if (timeout <= 0) {
				DEB_WARNING() << "Missing last skipped frame "
					      << elapsed << " sec after stop: "
					      << "remaining recv_list="
					      << PrettySortedList(recv_list);
				break;
			}
		}
		m_cond.wait(timeout);
	}
}

void Camera::processLastSkippedFrame(int recv_idx)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR1(recv_idx);
	AutoMutex l = lock();
	if (m_missing_last_skipped_frame.erase(recv_idx) != 1)
		DEB_ERROR() << "recv " << recv_idx << " already processed";
	m_cond.broadcast();
}

int Camera::getFramesCaught()
{
	DEB_MEMBER_FUNCT();
	sls::Result<int64_t> res;
	EXC_CHECK(res = m_det->getFramesCaught());
	int64_t frames_caught = 0;
	sls::Result<int64_t>::iterator it, end = res.end();
	for (it = res.begin(); it != end; ++it)
		frames_caught = max(frames_caught, *it);
	DEB_RETURN() << DEB_VAR1(frames_caught);
	return frames_caught;
}

int Camera::getLastFrameCaught()
{
	DEB_MEMBER_FUNCT();
	sls::Result<uint64_t> res;
	EXC_CHECK(res = m_det->getRxCurrentFrameIndex());
	uint64_t last_frame_caught = ULONG_MAX;
	sls::Result<uint64_t>::iterator it, end = res.end();
	for (it = res.begin(); it != end; ++it)
		last_frame_caught = min(last_frame_caught, *it);
	DEB_RETURN() << DEB_VAR1(last_frame_caught);
	return last_frame_caught - 1;
}

Camera::DetStatus Camera::getDetStatus()
{
	DEB_MEMBER_FUNCT();
	typedef slsDetectorDefs::runStatus RunStatus;
	sls::Result<RunStatus> det_status;
	const int nb_retries = 5;
	for (int i = 0; i < nb_retries; ++i) {
		det_status = m_det->getDetectorStatus();
		if (!det_status.equal()) {
			Sleep(0.1);
			DEB_ALWAYS() << "Status are different, retrying ...";
			continue;
		}
		DetStatus status = DetStatus(det_status.squash());
		DEB_RETURN() << DEB_VAR1(status);
		return status;
	}
	static string err_msg = ("Detector status are different after " +
				 to_string(nb_retries) + " retries");
	EXC_CHECK(det_status.tsquash(err_msg));
}

void Camera::setDAC(int mod_idx, DACIndex dac_idx, int val, bool milli_volt)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR4(mod_idx, dac_idx, val, milli_volt);

	if ((mod_idx < -1) || (mod_idx >= getNbDetModules()))
		THROW_HW_ERROR(InvalidValue) << DEB_VAR1(mod_idx);

	typedef slsDetectorDefs::dacIndex DAC;
	DAC sls_idx = DAC(dac_idx);
	Positions pos = Idx2Pos(mod_idx);
	EXC_CHECK(m_det->setDAC(sls_idx, val, milli_volt, pos));
}

void Camera::getDAC(int mod_idx, DACIndex dac_idx, int& val, bool milli_volt)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR3(mod_idx, dac_idx, milli_volt);

	if ((mod_idx < 0) || (mod_idx >= getNbDetModules()))
		THROW_HW_ERROR(InvalidValue) << DEB_VAR1(mod_idx);

	typedef slsDetectorDefs::dacIndex DAC;
	DAC sls_idx = DAC(dac_idx);
	Positions pos = Idx2Pos(mod_idx);
	EXC_CHECK(val = m_det->getDAC(sls_idx, milli_volt, pos).squash(-1));
	DEB_RETURN() << DEB_VAR1(val);
}

void Camera::getDACList(DACIndex dac_idx, IntList& val_list, bool milli_volt)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR2(dac_idx, milli_volt);

	int nb_modules = getNbDetModules();
	val_list.resize(nb_modules);
	for (int i = 0; i < nb_modules; ++i)
		getDAC(i, dac_idx, val_list[i], milli_volt);
}

void Camera::getADC(int mod_idx, ADCIndex adc_idx, int& val)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR2(mod_idx, adc_idx);

	if ((mod_idx < 0) || (mod_idx >= getNbDetModules()))
		THROW_HW_ERROR(InvalidValue) << DEB_VAR1(mod_idx);

	typedef slsDetectorDefs::dacIndex DAC;
	DAC sls_idx = DAC(adc_idx);
	Positions pos = Idx2Pos(mod_idx);
	EXC_CHECK(val = m_det->getTemperature(sls_idx, pos).squash(-1));
	DEB_RETURN() << DEB_VAR1(val);
}

void Camera::getADCList(ADCIndex adc_idx, IntList& val_list)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR1(adc_idx);

	int nb_modules = getNbDetModules();
	val_list.resize(nb_modules);
	for (int i = 0; i < nb_modules; ++i)
		getADC(i, adc_idx, val_list[i]);
}

void Camera::setSettings(Settings settings)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR1(settings);
	if (m_model) {
		if (!m_model->checkSettings(settings))
			THROW_HW_ERROR(InvalidValue) << DEB_VAR1(settings);
		typedef slsDetectorDefs::detectorSettings DetSettings;
		DetSettings cam_settings = DetSettings(settings);
		EXC_CHECK(m_det->setSettings(cam_settings));
	}
	m_settings = settings;
}

void Camera::getSettings(Settings& settings)
{
	DEB_MEMBER_FUNCT();
	settings = m_settings;
	DEB_RETURN() << DEB_VAR1(settings);
}

void Camera::setTolerateLostPackets(bool tol_lost_packets)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR1(tol_lost_packets);
	slsDetectorDefs::frameDiscardPolicy fdp;
	fdp = tol_lost_packets ? slsDetectorDefs::NO_DISCARD :
				 slsDetectorDefs::DISCARD_PARTIAL_FRAMES;
	EXC_CHECK(m_det->setRxFrameDiscardPolicy(fdp));
	m_tol_lost_packets = tol_lost_packets;
}

void Camera::getTolerateLostPackets(bool& tol_lost_packets)
{
	DEB_MEMBER_FUNCT();
	tol_lost_packets = m_tol_lost_packets;
	DEB_RETURN() << DEB_VAR1(tol_lost_packets);
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

void Camera::getStats(Stats& stats, int recv_idx)
{
	DEB_MEMBER_FUNCT();
	if ((recv_idx < -1) || (recv_idx >= getNbRecvs()))
		THROW_HW_ERROR(InvalidValue) << DEB_VAR1(recv_idx);

	if (recv_idx < 0) {
		stats.reset();
		RecvList::iterator it, end = m_recv_list.end();
		for (it = m_recv_list.begin(); it != end; ++it)
			stats += (*it)->getStats();
	} else {
		Receiver *recv = m_recv_list[recv_idx];
		stats = recv->getStats();
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
	EXC_CHECK(m_det->setRxFifoDepth(fifo_depth));
}

void Camera::getReceiverFifoDepth(int& fifo_depth)
{
	DEB_MEMBER_FUNCT();
	const char *err_msg = "Rx fifo_depth are different";
	EXC_CHECK(fifo_depth = m_det->getRxFifoDepth().tsquash(err_msg));
	DEB_RETURN() << DEB_VAR1(fifo_depth);
}

void Camera::resetFramesCaught()
{
	DEB_MEMBER_FUNCT();
	// nothing to do
}

void Camera::reportException(Exception& e, string name)
{
	DEB_MEMBER_FUNCT();

	ostringstream err_msg;
	err_msg << name << " failed: " << e;
	Event::Code err_code = Event::CamCommError;
	Event *event = new Event(Hardware, Event::Error, Event::Camera, 
				 err_code, err_msg.str());
	DEB_EVENT(*event) << DEB_VAR1(*event);
	reportEvent(event);
}
