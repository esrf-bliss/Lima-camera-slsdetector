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

using namespace std;
using namespace lima;
using namespace lima::SlsDetector;


Receiver::Port::Thread::Thread(Port& port)
	: m_port(port)
{
	DEB_CONSTRUCTOR();
}

Receiver::Port::Thread::~Thread()
{
	DEB_DESTRUCTOR();

	if (!hasStarted())
		return;

	m_end = true;
	m_port.stopPollFrameFinished();
}

void Receiver::Port::Thread::start()
{
	DEB_MEMBER_FUNCT();

	m_end = true;
	lima::Thread::start();

	struct sched_param param;
	param.sched_priority = 50;
	int ret = pthread_setschedparam(m_thread, SCHED_RR, &param);
	if (ret != 0)
		DEB_ERROR() << "Could not set real-time priority!!";

	while (m_end)
		Sleep(10e-3);
}

void Receiver::Port::Thread::threadFunction()
{
	DEB_MEMBER_FUNCT();

	m_tid = gettid();

	m_end = false;
	while (!m_end)
		m_port.pollFrameFinished();
}

Receiver::Port::Port(Receiver& recv, int port)
	: m_thread(*this)
{
	DEB_CONSTRUCTOR();

	m_cam = recv.m_cam;
	m_model = m_cam->m_model;
	m_port_idx = m_cam->getPortIndex(recv.m_idx, port);
	m_frame_map_item = &m_cam->m_frame_map.getItem(m_port_idx);
	  
	m_thread.start();
}

void Receiver::Port::prepareAcq()
{
	DEB_MEMBER_FUNCT();
	m_stats.reset();
	m_bad_frame_list.clear();
	m_bad_frame_list.reserve(16 * 1024);
}

void Receiver::Port::processFileStart(uint32_t dsize)
{
	DEB_MEMBER_FUNCT();
	m_model->processRecvFileStart(m_port_idx, dsize);
}

void Receiver::Port::processFrame(FrameType frame, char *dptr, uint32_t dsize)
{
	DEB_MEMBER_FUNCT();

	m_frame_map_item->checkFinishedFrame(frame);
	bool valid = (dptr != NULL);
	if (valid) {
		char *bptr = m_cam->getFrameBufferPtr(frame);
		m_model->processRecvPort(m_port_idx, frame, dptr, dsize, bptr);
	}
	Timestamp t0 = Timestamp::now();
	m_frame_map_item->frameFinished(frame, true, valid);
	Timestamp t1 = Timestamp::now();
	m_stats.stats.new_finish.add(t1 - t0);
}

void Receiver::Port::pollFrameFinished()
{
	DEB_MEMBER_FUNCT();

	FinishInfoList finfo_list;
	finfo_list = m_frame_map_item->pollFrameFinished();
	FinishInfoList::const_iterator it, end = finfo_list.end();
	for (it = finfo_list.begin(); it != end; ++it) {
		const FinishInfo& finfo = *it;
		if ((finfo.nb_lost != 0) || !finfo.finished.empty())
			processFinishInfo(finfo);
	}
}

void Receiver::Port::stopPollFrameFinished()
{
	DEB_MEMBER_FUNCT();
	m_frame_map_item->stopPollFrameFinished();
}

void Receiver::Port::processFinishInfo(const FinishInfo& finfo)
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
		err_msg << "Port::processFinishInfo: " << e;
		Event::Code err_code = Event::CamOverrun;
		Event *event = new Event(Hardware, Event::Error, Event::Camera, 
					 err_code, err_msg.str());
		DEB_EVENT(*event) << DEB_VAR1(*event);
		m_cam->reportEvent(event);
	}
}

bool Receiver::Port::isBadFrame(FrameType frame)
{ 
	AutoMutex l = lock();
	IntList::iterator end = m_bad_frame_list.end();
	return (find(m_bad_frame_list.begin(), end, frame) != end); 
}

Receiver::Receiver(Camera *cam, int idx, int rx_port)
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

Receiver::~Receiver()
{
	DEB_DESTRUCTOR();
	m_recv->stop();
}

void Receiver::start()
{	
	DEB_MEMBER_FUNCT();
	int init_ret;
	m_recv = new slsReceiverUsers(m_args.size(), m_args, init_ret);
	if (init_ret == slsReceiverDefs::FAIL)
		THROW_HW_ERROR(Error) << "Error creating slsReceiver";
	if (m_recv->start() == slsReceiverDefs::FAIL) 
		THROW_HW_ERROR(Error) << "Error starting slsReceiver";
}

void Receiver::setNbPorts(int nb_ports)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR1(nb_ports);

	for (int i = 0; i < nb_ports; ++i) {
		AutoPtr<Port> port = new Port(*this, i);
		m_port_list.push_back(port);
	}
}

void Receiver::prepareAcq()
{
	DEB_MEMBER_FUNCT();
	PortList::iterator it, end = m_port_list.end();
	for (it = m_port_list.begin(); it != end; ++it)
		(*it)->prepareAcq();
}

int Receiver::fileStartCallback(char *fpath, char *fname, uint64_t fidx,
				uint32_t dsize, void *priv)
{
	DEB_STATIC_FUNCT();
	Receiver *recv = static_cast<Receiver *>(priv);
	return recv->fileStartCallback(fpath, fname, fidx, dsize);
}

void Receiver::portCallback(FrameType frame,
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

int Receiver::fileStartCallback(char *fpath, char *fname, uint64_t fidx,
				uint32_t dsize)
{
	DEB_MEMBER_FUNCT();
	PortList::iterator it, end = m_port_list.end();
	for (it = m_port_list.begin(); it != end; ++it)
		(*it)->processFileStart(dsize);
	return 0;
}

void Receiver::portCallback(FrameType frame, int port, char *dptr,
			    uint32_t dsize)
{
	DEB_MEMBER_FUNCT();

	int nb_ports = m_port_list.size();
	if ((port >= nb_ports) || (m_cam->getState() == Stopping))
		return;

	Timestamp t0 = Timestamp::now();

	Port& recv_port = *m_port_list[port];
	Port::Stats& port_stats = recv_port.m_stats;
	if (port_stats.last_t0.isSet())
		port_stats.stats.cb_period.add(t0 - port_stats.last_t0);
	if (port_stats.last_t1.isSet())
		port_stats.stats.recv_exec.add(t0 - port_stats.last_t1);
	port_stats.last_t0 = t0;

	try {
		if (frame >= m_cam->m_nb_frames)
			THROW_HW_ERROR(Error) << "Invalid " 
					      << DEB_VAR2(frame, DebHex(frame));
		recv_port.processFrame(frame, dptr, dsize);
	} catch (Exception& e) {
		ostringstream err_msg;
		err_msg << "Receiver::portCallback: " << e << ": "
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
