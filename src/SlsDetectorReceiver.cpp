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

Receiver::Port::Thread::~Thread()
{
	DEB_DESTRUCTOR();

	if (!hasStarted())
		return;

	m_end = true;
	m_port->stopPollFrameFinished();
}

void Receiver::Port::Thread::init(Port *port, int idx)
{
	DEB_MEMBER_FUNCT();
	m_port = port;
	m_idx = idx;
	start();
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

	m_end = false;
	while (!m_end)
		m_port->pollFrameFinished(m_idx);
}

Receiver::Port::Port(Receiver& recv, int port)
{
	DEB_CONSTRUCTOR();

	m_cam = recv.m_cam;
	m_port_idx = m_cam->getPortIndex(recv.m_idx, port);
	m_model_port = m_cam->m_model->getRecvPort(m_port_idx);
}

void Receiver::Port::setNbThreads(int nb_threads)
{
	DEB_MEMBER_FUNCT();
	int prev_nb_threads = m_thread_list.size();
	DEB_PARAM() << DEB_VAR2(nb_threads, prev_nb_threads);

	if (nb_threads == prev_nb_threads)
		return;

	m_frame_map_item_list.resize(nb_threads);
	int item = m_port_idx * nb_threads;
	for (int i = 0; i < nb_threads; ++i, ++item)
		m_frame_map_item_list[i] = &m_cam->m_frame_map.getItem(item);

	m_thread_list.resize(nb_threads);
	for (int i = prev_nb_threads; i < nb_threads; ++i)
		m_thread_list[i].init(this, i);
}

int Receiver::Port::getNbThreads()
{
	return m_thread_list.size();
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
	m_model_port->processRecvFileStart(dsize);
}

void Receiver::Port::processFrame(FrameType frame, char *dptr, uint32_t dsize)
{
	DEB_MEMBER_FUNCT();

	FrameMapItemList::iterator it, end = m_frame_map_item_list.end();
	for (it = m_frame_map_item_list.begin(); it != end; ++it)
		(*it)->checkFinishedFrame(frame);
	bool valid = (dptr != NULL);
	if (valid) {
		char *bptr = m_cam->getFrameBufferPtr(frame);
		m_model_port->processRecvPort(frame, dptr, dsize, bptr);
	}
	Timestamp t0 = Timestamp::now();
	for (it = m_frame_map_item_list.begin(); it != end; ++it)
		(*it)->frameFinished(frame, true, valid);
	Timestamp t1 = Timestamp::now();
	m_stats.stats.new_finish.add(t1 - t0);
}

void Receiver::Port::pollFrameFinished(int thread_idx)
{
	DEB_MEMBER_FUNCT();

	FrameMap::Item *frame_map_item = m_frame_map_item_list[thread_idx];
	FrameDataList data_list = frame_map_item->pollFrameFinished();
	if (m_model_port->getNbPortProcessingThreads()) {
		FrameDataList::const_iterator it, end = data_list.end();
		for (it = data_list.begin(); it != end; ++it) {
			FrameType frame = it->first;
			char *bptr = m_cam->getFrameBufferPtr(frame);
			m_model_port->processPortThread(frame, bptr, 
							thread_idx);
		}
	}

	FinishInfoList finfo_list;
	finfo_list = frame_map_item->getFrameFinishInfo(data_list);
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
	FrameMapItemList::iterator it, end = m_frame_map_item_list.end();
	for (it = m_frame_map_item_list.begin(); it != end; ++it)
		(*it)->stopPollFrameFinished();
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
	m_recv->setFrameEventPolicy(slsReceiverUsers::SkipMissingFrames);
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

void Receiver::setCPUAffinity(const RecvCPUAffinity& recv_affinity)
{
	DEB_MEMBER_FUNCT();

	enum ThreadType {
		Listener, Writer, PortThread, NbThreadTypes,
	};

	Model *model = m_cam->m_model;
	Model::RecvPort *port = model->getRecvPort(0);
	int nb_proc_threads = port->getNbPortProcessingThreads();

	class AffinityListHelper
	{
	public:
		AffinityListHelper(const CPUAffinityList& aff_list,
				   int nb_ports, int nb_proc_threads,
				   ThreadType type, int recv_idx,
				   DebObj *deb_ptr)
			: m_aff_list(aff_list), m_nb_ports(nb_ports),
			  m_nb_proc_threads(nb_proc_threads),
			  m_type(type), m_recv_idx(recv_idx), m_deb_ptr(deb_ptr)
		{
			DEB_FROM_PTR(m_deb_ptr);
			unsigned int nb_aff = m_aff_list.size();
			if (!defaultAffinity() && !singleAffinity() &&
			    (nb_aff != getNbAffinities()))
				THROW_HW_ERROR(InvalidValue) <<
					DEB_VAR2(nb_aff, getNbAffinities());
		}

		int getNbAffinities() const
		{
			int nb_aff = m_nb_ports;
			if (m_type == PortThread)
				nb_aff *= m_nb_proc_threads;
			return nb_aff;
		}

		bool defaultAffinity() const
		{ return m_aff_list.empty(); }
		bool singleAffinity() const
		{ return (m_aff_list.size() == 1); }

		ConstStr typeDesc() const
		{
			switch (m_type) {
			case Listener: return "listener";
			case Writer: return "writer";
			case PortThread: return "port-thread";
			}
		}

		string getDebHead() const
		{
			DEB_FROM_PTR(m_deb_ptr);
			string deb_head;
			if (DEB_CHECK_ANY(DebTypeTrace)) {
				ostringstream os;
				os << "setting recv " << m_recv_idx << " "
				   << typeDesc() << " ";
				deb_head = os.str();
			}
			return deb_head;
		}

		CPUAffinityList getThreadAffinityList() const
		{
			DEB_FROM_PTR(m_deb_ptr);
			CPUAffinityList list;
			int nb_aff = getNbAffinities();
			if (defaultAffinity())
				list = CPUAffinityList(nb_aff);
			else if (singleAffinity() && (nb_aff > 1))
				list = CPUAffinityList(nb_aff, m_aff_list[0]);
			else
				list = m_aff_list;

			string deb_head = getDebHead();
			CPUAffinityList::const_iterator it = list.begin();
			for (int i = 0; i < nb_aff; ++i, ++it)
				DEB_TRACE() << deb_head << i << " "
					    << "CPU mask to " << *it;

			return list;
		}

		slsReceiverUsers::CPUMaskList getCPUMaskList() const
		{
			CPUAffinityList list = getThreadAffinityList();
			slsReceiverUsers::CPUMaskList cpu_mask_list;
			CPUAffinityList::const_iterator it = list.begin();
			for (int i = 0; i < m_nb_ports; ++i, ++it) {
				cpu_set_t cpu_set;
				it->initCPUSet(cpu_set);
				cpu_mask_list.push_back(cpu_set);
			}
			return cpu_mask_list;
		}

	private:
		const CPUAffinityList& m_aff_list;
		int m_nb_ports;
		int m_nb_proc_threads;
		ThreadType m_type;
		int m_recv_idx;
		DebObj *m_deb_ptr;
	};

	int nb_ports = m_port_list.size();

#define CreateHelper(n, a, t) \
	AffinityListHelper n(a, nb_ports, nb_proc_threads, t, m_idx, DEB_PTR());

	CreateHelper(lh, recv_affinity.listeners, Listener);
	CreateHelper(wh, recv_affinity.writers, Writer);
	CreateHelper(pth, recv_affinity.port_threads, PortThread);

#undef CreateHelper

	slsReceiverUsers::CPUMaskList list_cpu_mask = lh.getCPUMaskList();
	slsReceiverUsers::CPUMaskList writ_cpu_mask = wh.getCPUMaskList();
	m_recv->setThreadCPUAffinity(list_cpu_mask, writ_cpu_mask);

	slsReceiverUsers::NodeMaskList fifo_node_mask;
	int max_node;
	getNodeMaskList(recv_affinity.listeners, recv_affinity.writers,
			fifo_node_mask, max_node);
	m_recv->setFifoNodeAffinity(fifo_node_mask, max_node);

	CPUAffinityList port_thread_aff_list = pth.getThreadAffinityList();
	CPUAffinityList::const_iterator tit = port_thread_aff_list.begin();
	PortList::iterator pit, pend = m_port_list.end();
	for (pit = m_port_list.begin(); pit != pend; ++pit) {
		for (int i = 0; i < (*pit)->getNbThreads(); ++i, ++tit) {
			pid_t tid = (*pit)->getThreadID(i);
			(*tit).applyToTask(tid, false);
		}
	}
}

void Receiver::getNodeMaskList(const CPUAffinityList& listener,
			       const CPUAffinityList& writer,
			       slsReceiverUsers::NodeMaskList& fifo_node_mask,
			       int& max_node)
{
	DEB_MEMBER_FUNCT();

	fifo_node_mask.clear();

	string deb_head;
	if (DEB_CHECK_ANY(DebTypeTrace) || DEB_CHECK_ANY(DebTypeWarning)) {
		ostringstream os;
		os << "setting recv " << m_idx << " ";
		deb_head = os.str();
	}

	CPUAffinityList::const_iterator lit = listener.begin();
	CPUAffinityList::const_iterator wit = writer.begin();
	int nb_ports = m_port_list.size();
	for (int i = 0; i < nb_ports; ++i, ++lit, ++wit) {
		CPUAffinity both = *lit | *wit;
		vector<unsigned long> mlist;
		both.getNUMANodeMask(mlist, max_node);
		if (mlist.size() != 1)
			THROW_HW_ERROR(Error) << DEB_VAR1(mlist.size());
		unsigned long& nmask = mlist[0];
		DEB_TRACE() << deb_head << "Fifo " << i << " "
			    << "NUMA node mask mask to " << DEB_HEX(nmask);
		int c = bitset<64>(nmask).count();
		if (c != 1)
			DEB_WARNING() << deb_head << "Fifo " << i << " "
				      << "NUMA node mask has " << c << " nodes";
		fifo_node_mask.push_back(nmask);
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
	FrameType det_frame = frame - 1;
	DEB_PARAM() << DEB_VAR2(frame, det_frame);
	recv->portCallback(det_frame, port, dptr, dsize);
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

void Receiver::portCallback(FrameType det_frame, int port, char *dptr,
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
		if (det_frame >= m_cam->m_det_nb_frames)
			THROW_HW_ERROR(Error) << "Invalid " 
					      << DEB_VAR2(det_frame,
							  DebHex(det_frame));
		FrameType skip_freq = m_cam->m_skip_frame_freq;
		bool skip_frame = false;
		FrameType lima_frame = det_frame;
		int port_idx = recv_port.m_port_idx;
		if (skip_freq) {
			skip_frame = ((det_frame + 1) % (skip_freq + 1) == 0);
			lima_frame -= det_frame / (skip_freq + 1);
			DEB_TRACE() << DEB_VAR4(port_idx, det_frame,
						skip_frame, lima_frame);
		}
		if (skip_frame) {
			if (det_frame == m_cam->m_det_nb_frames - 1)
				m_cam->processLastSkippedFrame(port_idx);
		} else {
			recv_port.processFrame(lima_frame, dptr, dsize);
		}
	} catch (Exception& e) {
		ostringstream err_msg;
		err_msg << "Receiver::portCallback: " << e << ": "
			<< DEB_VAR3(m_idx, det_frame, port);
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
