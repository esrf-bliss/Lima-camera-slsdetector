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

Receiver::Port::Port(Receiver& recv, int port)
{
	DEB_CONSTRUCTOR();

	m_cam = recv.m_cam;
	m_model_port = recv.m_model_recv->getPort(port);
	m_port_idx = m_cam->getPortIndex(recv.m_idx, port);
}

void Receiver::Port::prepareAcq()
{
	DEB_MEMBER_FUNCT();
	m_stats.reset();
}

void Receiver::Port::portCallback(FrameType det_frame, char *dptr, 
				  uint32_t dsize)
{
	DEB_MEMBER_FUNCT();

	Timestamp t0 = Timestamp::now();

	if (m_stats.last_t0.isSet())
		m_stats.stats.cb_period.add(t0 - m_stats.last_t0);
	if (m_stats.last_t1.isSet())
		m_stats.stats.recv_exec.add(t0 - m_stats.last_t1);
	m_stats.last_t0 = t0;

	try {
		if (det_frame >= m_cam->m_det_nb_frames)
			THROW_HW_ERROR(Error) << "Invalid frame: " 
					      << DEB_VAR3(m_port_idx,
							  det_frame,
							  DebHex(det_frame));
		FrameType skip_freq = m_cam->m_skip_frame_freq;
		bool skip_frame = false;
		FrameType lima_frame = det_frame;
		if (skip_freq) {
			skip_frame = ((det_frame + 1) % (skip_freq + 1) == 0);
			lima_frame -= det_frame / (skip_freq + 1);
			DEB_TRACE() << DEB_VAR4(m_port_idx, det_frame,
						skip_frame, lima_frame);
		}
		if (skip_frame) {
			if (det_frame == m_cam->m_det_nb_frames - 1)
				m_cam->processLastSkippedFrame(m_port_idx);
		} else {
			Timestamp t0 = Timestamp::now();
			char *bptr = m_cam->getFrameBufferPtr(lima_frame);
			m_model_port->processFrame(lima_frame, dptr, dsize,
						   bptr);
			Timestamp t1 = Timestamp::now();
			m_stats.stats.new_finish.add(t1 - t0);
		}
	} catch (Exception& e) {
		ostringstream name;
		name << "Receiver::Port::portCallback: "
		     << DEB_VAR2(m_port_idx, det_frame);
		m_cam->reportException(e, name.str());
	}

	Timestamp t1 = Timestamp::now();
	m_stats.stats.cb_exec.add(t1 - t0);
	m_stats.last_t1 = t1;
}

Receiver::Receiver(Camera *cam, int idx, int rx_port)
	: m_cam(cam), m_idx(idx), m_rx_port(rx_port), m_model_recv(NULL)
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

void Receiver::setModelRecv(Model::Recv *model_recv)
{
	DEB_MEMBER_FUNCT();

	m_model_recv = model_recv;

	int nb_ports = model_recv->getNbPorts();
	for (int i = 0; i < nb_ports; ++i) {
		AutoPtr<Port> port = new Port(*this, i);
		m_port_list.push_back(port);
	}
}

void Receiver::setCPUAffinity(const RecvCPUAffinity& recv_affinity)
{
	DEB_MEMBER_FUNCT();

	enum ThreadType {
		Listener, Writer, RecvThread, NbThreadTypes,
	};

	Model *model = m_cam->m_model;
	Model::Recv *recv = model->getRecv(m_idx);

	recv->setNbProcessingThreads(recv_affinity.recv_threads.size());
	int nb_recv_threads = recv->getNbProcessingThreads();

	class AffinityListHelper
	{
	public:
		AffinityListHelper(const CPUAffinityList& aff_list,
				   int nb_ports, int nb_recv_threads,
				   ThreadType type, int recv_idx,
				   DebObj *deb_ptr)
			: m_aff_list(aff_list), m_nb_ports(nb_ports),
			  m_nb_recv_threads(nb_recv_threads),
			  m_type(type), m_recv_idx(recv_idx), m_deb_ptr(deb_ptr)
		{
			DEB_FROM_PTR(m_deb_ptr);
			unsigned int nb_aff = m_aff_list.size();
			if (!defaultAffinity() && !singleAffinity() &&
			    (nb_aff != getNbAffinities()))
				THROW_HW_ERROR(InvalidValue) << typeDesc() 
							     << " " <<
					DEB_VAR2(nb_aff, getNbAffinities());
		}

		int getNbAffinities() const
		{
			if (m_type == RecvThread)
				return m_nb_recv_threads;
			else
				return m_nb_ports;
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
			case RecvThread: return "recv-thread";
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
		int m_nb_recv_threads;
		ThreadType m_type;
		int m_recv_idx;
		DebObj *m_deb_ptr;
	};

	int nb_ports = m_port_list.size();

#define CreateHelper(n, a, t) \
	AffinityListHelper n(a, nb_ports, nb_recv_threads, t, m_idx, DEB_PTR());

	CreateHelper(lh, recv_affinity.listeners, Listener);
	CreateHelper(wh, recv_affinity.writers, Writer);
	CreateHelper(rth, recv_affinity.recv_threads, RecvThread);

#undef CreateHelper

	slsReceiverUsers::CPUMaskList list_cpu_mask = lh.getCPUMaskList();
	slsReceiverUsers::CPUMaskList writ_cpu_mask = wh.getCPUMaskList();
	m_recv->setThreadCPUAffinity(list_cpu_mask, writ_cpu_mask);

	slsReceiverUsers::NodeMaskList fifo_node_mask;
	int max_node;
	getNodeMaskList(recv_affinity.listeners, recv_affinity.writers,
			fifo_node_mask, max_node);
	m_recv->setFifoNodeAffinity(fifo_node_mask, max_node);

	CPUAffinityList recv_thread_aff_list = rth.getThreadAffinityList();
	CPUAffinityList::const_iterator rit = recv_thread_aff_list.begin();
	for (int i = 0; i < recv->getNbProcessingThreads(); ++i, ++rit) {
		pid_t tid = recv->getThreadID(i);
		rit->applyToTask(tid, false);
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

	int nb_ports = recv->m_port_list.size();
	if ((port >= nb_ports) || (recv->m_cam->getState() == Stopping))
		return;

	Port *recv_port = recv->m_port_list[port];
	recv_port->portCallback(det_frame, dptr, dsize);
}

int Receiver::fileStartCallback(char *fpath, char *fname, uint64_t fidx,
				uint32_t dsize)
{
	DEB_MEMBER_FUNCT();
	m_model_recv->processFileStart(dsize);
	return 0;
}
