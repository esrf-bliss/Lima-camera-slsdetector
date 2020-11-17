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

Receiver::Receiver(Camera *cam, int idx, int rx_port)
	: m_cam(cam), m_idx(idx), m_rx_port(rx_port)
{
	DEB_CONSTRUCTOR();

	ostringstream os;
	os << "slsReceiver"
	   << " --rx_tcpport " << m_rx_port;
	m_args.set(os.str());

	start();
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
	m_recv->setPassiveMode(true);
	if (m_recv->start() == slsReceiverDefs::FAIL) 
		THROW_HW_ERROR(Error) << "Error starting slsReceiver";
}

void Receiver::prepareAcq()
{
	DEB_MEMBER_FUNCT();
	m_stats.reset();
}

void Receiver::setCPUAffinity(const RecvCPUAffinity& recv_affinity)
{
	DEB_MEMBER_FUNCT();

	const CPUAffinityList& aff_list = recv_affinity.listeners;
	slsReceiverDefs::CPUMaskList cpu_masks(aff_list.size());
	slsReceiverDefs::CPUMaskList::iterator mit = cpu_masks.begin();
	CPUAffinityList::const_iterator it, end = aff_list.end();
	for (it = aff_list.begin(); it != end; ++it, ++mit)
		it->initCPUSet(*mit);
	m_recv->setThreadCPUAffinity(cpu_masks);

	string deb_head;
	if (DEB_CHECK_ANY(DebTypeTrace) || DEB_CHECK_ANY(DebTypeWarning)) {
		ostringstream os;
		os << "setting recv " << m_idx << " ";
		deb_head = os.str();
	}

	CPUAffinity aff = recv_affinity.all();
	int max_node;
	vector<unsigned long> mlist;
	aff.getNUMANodeMask(mlist, max_node);
	if (mlist.size() != 1)
		THROW_HW_ERROR(Error) << DEB_VAR1(mlist.size());
	unsigned long& fifo_node_mask = mlist[0];
	int c = bitset<64>(fifo_node_mask).count();
	if (c != 1)
		DEB_WARNING() << deb_head << "Fifo NUMA node mask has "
			      << c << " nodes";
	DEB_ALWAYS() << deb_head << DEB_VAR2(DEB_HEX(fifo_node_mask), max_node);
	m_recv->setFifoNodeAffinity(fifo_node_mask, max_node);
}

inline bool Receiver::readRecvImage(FrameType lima_frame, FrameType det_frame,
				    ImageData *image_data)
{
	DEB_MEMBER_FUNCT();

	ImageData skip_image_data;
	const char *action = "read";
	if (!image_data) {
		image_data = &skip_image_data;
		image_data->buffer = NULL;
		action = "skip";
	}

	DEB_TRACE() << "To " << action << ": "
		    << DEB_VAR3(m_idx, lima_frame, det_frame);
	image_data->frame = det_frame;
	int ret = m_recv->getImage(*image_data);
	if ((ret != 0) || (m_cam->getAcqState() == Stopping))
		return false;

	sls_receiver_header& header = image_data->header;
	FrameType recv_frame = header.detHeader.frameNumber;
	if (recv_frame > m_cam->m_det_nb_frames)
		THROW_HW_ERROR(Error) << "Invalid frame: " 
				      << DEB_VAR3(m_idx, recv_frame,
						  DebHex(recv_frame));
	else if (recv_frame != det_frame)
		THROW_HW_ERROR(Error) << "Bad frame: "
				      << DEB_VAR3(m_idx, recv_frame, det_frame);
	return true;
}

bool Receiver::getImage(ImageData& image_data)
{
	DEB_MEMBER_FUNCT();

	Timestamp t0 = Timestamp::now();

	if (m_stats.last_t0.isSet())
		m_stats.stats.cb_period.add(t0 - m_stats.last_t0);
	if (m_stats.last_t1.isSet())
		m_stats.stats.recv_exec.add(t0 - m_stats.last_t1);
	m_stats.last_t0 = t0;

	FrameType lima_frame = image_data.frame;
	try {
		FrameType det_frame = lima_frame + 1;  // Eiger first frame is 1
		bool skip_prev = false;
		bool skip_last = false;
		FrameType skip_freq = m_cam->m_skip_frame_freq;
		if (skip_freq) {
			det_frame += lima_frame / skip_freq;
			skip_prev = (lima_frame &&
				     (lima_frame % skip_freq == 0));
			skip_last = ((det_frame + 1) == m_cam->m_det_nb_frames);
			DEB_TRACE() << DEB_VAR5(m_idx, lima_frame, det_frame,
						skip_prev, skip_last);
		}

		if (skip_prev && !readRecvImage(lima_frame, det_frame - 1))
			return false;

		if (!readRecvImage(lima_frame, det_frame, &image_data))
			return false;

		if (skip_last) {
			if (!readRecvImage(lima_frame, det_frame + 1))
				return false;
			m_cam->processLastSkippedFrame(m_idx);
		}

		image_data.frame = lima_frame;
	} catch (Exception& e) {
		ostringstream name;
		name << "Receiver::getImage: " << DEB_VAR2(m_idx, lima_frame);
		m_cam->reportException(e, name.str());
		return false;
	}

	Timestamp t1 = Timestamp::now();
	m_stats.stats.cb_exec.add(t1 - t0);
	m_stats.last_t1 = t1;

	return true;
}

