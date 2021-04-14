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
	: m_cam(cam), m_idx(idx), m_rx_port(rx_port), m_gap_pixels_enable(false)
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
}

void Receiver::start()
{	
	DEB_MEMBER_FUNCT();
	EXC_CHECK(m_recv = new sls::Receiver(m_args.size(), m_args));
	m_recv->setPassiveMode(true);
}

void Receiver::prepareAcq()
{
	DEB_MEMBER_FUNCT();
	m_recv->enableGap(m_gap_pixels_enable);
	m_stats.reset();
}

void Receiver::setCPUAffinity(const RecvCPUAffinity& recv_affinity)
{
	DEB_MEMBER_FUNCT();

	const CPUAffinityList& aff_list = recv_affinity.listeners;
	slsDetectorDefs::CPUMaskList cpu_masks(aff_list.size());
	slsDetectorDefs::CPUMaskList::iterator mit = cpu_masks.begin();
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
	m_recv->setBufferNodeAffinity(fifo_node_mask, max_node);
}

inline bool Receiver::readRecvImage(ImageData *image_data)
{
	DEB_MEMBER_FUNCT();

	ImageData skip_image_data;
	const char *action = "read";
	if (!image_data) {
		image_data = &skip_image_data;
		image_data->buffer = NULL;
		action = "skip";
	}

	DEB_TRACE() << "Action: " << action;
	int ret = m_recv->getImage(*image_data);
	if ((ret != 0) || (m_cam->getAcqState() == Stopping)) {
		DEB_RETURN() << DEB_VAR1(false);
		return false;
	}

	sls_receiver_header& header = image_data->header;
	DEB_TRACE() << DEB_VAR4(header.detHeader.frameNumber,
				header.detHeader.modId,
				header.detHeader.row,
				header.detHeader.column);
	FrameType recv_frame = header.detHeader.frameNumber;
	if (recv_frame > m_cam->m_det_nb_frames)
		THROW_HW_ERROR(Error) << "Invalid frame: " 
				      << DEB_VAR3(m_idx, recv_frame,
						  DebHex(recv_frame));
	DEB_RETURN() << DEB_VAR1(true);
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

	try {
		if (!readRecvImage(&image_data))
			return false;

		FrameType& det_frame = image_data.header.detHeader.frameNumber;
		bool skip_this = false;
		bool skip_last = false;
		FrameType skip_freq = m_cam->m_skip_frame_freq;
		if (skip_freq) {
			skip_this = (det_frame % (skip_freq + 1) == 0);
			skip_last = ((det_frame + 1) == m_cam->m_det_nb_frames);
			DEB_TRACE() << DEB_VAR4(m_idx, det_frame, skip_this,
						skip_last);
		}

		if (skip_this && !readRecvImage(&image_data))
			return false;

		if (skip_freq)
			det_frame -= det_frame / (skip_freq + 1);
		// first frame is set to 1
		--det_frame;

		if (skip_last) {
			if (!readRecvImage(NULL))
				return false;
			m_cam->processLastSkippedFrame(m_idx);
		}
	} catch (Exception& e) {
		ostringstream name;
		name << "Receiver::getImage: " << DEB_VAR1(m_idx);
		m_cam->reportException(e, name.str());
		return false;
	}

	Timestamp t1 = Timestamp::now();
	m_stats.stats.cb_exec.add(t1 - t0);
	m_stats.last_t1 = t1;

	return true;
}

