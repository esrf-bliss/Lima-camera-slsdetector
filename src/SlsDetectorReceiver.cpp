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

#include "sls/Receiver.h"

using namespace std;
using namespace lima;
using namespace lima::SlsDetector;

using namespace sls::FrameAssembler;

using AnyPacketBlockList = sls::AnyPacketBlockList;

struct RecvImagePackets : public Receiver::ImagePackets {
	using Receiver::ImagePackets::ImagePackets;
	AnyPacketBlockList blocks;
};

inline
AnyPacketBlockList& RecvImagePacketBlocks(Receiver::ImagePackets *image_data) {
	RecvImagePackets *rd = static_cast<RecvImagePackets *>(image_data);
	return rd->blocks;
}

struct Receiver::AssemblerImpl {
	MPFrameAssemblerPtr m_asm;
};

Receiver::Receiver(Camera *cam, int idx, int rx_port)
	: m_cam(cam), m_idx(idx), m_rx_port(rx_port), m_gap_pixels_enable(false)
{
	DEB_CONSTRUCTOR();

	ostringstream os;
	os << "slsReceiver"
	   << " --rx_tcpport " << m_rx_port;
	m_args.set(os.str());

	start();

	m_asm_impl = new AssemblerImpl();
}

Receiver::~Receiver()
{
	DEB_DESTRUCTOR();
	delete m_asm_impl;
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
	AssemblerType asm_type = m_gap_pixels_enable ? AsmWithGap : AsmRaw;
	m_asm_impl->m_asm = std::move(m_recv->CreateFrameAssembler(asm_type));
	m_stats.reset();
	m_last_skipped = false;
}

void Receiver::setCPUAffinity(const RecvCPUAffinity& recv_affinity)
{
	DEB_MEMBER_FUNCT();

	using namespace sls::CPUAffinity;
	const CPUAffinityList& aff_list = recv_affinity.listeners;
	FixedCPUSetAffinityList cpu_masks(aff_list.size());
	FixedCPUSetAffinityList::iterator mit = cpu_masks.begin();
	CPUAffinityList::const_iterator it, end = aff_list.end();
	for (it = aff_list.begin(); it != end; ++it, ++mit)
		it->initCPUSet(mit->cpu_mask().cpu_set());
	m_recv->setListenersCPUAffinity(cpu_masks);
}

AutoPtr<Receiver::ImagePackets> Receiver::readSkippableImagePackets()
{
	DEB_MEMBER_FUNCT();
	AutoPtr<ImagePackets> image_data = new RecvImagePackets(this);
	AnyPacketBlockList& blocks = RecvImagePacketBlocks(image_data);
	sls_detector_header& header = image_data->header.detHeader;
	header.frameNumber = -1;
	blocks = std::move(m_recv->GetFramePacketBlocks());
	image_data->numberOfPorts = blocks.size();
	bool incomplete_data = (image_data->numberOfPorts == 0);
	for (int i = 0; i < image_data->numberOfPorts; ++i) {
		std::visit([&](auto &block) {
			bool valid(block);
			image_data->validPortData[i] = valid;
			if (valid && (header.frameNumber == uint64_t(-1)) &&
			    block->getNetworkHeader())
				header = *block->getNetworkHeader();
			incomplete_data |= !(valid && block->hasFullFrame());
		    }, blocks[i]);
	}
	if (incomplete_data && !m_cam->m_tol_lost_packets)
		THROW_HW_ERROR(Error) << "Recv. " << m_idx << ": "
				      << "got incomplete data";
	if (image_data->validPortData.none())
		return NULL;
	DEB_TRACE() << DEB_VAR5(m_idx, header.frameNumber, header.modId,
				header.row, header.column);
	FrameType recv_frame = header.frameNumber;
	if (recv_frame > m_cam->m_det_nb_frames)
		THROW_HW_ERROR(Error) << "Invalid frame: " 
				      << DEB_VAR3(m_idx, recv_frame,
						  DebHex(recv_frame));
	return image_data;
}

AutoPtr<Receiver::ImagePackets> Receiver::readImagePackets()
{
	DEB_MEMBER_FUNCT();

	Timestamp t0 = Timestamp::now();

	if (m_stats.last_t0.isSet())
		m_stats.stats.cb_period.add(t0 - m_stats.last_t0);
	if (m_stats.last_t1.isSet())
		m_stats.stats.recv_exec.add(t0 - m_stats.last_t1);
	m_stats.last_t0 = t0;

	AutoPtr<ImagePackets> image_data;
	try {
		image_data = readSkippableImagePackets();
		if (!image_data)
			return NULL;

		FrameType det_frame = image_data->detFrame();
		bool skip_this = false;
		bool skip_next = false;
		bool prev_last_skipped = m_last_skipped;
		FrameType skip_freq = m_cam->m_skip_frame_freq;
		if (skip_freq) {
			skip_this = (det_frame % (skip_freq + 1) == 0);
			FrameType last_frame = m_cam->m_det_nb_frames;
			skip_next = ((det_frame + 1) == last_frame);
			if (skip_this && (det_frame == last_frame))
				m_last_skipped = true;
			DEB_TRACE() << DEB_VAR5(m_idx, det_frame, skip_this,
						skip_next, m_last_skipped);
		}

		if (skip_this) {
			image_data = readSkippableImagePackets();
			if (!image_data)
				return NULL;
			det_frame = image_data->detFrame();
		}

		image_data->frame = det_frame - 1; // first frame is set to 1
		if (skip_freq)
			image_data->frame -= det_frame / (skip_freq + 1);

		if (skip_next && !m_last_skipped) {
			AutoPtr<ImagePackets> skip = readSkippableImagePackets();
			if (!skip)
				return NULL;
			m_last_skipped = true;
		}
		if (m_last_skipped && !prev_last_skipped)
			m_cam->processLastSkippedFrame(m_idx);
	} catch (Exception& e) {
		ostringstream name;
		name << "Receiver::getImage: " << DEB_VAR1(m_idx);
		m_cam->reportException(e, name.str());
		return NULL;
	}

	Timestamp t1 = Timestamp::now();
	m_stats.stats.cb_exec.add(t1 - t0);
	m_stats.last_t1 = t1;

	return image_data;
}

bool Receiver::asmImagePackets(ImagePackets *image_data, char *buffer)
{
	DEB_MEMBER_FUNCT();
	sls::FrameAssembler::Result res;
	MPFrameAssemblerPtr::element_type *a = m_asm_impl->m_asm.get();
	AnyPacketBlockList blks = std::move(RecvImagePacketBlocks(image_data));
	res = a->assembleFrame(blks, buffer);
	image_data->numberOfPorts = res.nb_ports;
	image_data->validPortData = res.valid_data;
	bool got_data = image_data->validPortData.any();
	DEB_RETURN() << DEB_VAR1(got_data);
	return got_data;
}

void Receiver::fillBadFrame(FrameType frame, char *buf)
{
	DEB_MEMBER_FUNCT();
	THROW_HW_ERROR(NotSupported) << DEB_VAR2(m_idx, frame) << ": "
				     << "Not implemented yet";
}

void Receiver::clearAllBuffers()
{
	DEB_MEMBER_FUNCT();
	m_recv->clearAllBuffers();
}

