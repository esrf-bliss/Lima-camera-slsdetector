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

using namespace FrameAssembler;

struct RecvImageData : Receiver::ImageData {
	AnyPacketBlockList blocks;
};

inline
AnyPacketBlockList& RecvImageDataBlocks(Receiver::ImageData *image_data) {
	RecvImageData *rd = static_cast<RecvImageData *>(image_data);
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

AutoPtr<Receiver::ImageData> Receiver::readSkippableImagePackets()
{
	DEB_MEMBER_FUNCT();
	AutoPtr<ImageData> image_data = new RecvImageData();
	AnyPacketBlockList& blocks = RecvImageDataBlocks(image_data);
	sls_detector_header& header = image_data->header.detHeader;
	header.frameNumber = -1;
	blocks = std::move(m_recv->GetFramePacketBlocks());
	image_data->numberOfPorts = blocks.size();
	for (int i = 0; i < image_data->numberOfPorts; ++i) {
		std::visit([&](auto &block) {
			bool valid = block;
			image_data->validPortData[i] = valid;
			if (valid && (header.frameNumber == uint64_t(-1))) {
				for (int p = 0; p < (*block).NbPackets; ++p) {
					if ((*block)[p].valid()) {
						auto packet = (*block)[p];
						header = *packet.networkHeader();
						break;
					}
				}
			}
		    }, blocks[i]);
	}
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

Receiver::ImageData *Receiver::readImagePackets()
{
	DEB_MEMBER_FUNCT();

	Timestamp t0 = Timestamp::now();

	if (m_stats.last_t0.isSet())
		m_stats.stats.cb_period.add(t0 - m_stats.last_t0);
	if (m_stats.last_t1.isSet())
		m_stats.stats.recv_exec.add(t0 - m_stats.last_t1);
	m_stats.last_t0 = t0;

	AutoPtr<ImageData> image_data;
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
			AutoPtr<ImageData> skip = readSkippableImagePackets();
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

	return image_data.forget();
}

bool Receiver::asmImagePackets(ImageData *image_data, char *buffer)
{
	DEB_MEMBER_FUNCT();
	FrameAssembler::Result res;
	MPFrameAssemblerPtr::pointer a = m_asm_impl->m_asm.get();
	res = a->assembleFrame(std::move(RecvImageDataBlocks(image_data)),
			       &image_data->header, buffer);
	image_data->numberOfPorts = res.nb_ports;
	image_data->validPortData = res.valid_data;
	bool got_data = image_data->validPortData.any();
	DEB_RETURN() << DEB_VAR1(got_data);
	return got_data;
}

void Receiver::clearAllBuffers()
{
	m_recv->clearAllBuffers();
}

