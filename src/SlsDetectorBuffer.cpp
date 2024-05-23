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


BufferMgr::BufferMgr(Camera *cam)
	: m_cam(cam), m_cond(cam->m_cond), m_resize_policy(Max),
	  m_buffer_ctrl_obj(NULL), m_max_memory(30)
{
	DEB_CONSTRUCTOR();
}

void BufferMgr::setBufferCtrlObj(BufferCtrlObj *buffer_ctrl_obj)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR2(m_buffer_ctrl_obj, buffer_ctrl_obj);
	m_buffer_ctrl_obj = buffer_ctrl_obj;
	m_buffer_sync = buffer_ctrl_obj ?
				buffer_ctrl_obj->getBufferSync(m_cond) : NULL;
}

bool BufferMgr::waitFrame(FrameType frame_nb)
{
	DEB_MEMBER_FUNCT();
	if (!m_buffer_ctrl_obj)
		THROW_HW_ERROR(Error) << "No BufferCbMgr defined";

	AutoMutex l(m_cond.mutex());
	BufferSync::Status status = m_buffer_sync->wait(frame_nb);
	switch (status) {
	case BufferSync::AVAILABLE:
		return true;
	case BufferSync::INTERRUPTED:
		return false;
	default:
		THROW_HW_ERROR(Error) << "Buffer sync wait error: "
				      << status;
	}
}

void BufferMgr::setBufferCPUAffinity(CPUAffinity buffer_affinity)
{
	DEB_MEMBER_FUNCT();
	DEB_ALWAYS() << DEB_VAR1(buffer_affinity);
	DEB_ALWAYS() << "node_mask="
		     << NumaNodeMask::fromCPUMask(buffer_affinity.getMask());
	
	BufferCtrlObj *buffer = getBufferCtrlObj();
	if (buffer)
		buffer->setCPUAffinityMask(buffer_affinity.getMask());
	m_buffer_affinity = buffer_affinity;
}

void BufferMgr::setMaxMemory(short max_memory)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR1(max_memory);
	if ((max_memory <= 0) || (max_memory > 100))
		THROW_HW_ERROR(InvalidValue) << "Invalid " << DEB_VAR1(max_memory);
	m_max_memory = max_memory;
}

void BufferMgr::getMaxMemory(short& max_memory)
{
	DEB_MEMBER_FUNCT();
	max_memory = m_max_memory;
	DEB_RETURN() << DEB_VAR1(max_memory);
}

void BufferMgr::getMaxNbBuffers(long& nb_buffers)
{
	DEB_MEMBER_FUNCT();
	bool raw_mode;
	m_cam->getRawMode(raw_mode);
	FrameDim frame_dim;
	Model *model = m_cam->getModel();
	model->getAcqFrameDim(frame_dim, raw_mode);
	int max_nb_buffers = GetDefMaxNbBuffers(frame_dim);
	nb_buffers = int(max_nb_buffers * m_max_memory / 100.0);
	PixelDepth pixel_depth;
	m_cam->getPixelDepth(pixel_depth);
	if (pixel_depth == PixelDepth4)
		nb_buffers *= 2;
	DEB_TRACE() << DEB_VAR2(max_nb_buffers, nb_buffers);
	DEB_RETURN() << DEB_VAR1(nb_buffers);
}

void BufferMgr::setResizePolicy(ResizePolicy resize_policy)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR1(resize_policy);
	m_resize_policy = resize_policy;
}

void BufferMgr::getResizePolicy(ResizePolicy& resize_policy)
{
	DEB_MEMBER_FUNCT();
	resize_policy = m_resize_policy;
	DEB_RETURN() << DEB_VAR1(resize_policy);
}

void BufferMgr::setPacketFifoDepth(int fifo_depth)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR1(fifo_depth);
	if (m_resize_policy != Manual)
		THROW_HW_ERROR(Error) << "Can only set PacketFifoDepth "
				      << "with Manual ResizePolicy";
	const int min_nb_buffers = 128;
	long max_nb_buffers;
	getMaxNbBuffers(max_nb_buffers);
	if ((fifo_depth < min_nb_buffers) || (fifo_depth > max_nb_buffers))
		THROW_HW_ERROR(Error) << "Invalid " << DEB_VAR1(fifo_depth)
				      << ", must be within "
				      << DEB_VAR2(min_nb_buffers,
						  max_nb_buffers);
	m_cam->setReceiverFifoDepth(fifo_depth);
}

void BufferMgr::getPacketFifoDepth(int& fifo_depth)
{
	DEB_MEMBER_FUNCT();
	m_cam->getReceiverFifoDepth(fifo_depth);
	DEB_RETURN() << DEB_VAR1(fifo_depth);
}

void BufferMgr::prepareAcq()
{
	DEB_MEMBER_FUNCT();

	if ((m_resize_policy == Auto) || (m_resize_policy == Max)) {
		FrameType nb_frames;
		m_cam->getNbFrames(nb_frames);
		long max_nb_buffers;
		getMaxNbBuffers(max_nb_buffers);
		int nb_buffers = max_nb_buffers;
		if ((m_resize_policy == Auto) && (nb_frames < max_nb_buffers))
			nb_buffers = nb_frames;
		const int min_nb_buffers = 128;
		if (nb_buffers < min_nb_buffers)
			nb_buffers = min_nb_buffers;
		m_cam->setReceiverFifoDepth(nb_buffers);
	}
}

void BufferMgr::releaseBuffers()
{
	DEB_MEMBER_FUNCT();
	bool prev_release_unused;
//	BufferCtrlObj::getBufferMgrResizePolicy(prev_release_unused);
//	BufferCtrlObj::setBufferMgrResizePolicy(true);
	if (m_buffer_ctrl_obj)
		m_buffer_ctrl_obj->releaseBuffers();
//	BufferCtrlObj::setBufferMgrResizePolicy(prev_release_unused);
}

void BufferMgr::clearAllBuffers()
{
	DEB_MEMBER_FUNCT();
	StdBufferCbMgr *buffer = getBufferCbMgr();
	if (buffer)
		buffer->clearAllBuffers();

	Camera::RecvList::iterator it, end = m_cam->m_recv_list.end();
	for (it = m_cam->m_recv_list.begin(); it != end; ++it)
		(*it)->clearAllBuffers();
}


