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
	: m_cam(cam), m_cond(m_cam->m_cond), m_mode(Single),
	  m_lima_buffer_ctrl_obj(NULL),
	  m_max_memory(70)
{
	DEB_CONSTRUCTOR();
}

void BufferMgr::setMode(Mode mode)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR2(mode, m_mode);
	if (mode == m_mode)
		return;

	m_mode = mode;
	m_acq_buffer_ctrl_obj = (m_mode == Dual) ? new BufferCtrlObj() : NULL;
	DEB_TRACE() << DEB_VAR2(m_lima_buffer_ctrl_obj, m_acq_buffer_ctrl_obj);

	BufferCtrlObj *buffer;
	buffer = getBufferCtrlObj(AcqBuffer);
	if (buffer) {
		buffer->setType(AcqBuffer);
		buffer->setCPUAffinityMask(m_buffer_affinity);
	}
	buffer = getBufferCtrlObj(LimaBuffer);
	if (buffer && (m_mode == Dual)) {
		buffer->setType(LimaBuffer);
		buffer->setCPUAffinityMask(CPUAffinity());
	}

	Model *model = m_cam->m_model;
	if (!model)
		return;

	if (m_acq_buffer_ctrl_obj) {
		FrameDim frame_dim;
		model->getAcqFrameDim(frame_dim, m_cam->m_raw_mode);
		m_acq_buffer_ctrl_obj->setFrameDim(frame_dim);
	}
	m_cam->updateImageSize();
		
}

void BufferMgr::getMode(Mode& mode)
{
	DEB_MEMBER_FUNCT();
	mode = m_mode;
	DEB_RETURN() << DEB_VAR1(mode);
}

void BufferMgr::setLimaBufferCtrlObj(BufferCtrlObj *buffer_ctrl_obj)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR2(m_lima_buffer_ctrl_obj, buffer_ctrl_obj);
	m_lima_buffer_ctrl_obj = buffer_ctrl_obj;
	m_lima_buffer_sync = buffer_ctrl_obj ?
				buffer_ctrl_obj->getBufferSync(m_cond) : NULL;
}

void BufferMgr::waitLimaFrame(FrameType frame_nb, AutoMutex& l)
{
	DEB_MEMBER_FUNCT();
	if (!m_lima_buffer_ctrl_obj)
		THROW_HW_ERROR(Error) << "No Lima BufferCbMgr defined";
	while (true) {
		BufferSync::Status status = m_lima_buffer_sync->wait(frame_nb);
		switch (status) {
		case BufferSync::AVAILABLE:
			return;
		case BufferSync::INTERRUPTED:
			continue;
		default:
			THROW_HW_ERROR(Error) << "Lima buffer sync wait error: "
					      << status;
		}
	}
}

char *BufferMgr::getAcqFrameBufferPtr(FrameType frame_nb)
{
	DEB_MEMBER_FUNCT();

	StdBufferCbMgr *cb_mgr = getBufferCbMgr(AcqBuffer);
	if (!cb_mgr)
		THROW_HW_ERROR(InvalidValue) << "No BufferCbMgr defined";
	void *ptr = cb_mgr->getFrameBufferPtr(frame_nb);
	return static_cast<char *>(ptr);
}

void BufferMgr::setAcqBufferCPUAffinity(CPUAffinity buffer_affinity)
{
	DEB_MEMBER_FUNCT();
	DEB_ALWAYS() << DEB_VAR1(buffer_affinity);
	BufferCtrlObj *buffer = getBufferCtrlObj(AcqBuffer);
	if (buffer)
		buffer->setCPUAffinityMask(buffer_affinity);
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
	if (m_acq_buffer_ctrl_obj) {
		int max_nb_buffers;
		m_acq_buffer_ctrl_obj->getMaxNbBuffers(max_nb_buffers);
		nb_buffers = int(max_nb_buffers * m_max_memory / 100.0);
		DEB_TRACE() << DEB_VAR2(max_nb_buffers, nb_buffers);
	} else {
		nb_buffers = 0;
	}
	DEB_RETURN() << DEB_VAR1(nb_buffers);
}

void BufferMgr::prepareAcq()
{
	DEB_MEMBER_FUNCT();

	if (m_acq_buffer_ctrl_obj) {
		FrameType nb_frames;
		m_cam->getNbFrames(nb_frames);
		long max_nb_buffers;
		getMaxNbBuffers(max_nb_buffers);
		int nb_buffers = (nb_frames < max_nb_buffers) ? nb_frames :
								max_nb_buffers;
		m_acq_buffer_ctrl_obj->setNbBuffers(nb_buffers);
	}

}

void BufferMgr::releaseBuffers()
{
	DEB_MEMBER_FUNCT();
	bool prev_release_unused;
//	BufferCtrlObj::getBufferMgrResizePolicy(prev_release_unused);
//	BufferCtrlObj::setBufferMgrResizePolicy(true);
	if (m_acq_buffer_ctrl_obj)
		m_acq_buffer_ctrl_obj->releaseBuffers();
	if (m_lima_buffer_ctrl_obj)
		m_lima_buffer_ctrl_obj->releaseBuffers();
//	BufferCtrlObj::setBufferMgrResizePolicy(prev_release_unused);
}

void BufferMgr::clearAllBuffers()
{
	DEB_MEMBER_FUNCT();
	StdBufferCbMgr *buffer;
	buffer = getBufferCbMgr(AcqBuffer);
	if (buffer)
		buffer->clearAllBuffers();
	buffer = getBufferCbMgr(LimaBuffer);
	if (buffer && (m_mode == Dual))
		buffer->clearAllBuffers();

	Camera::RecvList::iterator it, end = m_cam->m_recv_list.end();
	for (it = m_cam->m_recv_list.begin(); it != end; ++it)
		(*it)->clearAllBuffers();
}


