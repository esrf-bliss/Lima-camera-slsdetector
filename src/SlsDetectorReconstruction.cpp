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
#include "SlsDetectorReconstruction.h"

using namespace lima;
using namespace lima::SlsDetector;
using namespace std;

#include "SlsDetectorCamera.h"

/*******************************************************************
 * \brief ReconstructionCtrlObj constructor
 *******************************************************************/

Reconstruction::CtrlObjProxy::CtrlObjProxy(Reconstruction *r)
{
	DEB_CONSTRUCTOR();
	if (!r)
		THROW_HW_ERROR(InvalidValue) << "Invalid NULL reconstruction";
	else if (r->m_proxy)
		THROW_HW_ERROR(InvalidValue) << "Reconstruction has a proxy";

	m_r = r;
	m_r->m_proxy = this;
}

Reconstruction::CtrlObjProxy::~CtrlObjProxy()
{
	DEB_DESTRUCTOR();
	if (m_r)
		m_r->m_proxy = NULL;
}

LinkTask *Reconstruction::CtrlObjProxy::getReconstructionTask()
{
	DEB_MEMBER_FUNCT();
	LinkTask *task = m_r ? m_r : NULL;
	DEB_RETURN() << DEB_VAR1(task);
	return task;
}

Reconstruction::Reconstruction(Camera *cam)
	: m_cam(cam), m_proxy(NULL), m_active(false),
	  m_lima_buffer_mode(RawData)
{
	DEB_CONSTRUCTOR();
}

Reconstruction::~Reconstruction()
{
	DEB_DESTRUCTOR();
	if (m_proxy)
		m_proxy->m_r = NULL;
}

void Reconstruction::setActive(bool active)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR2(active, m_active);
	m_active = active;
}

void Reconstruction::getActive(bool& active)
{
	DEB_MEMBER_FUNCT();
	active = m_active;
	DEB_RETURN() << DEB_VAR1(active);
}

void Reconstruction::setLimaBufferMode(LimaBufferMode lima_buffer_mode)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR2(lima_buffer_mode, m_lima_buffer_mode);
	m_lima_buffer_mode = lima_buffer_mode;
}

void Reconstruction::getLimaBufferMode(LimaBufferMode& lima_buffer_mode)
{
	DEB_MEMBER_FUNCT();
	lima_buffer_mode = m_lima_buffer_mode;
	DEB_RETURN() << DEB_VAR1(lima_buffer_mode);
}

void Reconstruction::prepare()
{
	DEB_MEMBER_FUNCT();

	Model *model = m_cam->getModel();
	if (!model)
		THROW_HW_ERROR(Error) << "Camera has no model";
	model->getAcqFrameDim(m_raw_frame_dim, m_cam->m_raw_mode);
	
	AutoMutex l(m_mutex);
	m_stopped = false;
	if (!m_frame_packet_map.empty()) {
		DEB_ERROR() << "m_frame_packet_map not empty!";
		AutoMutexUnlock u(l);
		m_frame_packet_map.clear();
	}
}

Data Reconstruction::getRawData(Data& data)
{
	DEB_MEMBER_FUNCT();

	if (m_lima_buffer_mode == RawData)
		return data;

	static pthread_key_t thread_data_key;
	EXEC_ONCE(pthread_key_create(&thread_data_key, &releaseThreadData));

	ThreadData *d = (ThreadData *) pthread_getspecific(thread_data_key);
	if (d == NULL) {
		d = new ThreadData();
		pthread_setspecific(thread_data_key, d);
		d->ptr = NULL;
		d->size = 0;
	}
	long size = m_raw_frame_dim.getMemSize();
	if (d->size != size) {
		void *p = realloc(d->ptr, size);
		if (!p)
			THROW_HW_ERROR(Error) << "Cannot re-allocate "
					      << size << " bytes";
		d->ptr = p;
		d->size = size;
	}

	return GetMappedData(d->ptr, m_raw_frame_dim);
}

void Reconstruction::releaseThreadData(void *thread_data)
{
	ThreadData *d = (ThreadData *) thread_data;
	free(d->ptr);
	delete d;
}

void Reconstruction::stop()
{
	DEB_MEMBER_FUNCT();
	AutoMutex l(m_mutex);
	if (m_stopped)
		return;
	m_stopped = true;
	l.unlock();
	m_frame_packet_map.clear();
}

bool Reconstruction::addFramePackets(DetFrameImagePackets det_frame_packets)
{
	DEB_MEMBER_FUNCT();
	AutoMutex l(m_mutex);
	if (m_stopped)
		return false;
	m_frame_packet_map.emplace(std::move(det_frame_packets));
	return true;
}

Data Reconstruction::process(Data& data)
{
	DEB_MEMBER_FUNCT();

	AutoMutex l(m_mutex);
	if (m_stopped)
		return data;
	FrameType frame = data.frameNumber;
	FramePacketMap::iterator it = m_frame_packet_map.find(frame);
	if (it == m_frame_packet_map.end())
		return data;

	DetFrameImagePackets det_frame_packets = std::move(*it);
	m_frame_packet_map.erase(it);
	l.unlock();

	m_cam->assemblePackets(std::move(det_frame_packets));

	return m_active ? processModel(data) : data;
}


