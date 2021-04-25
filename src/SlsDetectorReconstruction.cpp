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
	: m_cam(cam), m_proxy(NULL), m_active(false)
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
	if (m_proxy)
		m_proxy->reconstructionChange(this);
}

void Reconstruction::getActive(bool& active)
{
	DEB_MEMBER_FUNCT();
	active = m_active;
	DEB_RETURN() << DEB_VAR1(active);
}

void Reconstruction::prepare()
{
	DEB_MEMBER_FUNCT();
	AutoMutex l(m_mutex);
	m_frame_packet_map.clear();
}

void Reconstruction::cleanUp()
{
	DEB_MEMBER_FUNCT();
	AutoMutex l(m_mutex);
	m_frame_packet_map.clear();
}

void Reconstruction::addFramePackets(DetFrameImagePackets&& det_frame_packets)
{
	DEB_MEMBER_FUNCT();
	AutoMutex l(m_mutex);
	m_frame_packet_map.emplace(std::move(det_frame_packets));
}

Data Reconstruction::process(Data& data)
{
	DEB_MEMBER_FUNCT();

	FrameType frame = data.frameNumber;
	AutoMutex l(m_mutex);
	FramePacketMap::iterator it = m_frame_packet_map.find(frame);
	if (it == m_frame_packet_map.end())
		return data;

	DetFrameImagePackets det_frame_packets = std::move(*it);
	m_frame_packet_map.erase(it);
	l.unlock();

	m_cam->assemblePackets(std::move(det_frame_packets));

	return m_active ? processModel(data) : data;
}


