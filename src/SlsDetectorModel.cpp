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


Model::Model(Camera *cam, Type type)
	: m_cam(cam), m_type(type), m_det(m_cam->m_det)
{
	DEB_CONSTRUCTOR();
	DEB_PARAM() << DEB_VAR1(type);

	m_nb_det_modules = m_cam->getNbDetModules();
	sls::Result<int> res;
	EXC_CHECK(res = m_det->getNumberofUDPInterfaces());
	const char *err_msg = "Numbers of UDP interfaces are different";
	EXC_CHECK(m_nb_udp_ifaces = res.tsquash(err_msg));
}

Model::~Model()
{
	DEB_DESTRUCTOR();

	if (m_cam)
		m_cam->setModel(NULL);
}

void Model::updateCameraModel()
{
	DEB_MEMBER_FUNCT();
	m_cam->setModel(this);
}

void Model::updateCameraImageSize()
{
	DEB_MEMBER_FUNCT();
	m_cam->updateImageSize();
}

void Model::setNbUDPInterfaces(int nb_udp_ifaces)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR1(nb_udp_ifaces);
	if (nb_udp_ifaces != m_nb_udp_ifaces)
		THROW_HW_ERROR(NotSupported)
			<< "Invalid number of UDP interfaces: " << nb_udp_ifaces
			<< ", only " << m_nb_udp_ifaces << " supported";
}

void Model::getNbUDPInterfaces(int& nb_udp_ifaces)
{
	DEB_MEMBER_FUNCT();
	nb_udp_ifaces = m_nb_udp_ifaces;
	DEB_RETURN() << DEB_VAR1(nb_udp_ifaces);
}


void Model::updateCameraTimeRanges()
{
	DEB_MEMBER_FUNCT();
	m_cam->updateTimeRanges();
}

void Model::putCmd(const string& s, int idx)
{
	DEB_MEMBER_FUNCT();
	m_cam->putCmd(s, idx);
}

string Model::getCmd(const string& s, int idx)
{
	DEB_MEMBER_FUNCT();
	return m_cam->getCmd(s, idx);
}

BufferMgr *Model::getBuffer()
{
	return m_cam->getBuffer();
}

void Model::getAcqFrameDim(FrameDim& frame_dim, bool raw)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR1(raw);
	getFrameDim(frame_dim, raw);
	DEB_RETURN() << DEB_VAR1(frame_dim);
}

#include "SlsDetectorReceiver.h"

bool Model::isAcqActive()
{
	DEB_MEMBER_FUNCT();
	bool acq_active = (m_cam->getDetStatus() == Defs::Running);
	DEB_RETURN() << DEB_VAR1(acq_active);
	return acq_active;
}

Reconstruction *Model::getReconstruction()
{
	DEB_MEMBER_FUNCT();
	return NULL;
}
