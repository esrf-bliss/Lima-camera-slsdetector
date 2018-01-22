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
	: m_cam(cam), m_type(type)
{
	DEB_CONSTRUCTOR();
	DEB_PARAM() << DEB_VAR1(type);
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

