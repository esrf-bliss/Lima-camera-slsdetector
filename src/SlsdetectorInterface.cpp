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
#include "SlsdetectorInterface.h"

using namespace lima;
using namespace lima::Slsdetector;
using namespace std;

/*******************************************************************
 * \brief EventCallback constructor
 *******************************************************************/

Slsdetector::EventCallback::EventCallback(HwEventCtrlObj& ctrl_obj) 
	: m_ctrl_obj(ctrl_obj)
{
	DEB_CONSTRUCTOR();
}

Slsdetector::EventCallback::~EventCallback()
{
	DEB_DESTRUCTOR();
}

void Slsdetector::EventCallback::processEvent(Event *event)
{
	DEB_MEMBER_FUNCT();
	m_ctrl_obj.reportEvent(event);
}


/*******************************************************************
 * \brief DetInfoCtrlObj constructor
 *******************************************************************/

DetInfoCtrlObj::DetInfoCtrlObj()
{
	DEB_CONSTRUCTOR();
}

DetInfoCtrlObj::~DetInfoCtrlObj()
{
	DEB_DESTRUCTOR();
}

void DetInfoCtrlObj::getMaxImageSize(Size& max_image_size)
{
	DEB_MEMBER_FUNCT();
	FrameDim max_frame_dim(Size(1024, 1024), Bpp16);
	max_image_size = max_frame_dim.getSize();

}

void DetInfoCtrlObj::getDetectorImageSize(Size& det_image_size)
{
	DEB_MEMBER_FUNCT();
	FrameDim max_frame_dim(Size(1024, 1024), Bpp16);
	det_image_size = max_frame_dim.getSize();
}

void DetInfoCtrlObj::getDefImageType(ImageType& def_image_type)
{
	DEB_MEMBER_FUNCT();
	FrameDim max_frame_dim(Size(1024, 1024), Bpp16);
	def_image_type = max_frame_dim.getImageType();
}

void DetInfoCtrlObj::getCurrImageType(ImageType& curr_image_type)
{
	DEB_MEMBER_FUNCT();
	FrameDim max_frame_dim(Size(1024, 1024), Bpp16);
	curr_image_type = max_frame_dim.getImageType();
}

void DetInfoCtrlObj::setCurrImageType(ImageType curr_image_type)
{
	DEB_MEMBER_FUNCT();
	ImageType unique_image_type = Bpp16;
	if (curr_image_type != unique_image_type)
		THROW_HW_ERROR(InvalidValue) 
			<< "Only " << DEB_VAR1(unique_image_type) << "allowed";
}

void DetInfoCtrlObj::getPixelSize(double& x_size, double& y_size)
{
	DEB_MEMBER_FUNCT();
	x_size = y_size = 10e-6;
	DEB_RETURN() << DEB_VAR2(x_size, y_size);
}

void DetInfoCtrlObj::getDetectorType(std::string& det_type)
{
	DEB_MEMBER_FUNCT();
	det_type = "Slsdetector";
	DEB_RETURN() << DEB_VAR1(det_type);
}

void DetInfoCtrlObj::getDetectorModel(std::string& det_model)
{
	DEB_MEMBER_FUNCT();
	det_model = "Generic";
	DEB_RETURN() << DEB_VAR1(det_model);
}

void DetInfoCtrlObj::registerMaxImageSizeCallback(
					HwMaxImageSizeCallback& cb)
{
	DEB_MEMBER_FUNCT();
}

void DetInfoCtrlObj::unregisterMaxImageSizeCallback(
					HwMaxImageSizeCallback& cb)
{
	DEB_MEMBER_FUNCT();
}


/*******************************************************************
 * \brief SyncCtrlObj constructor
 *******************************************************************/

SyncCtrlObj::SyncCtrlObj()
	: HwSyncCtrlObj()
{
	DEB_CONSTRUCTOR();
}

SyncCtrlObj::~SyncCtrlObj()
{
	DEB_DESTRUCTOR();
}

bool SyncCtrlObj::checkTrigMode(TrigMode trig_mode)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR1(trig_mode);

	bool valid_mode;
	switch (trig_mode) {
	case IntTrig:
		valid_mode = true;
		break;

	default:
		valid_mode = false;
	}

	DEB_RETURN() << DEB_VAR1(valid_mode);
	return valid_mode;
}

void SyncCtrlObj::setTrigMode(TrigMode trig_mode)
{
	DEB_MEMBER_FUNCT();

	if (!checkTrigMode(trig_mode))
		THROW_HW_ERROR(InvalidValue) << "Invalid " 
					     << DEB_VAR1(trig_mode);
}

void SyncCtrlObj::getTrigMode(TrigMode& trig_mode)
{
	DEB_MEMBER_FUNCT();
	trig_mode = IntTrig;
}

void SyncCtrlObj::setExpTime(double exp_time)
{
	DEB_MEMBER_FUNCT();
}

void SyncCtrlObj::getExpTime(double& exp_time)
{
	DEB_MEMBER_FUNCT();
	exp_time = 1.0;
}

void SyncCtrlObj::setLatTime(double lat_time)
{
	DEB_MEMBER_FUNCT();
}

void SyncCtrlObj::getLatTime(double& lat_time)
{
	DEB_MEMBER_FUNCT();
	lat_time = 0.0;
}

void SyncCtrlObj::setNbHwFrames(int nb_frames)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR1(nb_frames);
}

void SyncCtrlObj::getNbHwFrames(int& nb_frames)
{
	DEB_MEMBER_FUNCT();
	nb_frames = 1;
}

void SyncCtrlObj::getValidRanges(ValidRangesType& valid_ranges)
{
	DEB_MEMBER_FUNCT();

	valid_ranges.min_exp_time = 1e-6;
	valid_ranges.max_exp_time = 1e6;

	valid_ranges.min_lat_time = 1e-6;
	valid_ranges.max_lat_time = 1e6;

	DEB_RETURN() << DEB_VAR2(valid_ranges.min_exp_time, 
				 valid_ranges.max_exp_time);
	DEB_RETURN() << DEB_VAR2(valid_ranges.min_lat_time, 
				 valid_ranges.max_lat_time);
}


/*******************************************************************
 * \brief EventCtrlObj constructor
 *******************************************************************/

EventCtrlObj::EventCtrlObj()
{
	DEB_CONSTRUCTOR();
}

EventCtrlObj::~EventCtrlObj()
{
	DEB_DESTRUCTOR();
}


/*******************************************************************
 * \brief Hw Interface constructor
 *******************************************************************/

Interface::Interface()
	: m_event_cb(m_event)
{
	DEB_CONSTRUCTOR();

	HwDetInfoCtrlObj *det_info = &m_det_info;
	m_cap_list.push_back(HwCap(det_info));

	HwBufferCtrlObj *buffer = &m_buffer;
	m_cap_list.push_back(HwCap(buffer));

	HwSyncCtrlObj *sync = &m_sync;
	m_cap_list.push_back(HwCap(sync));

	HwEventCtrlObj *event = &m_event;
	m_cap_list.push_back(HwCap(event));

	reset(SoftReset);
	resetDefaults();
}

Interface::~Interface()
{
	DEB_DESTRUCTOR();
}

void Interface::getCapList(HwInterface::CapList &cap_list) const
{
	DEB_MEMBER_FUNCT();
	cap_list = m_cap_list;
}

void Interface::reset(ResetLevel reset_level)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR1(reset_level);

	stopAcq();

	if (reset_level == HardReset) {
		DEB_TRACE() << "Performing camera hard reset";
		resetDefaults();
	}
}

void Interface::resetDefaults()
{
	DEB_MEMBER_FUNCT();
	DEB_TRACE() << "Setting default configuration";

	stopAcq();
}

void Interface::prepareAcq()
{
	DEB_MEMBER_FUNCT();
}

void Interface::startAcq()
{
	DEB_MEMBER_FUNCT();
}

void Interface::stopAcq()
{
	DEB_MEMBER_FUNCT();
}

void Interface::getStatus(StatusType& status)
{
	DEB_MEMBER_FUNCT();

	status.acq = AcqReady;
	status.det = DetIdle;

	DEB_RETURN() << DEB_VAR1(status);
}

int Interface::getNbHwAcquiredFrames()
{
	DEB_MEMBER_FUNCT();
	int nb_hw_acq_frames = 0;
	DEB_RETURN() << DEB_VAR1(nb_hw_acq_frames);
	return nb_hw_acq_frames;
}


