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

namespace SlsDetector
{

%TypeHeaderCode
#include "SlsDetectorModel.h"
%End

class Model
{

public:
	Model(SlsDetector::Camera *cam, SlsDetector::Type type);
	virtual ~Model();

	virtual void getFrameDim(FrameDim& frame_dim /Out/,
				 bool raw = false) = 0;
	virtual void getAcqFrameDim(FrameDim& frame_dim /Out/,
				    bool raw = false);

	virtual void getDetMap(Data& det_map /Out/) = 0;

	SlsDetector::Camera *getCamera();
	SlsDetector::Type getType();

	virtual void setNbUDPInterfaces(int  nb_udp_ifaces);
	virtual void getNbUDPInterfaces(int& nb_udp_ifaces /Out/);

	virtual std::string getName() = 0;
	virtual void getPixelSize(double& x_size /Out/,
				  double& y_size /Out/) = 0;

	virtual void getDACInfo(std::vector<std::string>& name_list /Out/, 
				std::vector<int>& idx_list /Out/, 
				std::vector<int>& milli_volt_list /Out/) = 0;
	virtual void getADCInfo(std::vector<std::string>& name_list /Out/, 
				std::vector<int>& idx_list /Out/,
				std::vector<double>& factor_list /Out/,
				std::vector<double>& min_val_list /Out/) = 0;

	virtual void getTimeRanges(SlsDetector::TimeRanges& 
		     		   time_ranges /Out/) = 0;

	virtual bool checkTrigMode(SlsDetector::Defs::TrigMode trig_mode) = 0;

	virtual bool isXferActive() = 0;

	virtual SlsDetector::Reconstruction *getReconstruction();

protected:
	void updateCameraModel();
	void updateCameraImageSize();
	void updateCameraTimeRanges();

	virtual void updateImageSize() = 0;

	void putCmd(const std::string& s, int idx = -1);
	std::string getCmd(const std::string& s, int idx = -1);

	virtual 
	bool checkSettings(SlsDetector::Defs::Settings settings) = 0;

	virtual void prepareAcq() = 0;
	virtual void startAcq() = 0;
	virtual void stopAcq() = 0;
};



}; // namespace SlsDetector
