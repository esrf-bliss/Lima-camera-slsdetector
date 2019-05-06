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
#include "SlsDetectorEiger.h"
%End


class Eiger : public SlsDetector::Model
{

%TypeHeaderCode
#include "SlsDetectorEiger.h"
%End

 public:
	enum ParallelMode {
		NonParallel, Parallel, Safe,
	};

	class Correction : public LinkTask
	{
	public:
		Correction(SlsDetector::Eiger *eiger);

		virtual Data process(Data& data);
	};

	Eiger(SlsDetector::Camera *cam);

	virtual void getFrameDim(FrameDim& frame_dim /Out/, bool raw = false);
	SlsDetector::Eiger::Correction *createCorrectionTask() /Factory/;

	virtual std::string getName();
	virtual void getPixelSize(double& x_size /Out/, double& y_size /Out/);

	virtual void getDACInfo(std::vector<std::string>& name_list /Out/,
				std::vector<int>& idx_list /Out/, 
				std::vector<int>& milli_volt_list /Out/);
	virtual void getADCInfo(std::vector<std::string>& name_list /Out/,
				std::vector<int>& idx_list /Out/,
				std::vector<double>& factor_list /Out/,
				std::vector<double>& min_val_list /Out/);

	virtual void getTimeRanges(SlsDetector::TimeRanges& time_ranges /Out/);
	static void calcTimeRanges(SlsDetector::PixelDepth pixel_depth,
				   SlsDetector::Defs::ClockDiv clock_div,
				   SlsDetector::Eiger::ParallelMode parallel_mode,
				   SlsDetector::TimeRanges& time_ranges /Out/);

	void setParallelMode(SlsDetector::Eiger::ParallelMode  mode);
	void getParallelMode(SlsDetector::Eiger::ParallelMode& mode);

	void setFixedClockDiv(bool  fixed_clock_div);
	void getFixedClockDiv(bool& fixed_clock_div);
	void setClockDiv(SlsDetector::Defs::ClockDiv  clock_div);
	void getClockDiv(SlsDetector::Defs::ClockDiv& clock_div /Out/);
	void setSubExpTime(double  sub_exp_time);
	void getSubExpTime(double& sub_exp_time /Out/);

	void setAllTrimBits(int sub_mod_idx, int  val);
	void getAllTrimBits(int sub_mod_idx, int& val /Out/);
	void getAllTrimBitsList(std::vector<int>& val_list /Out/);

	void setHighVoltage(int  hvolt);
	void getHighVoltage(int& hvolt /Out/);

	void setThresholdEnergy(int  thres);
	void getThresholdEnergy(int& thres /Out/);

 protected:
	virtual int getNbFrameMapItems();
	virtual void updateFrameMapItems(FrameMap *map);
	virtual void processBadItemFrame(unsigned long frame, int item,
					 char *bptr);

	virtual void updateImageSize();

	virtual bool checkSettings(SlsDetector::Defs::Settings settings);

	virtual int getNbRecvs();
	virtual SlsDetector::Model::Recv *getRecv(int recv_idx);

	virtual void prepareAcq();
	virtual void startAcq();
	virtual void stopAcq();
};

}; // namespace SlsDetector
