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
#include "SlsDetectorJungfrau.h"
%End


class Jungfrau : public SlsDetector::Model
{

%TypeHeaderCode
#include "SlsDetectorJungfrau.h"
%End

 public:
	class ImgProcTask : public SinkTaskBase
	{
	public:
		ImgProcTask(SlsDetector::Jungfrau *jungfrau);

		void setConfig(std::string  config);
		void getConfig(std::string &config /Out/);

		virtual void process(Data& data);
	};

	Jungfrau(SlsDetector::Camera *cam);

	virtual void getFrameDim(FrameDim& frame_dim /Out/, bool raw = false);

	SlsDetector::Jungfrau::ImgProcTask *createImgProcTask() /Factory/;

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

	void setHighVoltage(int  hvolt);
	void getHighVoltage(int& hvolt /Out/);

	void setThresholdEnergy(int  thres);
	void getThresholdEnergy(int& thres /Out/);

	void setImgProcConfig(std::string  config);
	void getImgProcConfig(std::string &config /Out/);

	void readGainADCMaps(Data& gain_map /Out/, Data& adc_map /Out/,
			     unsigned long& frame /In,Out/);

	virtual bool isXferActive();

 protected:
	virtual int getNbFrameMapItems();
	virtual void updateFrameMapItems(FrameMap *map);
	virtual void processBadItemFrame(unsigned long frame, int item,
					 char *bptr);

	virtual void updateImageSize();

	virtual bool checkSettings(SlsDetector::Defs::Settings settings);

	virtual void setThreadCPUAffinity(const SlsDetector::CPUAffinityList&
						det_thread_aff_list);

	virtual void prepareAcq();
	virtual void startAcq();
	virtual void stopAcq();
};

}; // namespace SlsDetector
