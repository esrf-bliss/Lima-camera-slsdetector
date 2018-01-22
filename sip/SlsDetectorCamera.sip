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
#include "SlsDetectorCamera.h"
%End

class Camera : public HwMaxImageSizeCallbackGen, public EventCallbackGen
{
public:
	Camera(std::string config_fname);
	virtual ~Camera();

	SlsDetector::Type getType();
	SlsDetector::Model *getModel();
	std::vector<std::string> getHostnameList();
	int getNbDetModules();
	int getNbDetSubModules();
	int getPortIndex(int recv_idx, int port);

	void setBufferCbMgr(StdBufferCbMgr *buffer_cb_mgr);

	void setPixelDepth(SlsDetector::PixelDepth  pixel_depth);
	void getPixelDepth(SlsDetector::PixelDepth& pixel_depth /Out/);

	void setRawMode(bool  raw_mode);
	void getRawMode(bool& raw_mode /Out/);

	SlsDetector::State getState();
	void waitState(SlsDetector::State state);
	SlsDetector::State waitNotState(SlsDetector::State state);

	void getFrameDim(FrameDim& frame_dim /Out/, bool raw = false);

	const SlsDetector::FrameMap& getFrameMap();

	void putCmd(const std::string& s, int idx = -1);
	std::string getCmd(const std::string& s, int idx = -1);

	int getFramesCaught();
	SlsDetector::Defs::DetStatus getDetStatus();

	void setTrigMode(SlsDetector::Defs::TrigMode  trig_mode);
	void getTrigMode(SlsDetector::Defs::TrigMode& trig_mode /Out/);
	void setNbFrames(unsigned long  nb_frames);
	void getNbFrames(unsigned long& nb_frames /Out/);
	void setExpTime(double  exp_time);
	void getExpTime(double& exp_time /Out/);
	void setLatTime(double  lat_time);
	void getLatTime(double& lat_time /Out/);
	void setFramePeriod(double  frame_period);
	void getFramePeriod(double& frame_period /Out/);

	// setDAC: sub_mod_idx: 0-N=module, -1=all
	void setDAC(int sub_mod_idx, SlsDetector::Defs::DACIndex dac_idx,
		    int  val,       bool milli_volt = false);
	void getDAC(int sub_mod_idx, SlsDetector::Defs::DACIndex dac_idx,
	     	    int& val /Out/, bool milli_volt = false);
	void getDACList(SlsDetector::Defs::DACIndex dac_idx,
			std::vector<int>& val_list /Out/,
			bool milli_volt = false);

	void getADC(int sub_mod_idx, SlsDetector::Defs::ADCIndex adc_idx,
		    int& val /Out/);
	void getADCList(SlsDetector::Defs::ADCIndex adc_idx,
		        std::vector<int>& val_list /Out/);

	void setAllTrimBits(int sub_mod_idx, int  val);
	void getAllTrimBits(int sub_mod_idx, int& val /Out/);
	void getAllTrimBitsList(std::vector<int>& val_list /Out/);

	void setSettings(SlsDetector::Defs::Settings  settings);
	void getSettings(SlsDetector::Defs::Settings& settings /Out/);
	void setThresholdEnergy(int  thres);
	void getThresholdEnergy(int& thres /Out/);

	void setClockDiv(SlsDetector::Defs::ClockDiv  clock_div);
	void getClockDiv(SlsDetector::Defs::ClockDiv& clock_div /Out/);
	void setReadoutFlags(SlsDetector::Defs::ReadoutFlags  flags);
	void getReadoutFlags(SlsDetector::Defs::ReadoutFlags& flags /Out/);
	void getValidReadoutFlags(std::vector<int>& flag_list /Out/,
	     			std::vector<std::string>& flag_name_list /Out/);

	void setTolerateLostPackets(bool  tol_lost_packets);
	void getTolerateLostPackets(bool& tol_lost_packets /Out/);
	void getBadFrameList(std::vector<int>& bad_frame_list /Out/);

	void prepareAcq();
	void startAcq();
	void stopAcq();

	void getStats(SlsDetector::Stats& stats /Out/);

	void setPixelDepthCPUAffinityMap(
		SlsDetector::PixelDepthCPUAffinityMap aff_map);
	void getPixelDepthCPUAffinityMap(
		SlsDetector::PixelDepthCPUAffinityMap& aff_map /Out/);

	SlsDetector::SystemCPUAffinityMgr::ProcessingFinishedEvent *
		getProcessingFinishedEvent();
};


}; // namespace SlsDetector
