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
	Camera(std::string config_fname, int det_id = 0);
	virtual ~Camera();

	SlsDetector::Type getType();
	SlsDetector::Model *getModel();
	std::vector<std::string> getHostnameList();
	int getNbDetModules();

//	void setBufferCtrlObj(NumaSoftBufferCtrlObj *buffer_ctrl_obj);

	void setModuleActive(int mod_idx, bool  active);
	void getModuleActive(int mod_idx, bool& active /Out/);

	void clearAllBuffers();

	void setPixelDepth(SlsDetector::PixelDepth  pixel_depth);
	void getPixelDepth(SlsDetector::PixelDepth& pixel_depth /Out/);

	void setRawMode(bool  raw_mode);
	void getRawMode(bool& raw_mode /Out/);

	SlsDetector::AcqState getAcqState();
	void waitAcqState(SlsDetector::AcqState state);
	SlsDetector::AcqState waitNotAcqState(SlsDetector::AcqState state);

	void getFrameDim(FrameDim& frame_dim /Out/, bool raw = false);

	SlsDetector::FrameMap *getFrameMap();

	void putCmd(const std::string& s, int idx = -1);
	std::string getCmd(const std::string& s, int idx = -1);

	int getFramesCaught();
	SlsDetector::Defs::DetStatus getDetStatus();
	SlsDetector::Defs::DetStatus getDetTrigStatus();

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

	void setSkipFrameFreq(unsigned long  skip_frame_freq);
	void getSkipFrameFreq(unsigned long& skip_frame_freq /Out/);

	// setDAC: mod_idx: 0-N=module, -1=all
	void setDAC(int mod_idx, SlsDetector::Defs::DACIndex dac_idx,
		    int  val,       bool milli_volt = false);
	void getDAC(int mod_idx, SlsDetector::Defs::DACIndex dac_idx,
	     	    int& val /Out/, bool milli_volt = false);
	void getDACList(SlsDetector::Defs::DACIndex dac_idx,
			std::vector<int>& val_list /Out/,
			bool milli_volt = false);

	void getADC(int mod_idx, SlsDetector::Defs::ADCIndex adc_idx,
		    int& val /Out/);
	void getADCList(SlsDetector::Defs::ADCIndex adc_idx,
		        std::vector<int>& val_list /Out/);

	void setSettings(SlsDetector::Defs::Settings  settings);
	void getSettings(SlsDetector::Defs::Settings& settings /Out/);

	void setTolerateLostPackets(bool  tol_lost_packets);
	void getTolerateLostPackets(bool& tol_lost_packets /Out/);

	int getNbBadFrames(int recv_idx);
	void getBadFrameList(int recv_idx, int first_idx, int last_idx,
			     std::vector<int>& bad_frame_list /Out/);
	void getBadFrameList(int recv_idx, 
			     std::vector<int>& bad_frame_list /Out/);

	void prepareAcq();
	void startAcq();
	void stopAcq();

	void triggerFrame();

	void getStats(SlsDetector::Stats& stats /Out/, int recv_idx=-1);

	void setPixelDepthCPUAffinityMap(
		SlsDetector::PixelDepthCPUAffinityMap aff_map);
	void getPixelDepthCPUAffinityMap(
		SlsDetector::PixelDepthCPUAffinityMap& aff_map /Out/);

	SlsDetector::GlobalCPUAffinityMgr::ProcessingFinishedEvent *
		getProcessingFinishedEvent();

 private:
	Camera(const SlsDetector::Camera& o);
};


}; // namespace SlsDetector
