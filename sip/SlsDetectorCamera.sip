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

%TypeHeaderCode
#include "SlsDetectorCamera.h"
%End

public:
	enum State {
		Idle, Init, Starting, Running, StopReq, Stopping, Stopped,
	};

	enum Type {
		GenericDet, EigerDet, JungfrauDet,
	};

	class Model
	{
	public:
		Model(SlsDetector::Camera *cam, SlsDetector::Camera::Type type);
		virtual ~Model();

		virtual void getFrameDim(FrameDim& frame_dim /Out/,
					 bool raw = false) = 0;

		SlsDetector::Camera *getCamera();
		SlsDetector::Camera::Type getType();

		virtual std::string getName() = 0;
		virtual void getPixelSize(double& x_size /Out/,
					  double& y_size /Out/) = 0;

		virtual 
		void getDACInfo(std::vector<std::string>& name_list /Out/, 
				std::vector<int>& idx_list /Out/, 
				std::vector<int>& milli_volt_list /Out/) = 0;
		virtual 
		void getADCInfo(std::vector<std::string>& name_list /Out/, 
				std::vector<int>& idx_list /Out/,
				std::vector<double>& factor_list /Out/,
				std::vector<double>& min_val_list /Out/) = 0;

	protected:
		void putCmd(const std::string& s, int idx = -1);
		std::string getCmd(const std::string& s, int idx = -1);

		virtual 
		bool checkSettings(SlsDetector::Defs::Settings settings) = 0;

		virtual int getRecvPorts() = 0;

		virtual void prepareAcq() = 0;
		virtual void processRecvFileStart(int recv_idx, 
						  unsigned int dsize) = 0;
		virtual void processRecvPort(int recv_idx, unsigned long frame,
					     int port, char *dptr, 
					     unsigned int dsize,
					     Mutex& lock, char *bptr) = 0;
	};

	class FrameMap
	{
	public:
		typedef std::set<int> List;
		typedef std::map<unsigned long, std::set<int> > Map;

		class Callback
		{
		public:
			Callback();
			virtual ~Callback();
		protected:
			virtual void frameFinished(unsigned long frame) = 0;
		};

		FrameMap(bool debug = false);
		~FrameMap();
		
		void setCallback(Callback *cb);
		void setNbItems(int nb_items);
		void clear();

		void checkFinishedFrameItem(unsigned long frame, int item);
		void frameItemFinished(unsigned long frame, int item, 
				       bool no_check = false);
		
		unsigned long getLastSeqFinishedFrame() const;

		const SlsDetector::Camera::FrameMap::List& 
		      getNonSeqFinishedFrames() const;

		const SlsDetector::Camera::FrameMap::Map&
		      getFramePendingItemsMap() const;

		bool isValidFrame(unsigned long frame);
	};

	Camera(std::string config_fname);
	virtual ~Camera();

	SlsDetector::Camera::Type getType();
	SlsDetector::Camera::Model *getModel();
	std::vector<std::string> getHostnameList();
	int getNbDetModules();
	int getNbDetSubModules();

	void setBufferCbMgr(StdBufferCbMgr *buffer_cb_mgr);

	void setRawMode(bool  raw_mode);
	void getRawMode(bool& raw_mode /Out/);

	SlsDetector::Camera::State getState();
	void waitState(SlsDetector::Camera::State state);
	SlsDetector::Camera::State waitNotState(
				   SlsDetector::Camera::State state);

	void getFrameDim(FrameDim& frame_dim, bool raw = false);

	const SlsDetector::Camera::FrameMap& getRecvMap();

	void putCmd(const std::string& s, int idx = -1);
	std::string getCmd(const std::string& s, int idx = -1);

	int getFramesCaught();
	std::string getStatus();

	void setTrigMode(SlsDetector::Defs::TrigMode  trig_mode);
	void getTrigMode(SlsDetector::Defs::TrigMode& trig_mode /Out/);
	void setNbFrames(unsigned long  nb_frames);
	void getNbFrames(unsigned long& nb_frames /Out/);
	void setExpTime(double  exp_time);
	void getExpTime(double& exp_time /Out/);
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

	void setSettings(SlsDetector::Defs::Settings  settings);
	void getSettings(SlsDetector::Defs::Settings& settings /Out/);
	void setThresholdEnergy(int  thres);
	void getThresholdEnergy(int& thres /Out/);

	void prepareAcq();
	void startAcq();
	void stopAcq();
};


}; // namespace SlsDetector
