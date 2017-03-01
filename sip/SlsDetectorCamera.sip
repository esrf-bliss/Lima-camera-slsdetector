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

enum State {
	Idle, Init, Starting, Running, StopReq, Stopping, Stopped,
};

class Camera
{

%TypeHeaderCode
#include "SlsDetectorCamera.h"
%End

public:
	class FrameMap
	{
	public:
		typedef std::set<int> List;
		typedef std::map<int, std::set<int> > Map;

		class Callback
		{
		public:
			Callback();
			virtual ~Callback();
		protected:
			virtual void frameFinished(int frame) = 0;
		};

		FrameMap();
		~FrameMap();
		
		void setCallback(Callback *cb);
		void setNbItems(int nb_items);
		void clear();

		void frameItemFinished(int frame, int item);
		
		int getLastSeqFinishedFrame() const;

		const SlsDetector::Camera::FrameMap::List& 
		      getNonSeqFinishedFrames() const;

		const SlsDetector::Camera::FrameMap::Map&
		      getFramePendingItemsMap() const;
	};

	Camera(std::string config_fname);
	virtual ~Camera();

	void setBufferCbMgr(StdBufferCbMgr *buffer_cb_mgr);

	void setSaveRaw(bool save_raw);

	void setNbFrames(int  nb_frames);
	void getNbFrames(int& nb_frames /Out/);
	void setExpTime(double  exp_time);
	void getExpTime(double& exp_time /Out/);
	void setFramePeriod(double  frame_period);
	void getFramePeriod(double& frame_period /Out/);

	SlsDetector::State getState();
	void waitState(SlsDetector::State state);
	SlsDetector::State waitNotState(SlsDetector::State state);

	void prepareAcq();
	void startAcq();
	void stopAcq();

	int getNbHalfModules();

	void getFrameDim(FrameDim& frame_dim, bool raw = false);

	int getFramesCaught();
	std::string getStatus();

	const SlsDetector::Camera::FrameMap& getRecvMap();
};


}; // namespace SlsDetector
