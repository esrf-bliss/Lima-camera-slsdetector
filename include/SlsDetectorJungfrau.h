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

#ifndef __SLS_DETECTOR_JUNGFRAU_H
#define __SLS_DETECTOR_JUNGFRAU_H

#include "SlsDetectorCamera.h"

#include "processlib/LinkTask.h"

namespace lima 
{

namespace SlsDetector
{

class Jungfrau : public Model
{
	DEB_CLASS_NAMESPC(DebModCamera, "Jungfrau", "SlsDetector");

 public:
	typedef unsigned char Byte;
	typedef unsigned short Word;
	typedef unsigned int Long;

	Jungfrau(Camera *cam);
	~Jungfrau();

	virtual void getFrameDim(FrameDim& frame_dim, bool raw = false);

	virtual std::string getName();
	virtual void getPixelSize(double& x_size, double& y_size);

	virtual void getDACInfo(NameList& name_list, IntList& idx_list,
				IntList& milli_volt_list);
	virtual void getADCInfo(NameList& name_list, IntList& idx_list,
				FloatList& factor_list, 
				FloatList& min_val_list);

	virtual void getTimeRanges(TimeRanges& time_ranges);

	void setHighVoltage(int  hvolt);
	void getHighVoltage(int& hvolt);

	void setThresholdEnergy(int  thres);
	void getThresholdEnergy(int& thres);

	virtual bool isXferActive();

 protected:
	virtual int getNbFrameMapItems();
	virtual void updateFrameMapItems(FrameMap *map);
	virtual void processBadItemFrame(FrameType frame, int item,
					 char *bptr);

	virtual void setThreadCPUAffinity(const CPUAffinityList& aff_list);

	virtual void updateImageSize();

	virtual bool checkSettings(Settings settings);

	virtual void prepareAcq();
	virtual void startAcq();
	virtual void stopAcq();

 private:
	class Thread : public lima::Thread
	{
		DEB_CLASS_NAMESPC(DebModCamera, "Jungfrau::Thread", "SlsDetector");
	public:
		enum State {
			Init, Ready, Running, Stopping, Quitting, End,
		};

		Thread(Jungfrau *jungfrau, int idx);
		virtual ~Thread();

		void setCPUAffinity(CPUAffinity aff);

		void prepareAcq();

		void startAcq()
		{ setState(Running); }
		void stopAcq()
		{
			setState(Stopping);
			AutoMutex l = lock();
			while (m_state != Ready)
				wait();
		}

	protected:
		virtual void threadFunction();

	private:
		friend class Jungfrau;

		AutoMutex lock()
		{ return m_jungfrau->lock(); }
		void wait()
		{ m_jungfrau->wait(); }
		void broadcast()
		{ m_jungfrau->broadcast(); }

		void setState(State state)
		{
			AutoMutex l = lock();
			m_state = state;
			broadcast();
		}

		Jungfrau *m_jungfrau;
		int m_idx;
		State m_state;
	};
	typedef std::vector<AutoPtr<Thread> > ThreadList;

	AutoMutex lock()
	{ return AutoMutex(m_cond.mutex()); }
	void wait()
	{ m_cond.wait(); }
	void broadcast()
	{ m_cond.broadcast(); }

	bool allFramesAcquired()
	{ return m_next_frame == m_nb_frames; }

	int getNbProcessingThreads();
	void setNbProcessingThreads(int nb_proc_threads);

	void processOneFrame(AutoMutex& l);

	static const int ChipSize;
	static const int ChipGap;
	static const int HalfModuleChips;

	Cond m_cond;
	Receiver *m_recv;
	FrameType m_nb_frames;
	FrameType m_next_frame;
	FrameType m_last_frame;
	SortedIntList m_in_process;
	FrameMap::Item *m_frame_map_item;
	ThreadList m_thread_list;
};

} // namespace SlsDetector

} // namespace lima



#endif // __SLS_DETECTOR_JUNGFRAU_H
