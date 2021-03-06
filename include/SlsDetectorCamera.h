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

#ifndef __SLS_DETECTOR_CAMERA_H
#define __SLS_DETECTOR_CAMERA_H

#include "SlsDetectorArgs.h"
#include "SlsDetectorReceiver.h"
#include "SlsDetectorCPUAffinity.h"
#include "SlsDetectorBebTools.h"

#include "slsDetectorUsers.h"

#include "lima/HwBufferMgr.h"
#include "lima/HwMaxImageSizeCallback.h"
#include "lima/Event.h"

#include <queue>

namespace lima 
{

namespace SlsDetector
{

class Eiger;

class Camera : public HwMaxImageSizeCallbackGen, public EventCallbackGen
{
	DEB_CLASS_NAMESPC(DebModCamera, "Camera", "SlsDetector");

public:
	typedef Defs::TrigMode TrigMode;
	typedef Defs::Settings Settings;
	typedef Defs::DACIndex DACIndex;
	typedef Defs::ADCIndex ADCIndex;
	typedef Defs::DetStatus DetStatus;
	typedef Defs::NetworkParameter NetworkParameter;

	Camera(std::string config_fname, int det_id = 0);
	Camera(const Camera& o) = delete;
	virtual ~Camera();

	Type getType();

	Model *getModel()
	{ return m_model; }

	NameList getHostnameList()
	{ return m_input_data->host_name_list; }

	int getNbDetModules()
	{ return m_input_data->host_name_list.size(); }

	int getNbDetSubModules()
	{ return m_det->getNMods(); }

	int getNbRecvs()
	{ return m_recv_list.size(); }

	Receiver* getRecv(int i)
	{ return m_recv_list[i]; }

	void setBufferCtrlObj(NumaSoftBufferCtrlObj *buffer_ctrl_obj)
	{ m_buffer_ctrl_obj = buffer_ctrl_obj; }

	void clearAllBuffers();

	void setPixelDepth(PixelDepth  pixel_depth);
	void getPixelDepth(PixelDepth& pixel_depth);

	void setRawMode(bool  raw_mode);
	void getRawMode(bool& raw_mode);

	State getState();
	void waitState(State state);
	State waitNotState(State state);

	ImageType getImageType() const
	{ return m_image_type; }

	void getFrameDim(FrameDim& frame_dim, bool raw = false)
	{ m_model->getFrameDim(frame_dim, raw); }

	FrameMap *getFrameMap()
	{ return &m_frame_map; }

	void putCmd(const std::string& s, int idx = -1);
	std::string getCmd(const std::string& s, int idx = -1);

	int getFramesCaught();
	DetStatus getDetStatus();

	void setTrigMode(TrigMode  trig_mode);
	void getTrigMode(TrigMode& trig_mode);
	void setNbFrames(FrameType  nb_frames);
	void getNbFrames(FrameType& nb_frames);
	void setExpTime(double  exp_time);
	void getExpTime(double& exp_time);
	void setLatTime(double  lat_time);
	void getLatTime(double& lat_time);
	void setFramePeriod(double  frame_period);
	void getFramePeriod(double& frame_period);

	void setSkipFrameFreq(FrameType  skip_frame_freq);
	void getSkipFrameFreq(FrameType& skip_frame_freq);

	// setDAC: sub_mod_idx: 0-N=sub_module, -1=all
	void setDAC(int sub_mod_idx, DACIndex dac_idx, int  val, 
		    bool milli_volt = false);
	void getDAC(int sub_mod_idx, DACIndex dac_idx, int& val, 
		    bool milli_volt = false);
	void getDACList(DACIndex dac_idx, IntList& val_list,
			bool milli_volt = false);

	void getADC(int sub_mod_idx, ADCIndex adc_idx, int& val);
	void getADCList(ADCIndex adc_idx, IntList& val_list);

	void setSettings(Settings  settings);
	void getSettings(Settings& settings);

	void setNetworkParameter(NetworkParameter net_param, std::string& val);
	void getNetworkParameter(NetworkParameter net_param, std::string& val);

	void setTolerateLostPackets(bool  tol_lost_packets);
	void getTolerateLostPackets(bool& tol_lost_packets);

	int getNbBadFrames(int item_idx);
	void getBadFrameList(int item_idx, int first_idx, int last_idx,
			     IntList& bad_frame_list);
	void getBadFrameList(int item_idx, IntList& bad_frame_list);

	void prepareAcq();
	void startAcq();
	void stopAcq();

	void triggerFrame();

	void registerTimeRangesChangedCallback(TimeRangesChangedCallback& cb);
	void unregisterTimeRangesChangedCallback(TimeRangesChangedCallback& cb);

	void getStats(Stats& stats, int recv_idx=-1);

	void setPixelDepthCPUAffinityMap(PixelDepthCPUAffinityMap aff_map);
	void getPixelDepthCPUAffinityMap(PixelDepthCPUAffinityMap& aff_map);

	GlobalCPUAffinityMgr::ProcessingFinishedEvent *
		getProcessingFinishedEvent();

	void reportException(Exception& e, std::string name);

private:
	typedef std::map<int, int> RecvPortMap;
	typedef std::queue<int> FrameQueue;
	typedef std::vector<AutoPtr<Receiver> > RecvList;

	struct Beb {
		BebShell shell;
		BebFpgaMem fpga_mem;
		Beb(const std::string& host_name);
	};
	typedef std::vector<AutoPtr<Beb> > BebList;

	struct AppInputData
	{
		DEB_CLASS_NAMESPC(DebModCamera, "Camera::AppInputData", 
				  "SlsDetector");
	public:
		std::string config_file_name;
		NameList host_name_list;
		RecvPortMap recv_port_map;
		AppInputData(std::string cfg_fname);
		void parseConfigFile();
	};

	class AcqThread : public Thread
	{
		DEB_CLASS_NAMESPC(DebModCamera, "Camera::AcqThread", 
				  "SlsDetector");
	public:
		AcqThread(Camera *cam);

		void queueFinishedFrame(FrameType frame);
		virtual void start();
		void stop(bool wait);

	protected:
		virtual void threadFunction();

	private:
		typedef std::pair<bool, bool> Status;

		class ExceptionCleanUp : Thread::ExceptionCleanUp
		{
			DEB_CLASS_NAMESPC(DebModCamera, 
					  "Camera::AcqThread::ExceptionCleanUp",
					  "SlsDetector");
		public:
			ExceptionCleanUp(AcqThread& thread);
			virtual ~ExceptionCleanUp();
		};

		Status newFrameReady(FrameType frame);
		void startAcq();
		void stopAcq();
		void cleanUp();

		Camera *m_cam;
		Cond& m_cond;
		State& m_state;
		FrameQueue m_frame_queue;
	};

	friend class Model;
	friend class Receiver;
	friend class GlobalCPUAffinityMgr;

	friend class Eiger;

	void setModel(Model *model);

	AutoMutex lock()
	{ return AutoMutex(m_cond.mutex()); }

	void updateImageSize();
	void updateTimeRanges();
	void updateCPUAffinity(bool recv_restarted);
	void setRecvCPUAffinity(const RecvCPUAffinityList& recv_affinity_list);

	static int64_t NSec(double x)
	{ return int64_t(x * 1e9); }

	State getEffectiveState();

	StdBufferCbMgr *getBufferCbMgr()
	{ return &m_buffer_ctrl_obj->getBuffer(); }

	char *getFrameBufferPtr(FrameType frame_nb);
	void removeSharedMem();
	void createReceivers();

	bool checkLostPackets();
	FrameType getLastReceivedFrame();

	void waitLastSkippedFrame();
	void processLastSkippedFrame(int recv_idx);

	void getSortedBadFrameList(IntList first_idx, IntList last_idx,
				   IntList& bad_frame_list );
	void getSortedBadFrameList(IntList& bad_frame_list)
	{ getSortedBadFrameList(IntList(), IntList(), bad_frame_list); }

	template <class T>
	void putNbCmd(const std::string& cmd, T val, int idx = -1)
	{
		std::ostringstream os;
		os << cmd << " " << val;
		putCmd(os.str(), idx);
	}

	template <class T>
	T getNbCmd(const std::string& cmd, int idx = -1)
	{
		std::string ans = getCmd(cmd, idx);
		std::istringstream is(ans);
		T val;
		is >> val;
		return val;
	}

	void setReceiverFifoDepth(int fifo_depth);
	bool isTenGigabitEthernetEnabled();
	void setFlowControl10G(bool enabled);
	void resetFramesCaught();

	int m_det_id;
	Model *m_model;
	Cond m_cond;
	AutoPtr<AppInputData> m_input_data;
	AutoPtr<slsDetectorUsers> m_det;
	BebList m_beb_list;
	FrameMap m_frame_map;
	RecvList m_recv_list;
	TrigMode m_trig_mode;
	FrameType m_lima_nb_frames;
	FrameType m_det_nb_frames;
	FrameType m_skip_frame_freq;
	SortedIntList m_missing_last_skipped_frame;
	double m_last_skipped_frame_timeout;
	double m_exp_time;
	double m_lat_time;
	double m_frame_period;
	Settings m_settings;
	NumaSoftBufferCtrlObj *m_buffer_ctrl_obj;
	PixelDepth m_pixel_depth;
	ImageType m_image_type;
	bool m_raw_mode;
	State m_state;
	double m_new_frame_timeout;
	double m_abort_sleep_time;
	bool m_tol_lost_packets;
	FrameArray m_prev_ifa;
	TimeRangesChangedCallback *m_time_ranges_cb;
	PixelDepthCPUAffinityMap m_cpu_affinity_map;
	GlobalCPUAffinityMgr m_global_cpu_affinity_mgr;
	AutoPtr<AcqThread> m_acq_thread;
};

} // namespace SlsDetector

} // namespace lima


#endif // __SLS_DETECTOR_CAMERA_H
