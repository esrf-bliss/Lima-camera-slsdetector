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
#include "SlsDetectorModel.h"
#include "SlsDetectorCPUAffinity.h"

#include "multiSlsDetector.h"
#include "slsReceiverUsers.h"

#include "lima/HwBufferMgr.h"
#include "lima/HwMaxImageSizeCallback.h"
#include "lima/Event.h"

#include <queue>

namespace lima 
{

namespace SlsDetector
{

class Camera : public HwMaxImageSizeCallbackGen, public EventCallbackGen
{
	DEB_CLASS_NAMESPC(DebModCamera, "Camera", "SlsDetector");

public:
	typedef Defs::TrigMode TrigMode;
	typedef Defs::Settings Settings;
	typedef Defs::DACIndex DACIndex;
	typedef Defs::ADCIndex ADCIndex;
	typedef Defs::ClockDiv ClockDiv;
	typedef Defs::ReadoutFlags ReadoutFlags;
	typedef Defs::DetStatus DetStatus;
	typedef Defs::NetworkParameter NetworkParameter;

	Camera(std::string config_fname);
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

	int getTotNbPorts()
	{ return m_recv_list.size() * m_recv_ports; }

	int getPortIndex(int recv_idx, int port)
	{ return recv_idx * m_recv_ports + port; }

	void setBufferCbMgr(StdBufferCbMgr *buffer_cb_mgr)
	{ m_buffer_cb_mgr = buffer_cb_mgr; }

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

	const FrameMap& getFrameMap()
	{ return m_frame_map; }

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

	// setDAC: sub_mod_idx: 0-N=sub_module, -1=all
	void setDAC(int sub_mod_idx, DACIndex dac_idx, int  val, 
		    bool milli_volt = false);
	void getDAC(int sub_mod_idx, DACIndex dac_idx, int& val, 
		    bool milli_volt = false);
	void getDACList(DACIndex dac_idx, IntList& val_list,
			bool milli_volt = false);

	void getADC(int sub_mod_idx, ADCIndex adc_idx, int& val);
	void getADCList(ADCIndex adc_idx, IntList& val_list);

	void setAllTrimBits(int sub_mod_idx, int  val);
	void getAllTrimBits(int sub_mod_idx, int& val);
	void getAllTrimBitsList(IntList& val_list);

	void setSettings(Settings  settings);
	void getSettings(Settings& settings);
	void setThresholdEnergy(int  thres);
	void getThresholdEnergy(int& thres);

	void setClockDiv(ClockDiv  clock_div);
	void getClockDiv(ClockDiv& clock_div);
	void setReadoutFlags(ReadoutFlags  flags);
	void getReadoutFlags(ReadoutFlags& flags);
	void getValidReadoutFlags(IntList& flag_list, NameList& flag_name_list);

	void setNetworkParameter(NetworkParameter net_param, std::string& val);
	void getNetworkParameter(NetworkParameter net_param, std::string& val);

	void setTolerateLostPackets(bool  tol_lost_packets);
	void getTolerateLostPackets(bool& tol_lost_packets);

	int getNbBadFrames(int port_idx);
	void getBadFrameList(int port_idx, int first_idx, int last_idx,
			     IntList& bad_frame_list);
	void getBadFrameList(int port_idx, IntList& bad_frame_list);

	void prepareAcq();
	void startAcq();
	void stopAcq();

	void registerTimeRangesChangedCallback(TimeRangesChangedCallback& cb);
	void unregisterTimeRangesChangedCallback(TimeRangesChangedCallback& cb);

	void getStats(Stats& stats, int port_idx=-1);

	void setPixelDepthCPUAffinityMap(PixelDepthCPUAffinityMap aff_map);
	void getPixelDepthCPUAffinityMap(PixelDepthCPUAffinityMap& aff_map);

	GlobalCPUAffinityMgr::ProcessingFinishedEvent *
		getProcessingFinishedEvent();

private:
	typedef std::map<int, int> RecvPortMap;

	typedef std::queue<int> FrameQueue;

	typedef FrameMap::FinishInfo FinishInfo;
	typedef FrameMap::FinishInfoList FinishInfoList;

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

	class Receiver 
	{
		DEB_CLASS_NAMESPC(DebModCamera, "Camera::Receiver", 
				  "SlsDetector");

	public:
		Receiver(Camera *cam, int idx, int rx_port);
		~Receiver();
		void start();

	private:
		friend class Camera;

		static int fileStartCallback(char *fpath, char *fname, 
					 FrameType fidx, uint32_t dsize, 
					 void *priv);

		static void portCallback(FrameType frame, 
					 uint32_t exp_len,
					 uint32_t recv_packets,
					 uint64_t bunch_id,
					 uint64_t timestamp,
					 uint16_t mod_id,
					 uint16_t x, uint16_t y, uint16_t z,
					 uint32_t debug,
					 uint16_t rr_nb,
					 uint8_t det_type,
					 uint8_t cb_version,
					 char *dptr, 
					 uint32_t dsize, 
					 void *priv);
		int fileStartCallback(char *fpath, char *fname, uint64_t fidx, 
				      uint32_t dsize);
		void portCallback(FrameType frame, int port, char *dptr, 
				  uint32_t dsize);

		Camera *m_cam;
		int m_idx;
		int m_rx_port;
		Args m_args;
		AutoPtr<slsReceiverUsers> m_recv;
	}; 

	typedef std::vector<AutoPtr<Receiver> > RecvList;

	class BufferThread : public Thread
	{
		DEB_CLASS_NAMESPC(DebModCamera, "Camera::BufferThread", 
				  "SlsDetector");
	public:
		typedef std::vector<FinishInfo> FinishInfoArray;

		BufferThread();
		~BufferThread();

		void init(Camera *cam, int port_idx);

		void prepareAcq();

		pid_t getTID()
		{ return m_tid; }

		bool isBadFrame(FrameType frame);

		int getNbBadFrames()
		{
			AutoMutex l = lock();
			return m_bad_frame_list.size();
		}

		void getBadFrameList(int first_idx, int last_idx, IntList& bfl)
		{
			AutoMutex l = lock();
			IntList::const_iterator b = m_bad_frame_list.begin();
			bfl.assign(b + first_idx, b + last_idx);
		}

	protected:
		virtual void start();
		virtual void threadFunction();

	private:
		friend class Camera;

		AutoMutex lock()
		{ return m_cond.mutex(); }

		void processFinishInfo(const FinishInfo& finfo);

		Camera *m_cam;
		FrameMap *m_frame_map;
		int m_port_idx;
		pid_t m_tid;
		bool m_end;
		Cond m_cond;
		IntList m_bad_frame_list;
	};

	class AcqThread : public Thread
	{
		DEB_CLASS_NAMESPC(DebModCamera, "Camera::AcqThread", 
				  "SlsDetector");
	public:
		AcqThread(Camera *cam);
		void stop(bool wait);
	protected:
		virtual void threadFunction();
	private:
		class ExceptionCleanUp : Thread::ExceptionCleanUp
		{
			DEB_CLASS_NAMESPC(DebModCamera, 
					  "Camera::AcqThread::ExceptionCleanUp",
					  "SlsDetector");
		public:
			ExceptionCleanUp(AcqThread& thread);
			virtual ~ExceptionCleanUp();
		};

		bool newFrameReady(FrameType frame);
		void startAcq();
		void stopAcq();
		void cleanUp();

		Camera *m_cam;
		Cond& m_cond;
		State& m_state;
		FrameQueue& m_frame_queue;
	};

	struct PortStats {
		Stats stats;
		Timestamp last_t0;
		Timestamp last_t1;
		void reset()
		{
			stats.reset();
			last_t0 = last_t1 = Timestamp();
		}
	};

	friend class Model;
	friend class GlobalCPUAffinityMgr;

	void setModel(Model *model);

	AutoMutex lock()
	{ return AutoMutex(m_cond.mutex()); }

	void updateImageSize();
	void updateTimeRanges();
	void updateCPUAffinity(bool recv_restarted);
	void setRecvCPUAffinity(const RecvCPUAffinity& recv_affinity);

	static int64_t NSec(double x)
	{ return int64_t(x * 1e9); }

	State getEffectiveState();

	char *getFrameBufferPtr(FrameType frame_nb);
	void removeSharedMem();
	void createReceivers();

	void processRecvFileStart(int recv_idx, uint32_t dsize);
	void processRecvPort(int port_idx, FrameType frame, char *dptr, 
			     uint32_t dsize);
	void frameFinished(FrameType frame);

	bool checkLostPackets();
	FrameType getLastReceivedFrame();

	void getSortedBadFrameList(IntList first_idx, IntList last_idx,
				   IntList& bad_frame_list );
	void getSortedBadFrameList(IntList& bad_frame_list)
	{ getSortedBadFrameList(IntList(), IntList(), bad_frame_list); }

	void addValidReadoutFlags(DebObj *deb_ptr, ReadoutFlags flags, 
				  IntList& flag_list, NameList& flag_name_list);

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

	Model *m_model;
	Cond m_cond;
	AutoPtr<AppInputData> m_input_data;
	RecvList m_recv_list;
	AutoPtr<multiSlsDetector> m_det;
	int m_recv_fifo_depth;
	TrigMode m_trig_mode;
	FrameType m_nb_frames;
	double m_exp_time;
	double m_lat_time;
	double m_frame_period;
	Settings m_settings;
	FrameMap m_frame_map;
	int m_recv_ports;
	StdBufferCbMgr *m_buffer_cb_mgr;
	PixelDepth m_pixel_depth;
	ImageType m_image_type;
	bool m_raw_mode;
	AutoPtr<BufferThread, true> m_buffer_thread;
	AutoPtr<AcqThread> m_acq_thread;
	State m_state;
	FrameQueue m_frame_queue;
	double m_new_frame_timeout;
	double m_abort_sleep_time;
	bool m_tol_lost_packets;
	std::vector<PortStats> m_port_stats;
	TimeRangesChangedCallback *m_time_ranges_cb;
	PixelDepthCPUAffinityMap m_cpu_affinity_map;
	GlobalCPUAffinityMgr m_global_cpu_affinity_mgr;
};

} // namespace SlsDetector

} // namespace lima


#endif // __SLS_DETECTOR_CAMERA_H
