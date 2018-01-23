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

	void setTolerateLostPackets(bool  tol_lost_packets);
	void getTolerateLostPackets(bool& tol_lost_packets);
	void getBadFrameList(IntList& bad_frame_list);

	bool isBadFrame(int port_idx, FrameType frame)
	{ return m_buffer_thread[port_idx].isBadFrame(frame); }

	void prepareAcq();
	void startAcq();
	void stopAcq();

	void registerTimeRangesChangedCallback(TimeRangesChangedCallback& cb);
	void unregisterTimeRangesChangedCallback(TimeRangesChangedCallback& cb);

	void getStats(Stats& stats);

	void setPixelDepthCPUAffinityMap(PixelDepthCPUAffinityMap aff_map);
	void getPixelDepthCPUAffinityMap(PixelDepthCPUAffinityMap& aff_map);

	SystemCPUAffinityMgr::ProcessingFinishedEvent *
		getProcessingFinishedEvent();

private:
	typedef std::map<int, int> RecvPortMap;

	typedef std::queue<int> FrameQueue;

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
		typedef FrameMap::FinishInfo FinishInfo;
		typedef std::vector<FinishInfo> FinishInfoArray;

		BufferThread();
		~BufferThread();

		void init(Camera *cam, int port_idx, int size);

		void prepareAcq();

		void getNewFrameEntry(int& idx, FinishInfo*& finfo)
		{
			DEB_MEMBER_FUNCT();
			AutoMutex l = lock();
			if (m_free_idx == m_finish_idx)
				THROW_HW_ERROR(Error) << "BufferThread overrun";
			idx = m_free_idx;
			m_free_idx = getIndex(m_free_idx + 1);
			l.unlock();
			finfo = &m_finfo_array[idx];
		}

		void putNewFrameEntry(int idx, FinishInfo* /*finfo*/)
		{
			DEB_MEMBER_FUNCT();
			AutoMutex l = lock();
			m_ready_idx = idx;
			m_cond.broadcast();
		}

		pid_t getTID()
		{ return m_tid; }

		bool isBadFrame(FrameType frame);

	protected:
		virtual void start();
		virtual void threadFunction();

	private:
		friend class Camera;

		AutoMutex lock()
		{ return m_cond.mutex(); }

		int getIndex(int i)
		{
			while (i < 0)
				i += m_size;
			return i % m_size;
		}

		void processFinishInfo(FinishInfo& finfo);

		Camera *m_cam;
		int m_port_idx;
		pid_t m_tid;
		bool m_end;
		Cond m_cond;
		int m_size;
		FinishInfoArray m_finfo_array;
		int m_free_idx;
		int m_ready_idx;
		int m_finish_idx;
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
		bool newFrameReady(FrameType frame);

		Camera *m_cam;
		Cond& m_cond;
		volatile State& m_state;
		FrameQueue& m_frame_queue;
	};

	friend class Model;
	friend class SystemCPUAffinityMgr;

	void setModel(Model *model);

	AutoMutex lock()
	{ return AutoMutex(m_cond.mutex()); }

	void updateImageSize();
	void updateTimeRanges();
	void updateCPUAffinity(bool recv_restarted);
	void setRecvCPUAffinity(CPUAffinity recv_affinity);

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

	IntList getSortedBadFrameList(IntList first_idx, IntList last_idx);
	IntList getSortedBadFrameList()
	{ return getSortedBadFrameList(IntList(), IntList()); }

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
	volatile State m_state;
	FrameQueue m_frame_queue;
	double m_new_frame_timeout;
	double m_abort_sleep_time;
	bool m_tol_lost_packets;
	std::vector<Timestamp> m_stat_last_t0;
	std::vector<Timestamp> m_stat_last_t1;
	Stats m_stats;
	TimeRangesChangedCallback *m_time_ranges_cb;
	PixelDepthCPUAffinityMap m_cpu_affinity_map;
	SystemCPUAffinityMgr m_system_cpu_affinity_mgr;
};

} // namespace SlsDetector

} // namespace lima


#endif // __SLS_DETECTOR_CAMERA_H
