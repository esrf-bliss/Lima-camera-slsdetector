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
#include "SlsDetectorDefs.h"

#include "multiSlsDetector.h"
#include "slsReceiverUsers.h"

#include "lima/RegExUtils.h"
#include "lima/HwBufferMgr.h"
#include "lima/HwMaxImageSizeCallback.h"
#include "lima/Event.h"
#include "lima/SimplePipe.h"

#include <set>
#include <queue>

namespace lima 
{

namespace SlsDetector
{

template <class T>
class PrettyList
{
 public:
	typedef typename T::const_iterator const_iterator;

	PrettyList(const T& l) : begin(l.begin()), end(l.end()) {}
	PrettyList(const_iterator b, const_iterator e) : begin(b), end(e) {}

	ostream& print(ostream& os) const
	{
		os << "[";
		int prev;
		bool in_seq = false;
		bool first = true;
		for (const_iterator it = begin; it != end; ++it) {
			int val = *it;
			bool seq = (!first && (val == prev + 1));
			if (!seq) {
				if (in_seq)
					os << "-" << prev;
				os << (first ? "" : ",") << val;
			}
			prev = val;
			in_seq = seq;
			first = false;
		}
		if (in_seq)
			os << "-" << prev;
		return os << "]";
	}

 private:
	const_iterator begin, end;
};

template <class T>
std::ostream& operator <<(std::ostream& os, const PrettyList<T>& pl)
{
	return pl.print(os);
}


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

	enum State {
		Idle, Init, Starting, Running, StopReq, Stopping, Stopped,
	};

	enum Type {
		UnknownDet, GenericDet, EigerDet, JungfrauDet,
	};

	enum PixelDepth {
		PixelDepth4 = 4, 
		PixelDepth8 = 8, 
		PixelDepth16 = 16, 
		PixelDepth32 = 32,
	};

	typedef uint64_t FrameType;
	typedef std::vector<std::string> StringList;
	typedef StringList NameList;
	typedef std::vector<int> IntList;
	typedef std::vector<double> FloatList;
	typedef std::set<int> SortedIntList;
	typedef std::vector<FrameType> FrameArray;

	struct TimeRanges {
		TimeRanges() :
			min_exp_time(-1.), 
			max_exp_time(-1.),
			min_lat_time(-1.),
			max_lat_time(-1.),
			min_frame_period(-1.),
			max_frame_period(-1.)
		{}

		double min_exp_time;
		double max_exp_time;
		double min_lat_time;
		double max_lat_time;
		double min_frame_period;
		double max_frame_period;
	};

	class TimeRangesChangedCallback {
		DEB_CLASS_NAMESPC(DebModCamera, "TimeRangesChangedCallback", 
				  "SlsDetector::Camera");
	public:
		TimeRangesChangedCallback();
		virtual ~TimeRangesChangedCallback();

	protected:
		virtual void timeRangesChanged(TimeRanges time_ranges) = 0;

	private:
		friend class Camera;
		Camera *m_cam;
	};

	class Glob
	{
		DEB_CLASS_NAMESPC(DebModCamera, "Glob", "SlsDetector::Camera");
	public:
		Glob(std::string pattern = "");
		Glob& operator =(std::string pattern);

		static StringList find(std::string pattern);
		static StringList split(std::string path);

		int getNbEntries() const
		{ return m_found_list.size(); }

		StringList getPathList() const
		{ return m_found_list; }

		StringList getSubPathList(int idx) const;

	private:
		std::string m_pattern;
		StringList m_found_list;
	};

	class NumericGlob
	{
		DEB_CLASS_NAMESPC(DebModCamera, "NumericGlob", 
				  "SlsDetector::Camera");
	public:
		typedef std::pair<int, std::string> IntString;
		typedef std::vector<IntString> IntStringList;

		NumericGlob(std::string pattern_prefix, 
			    std::string pattern_suffix = "");

		int getNbEntries() const
		{ return m_glob.getNbEntries(); }

		IntStringList getIntPathList() const;

	private:
		int m_nb_idx;
		int m_prefix_len;
		int m_suffix_len;
		Glob m_glob;
	};

	class CPUAffinity 
	{
		DEB_CLASS_NAMESPC(DebModCamera, "CPUAffinity", 
				  "SlsDetector::Camera");
	public:
		CPUAffinity(uint64_t m = 0) : m_mask(m) 
		{}

		static bool UseSudo;
		static int getNbCPUs(bool max_nb = false);

		static uint64_t allCPUs(bool max_nb = false)
		{ return (uint64_t(1) << getNbCPUs(max_nb)) - 1; }

		void initCPUSet(cpu_set_t& cpu_set) const;
		void applyToTask(pid_t task, bool incl_threads = true,
				 bool use_taskset = true) const;

		operator uint64_t() const
		{ return m_mask ? m_mask : allCPUs(); }

		CPUAffinity& operator =(uint64_t m)
		{ m_mask = m; return *this; }

		bool isDefault() const
		{ return !m_mask || (m_mask == allCPUs()); }

	private:
		void applyWithTaskset(pid_t task, bool incl_threads) const;
		void applyWithSetAffinity(pid_t task, bool incl_threads) const;

		static int findNbCPUs();
		static int findMaxNbCPUs();
		uint64_t m_mask;
	};

	class ProcCPUAffinityMgr
	{
		DEB_CLASS_NAMESPC(DebModCamera, "ProcCPUAffinityMgr", 
				  "SlsDetector::Camera");
	public:
		typedef IntList ProcList;

		enum Filter {
			All, MatchAffinity, NoMatchAffinity,
		};

		ProcCPUAffinityMgr();
		~ProcCPUAffinityMgr();

		static ProcList getProcList(Filter filter = All, 
					    CPUAffinity cpu_affinity = 0);

		void setOtherCPUAffinity(CPUAffinity cpu_affinity);

	private:
		class WatchDog
		{
			DEB_CLASS_NAMESPC(DebModCamera, "WatchDog", 
					  "SlsDetector::"
					  "Camera::ProcCPUAffinityMgr");
		public:
			WatchDog();
			~WatchDog();

			bool childEnded();
			void setOtherCPUAffinity(CPUAffinity cpu_affinity);

		private:
			enum Cmd {
				Init, SetAffinity, CleanUp, Ok,
			};

			typedef uint64_t Arg;

			struct Packet {
				Cmd cmd;
				Arg arg;
			};
			
			static void sigTermHandler(int signo);

			void childFunction();
			void affinitySetter(CPUAffinity cpu_affinity);

			ProcList getOtherProcList(CPUAffinity cpu_affinity);

			void sendChildCmd(Cmd cmd, Arg arg = 0);
			Packet readParentCmd();
			void ackParentCmd();

			Pipe m_cmd_pipe;
			Pipe m_res_pipe;
			pid_t m_lima_pid;
			pid_t m_child_pid;
		};

		AutoPtr<WatchDog> m_watchdog;
	};

	struct SystemCPUAffinity {
	private:
		DEB_CLASS_NAMESPC(DebModCamera, "SystemCPUAffinity", 
				  "SlsDetector::Camera");
	public:
		CPUAffinity recv;
		CPUAffinity lima;
		CPUAffinity other;

		SystemCPUAffinity(Camera *cam = NULL) : m_cam(cam)
		{}
		
		void applyAndSet(const SystemCPUAffinity& o);

	private:
		Camera *m_cam;
		AutoPtr<ProcCPUAffinityMgr> m_proc_mgr;
	};

	typedef std::map<PixelDepth, SystemCPUAffinity> 
						PixelDepthCPUAffinityMap;

	static bool isValidFrame(FrameType frame)
	{ return (frame != FrameType(-1)); }

	static FrameType latestFrame(FrameType a, FrameType b)
	{
		if (!isValidFrame(a))
			return b;
		if (!isValidFrame(b))
			return a;
		return std::max(a, b);
	}

	static FrameType oldestFrame(FrameType a, FrameType b)
	{
		if (!isValidFrame(a))
			return a;
		if (!isValidFrame(b))
			return b;
		return std::min(a, b);
	}

	static FrameType updateLatestFrame(FrameType& a, FrameType b)
	{
		a = latestFrame(a, b);
		return a;
	}

	static FrameType updateOldestFrame(FrameType& a, FrameType b)
	{
		a = oldestFrame(a, b);
		return a;
	}

	static FrameType getLatestFrame(const FrameArray& l);
	static FrameType getOldestFrame(const FrameArray& l);

	class Model
	{
		DEB_CLASS_NAMESPC(DebModCamera, "Camera::Model", "SlsDetector");
	public:
		Model(Camera *cam, Type type);
		virtual ~Model();
	
		virtual void getFrameDim(FrameDim& frame_dim, 
					 bool raw = false) = 0;
		
		Camera *getCamera()
		{ return m_cam; }

		Type getType()
		{ return m_type; }

		virtual std::string getName() = 0;
		virtual void getPixelSize(double& x_size, double& y_size) = 0;

		virtual void getDACInfo(NameList& name_list, 
					IntList& idx_list,
					IntList& milli_volt_list) = 0;
		virtual void getADCInfo(NameList& name_list, 
					IntList& idx_list,
					FloatList& factor_list, 
					FloatList& min_val_list) = 0;

		virtual void getTimeRanges(TimeRanges& time_ranges) = 0;

	protected:
		void updateCameraModel();

		virtual void updateImageSize() = 0;

		void putCmd(const std::string& s, int idx = -1);
		std::string getCmd(const std::string& s, int idx = -1);

		virtual bool checkSettings(Settings settings) = 0;

		virtual ReadoutFlags getReadoutFlagsMask() = 0;
		virtual bool checkReadoutFlags(ReadoutFlags flags,
					       IntList& flag_list,
					       bool silent = false) = 0;

		virtual int getRecvPorts() = 0;

		virtual void prepareAcq() = 0;
		virtual void processRecvFileStart(int port_idx,
						  uint32_t dsize) = 0;
		// TODO: add file finished callback
		virtual void processRecvPort(int port_idx, FrameType frame, 
					     char *dptr, uint32_t dsize, 
					     char *bptr) = 0;

	private:
		friend class Camera;
		Camera *m_cam;
		Type m_type;
	};

	class FrameMap
	{
		DEB_CLASS_NAMESPC(DebModCamera, "Camera::FrameMap", 
				  "SlsDetector");

	public:
		struct FinishInfo {
			FrameType first_lost;
			int nb_lost;
			SortedIntList finished;
		};

		FrameMap();
		~FrameMap();
		
		void setNbItems(int nb_items);
		void setBufferSize(int buffer_size);
		void clear();

		void checkFinishedFrameItem(FrameType frame, int item);
		FinishInfo frameItemFinished(FrameType frame, int item, 
					     bool no_check, bool valid);

		FrameArray getItemFrameArray() const
		{ return m_last_item_frame; }

		FrameType getLastItemFrame() const
		{ return getLatestFrame(m_last_item_frame); }

		FrameType getLastFinishedFrame() const
		{ return getOldestFrame(m_last_item_frame); }

	private:
		struct AtomicCounter {
			int count;
			Mutex mutex;

			void set(int reset)
			{ count = reset; }

			bool dec_test_and_reset(int reset)
			{
				mutex.lock();
				bool zero = (--count == 0);
				if (zero)
					set(reset);
				mutex.unlock();
				return zero;
			}
		};
		typedef std::vector<AtomicCounter> CounterList;

		int m_nb_items;
		FrameArray m_last_item_frame;
		int m_buffer_size;
		CounterList m_frame_item_count;
	};

	class SeqFilter {
	public:
		struct Range {
			int first;
			int nb;

			Range()
			{
				reset();
			}

			void reset(int new_first = 0)
			{
				first = new_first;
				nb = 0;
			}

			int end()
			{
				return first + nb;
			}

			bool tryExtend(int val)
			{
				bool ok = (val == end());
				if (ok)
					nb++;
				return ok;
			}
		};

		void addVal(int new_val)
		{
			SortedIntList& w = m_waiting;
			if (!m_seq_range.tryExtend(new_val)) {
				w.insert(new_val);
			} else {
				while (!w.empty()) {
					SortedIntList::iterator wit = w.begin();
					if (!m_seq_range.tryExtend(*wit))
						break;
					w.erase(wit);
				}
			}
		}

		Range getSeqRange() 
		{
			Range ret = m_seq_range;
			m_seq_range.reset(m_seq_range.end());
			return ret;
		}

	private:
		Range m_seq_range;
		SortedIntList m_waiting;
	};

	struct SimpleStat {
		double xmin, xmax, xacc, xacc2;
		int xn;
		double factor;
		mutable Mutex lock;

		SimpleStat(double f = 1);
		void reset();
		void add(double x);
		SimpleStat& operator =(const SimpleStat& o);

		int n() const;
		double min() const;
		double max() const;
		double ave() const;
		double std() const;
	};

	struct Stats {
		SimpleStat cb_period;
		SimpleStat new_finish;
		SimpleStat cb_exec;
		SimpleStat recv_exec;
		Stats();
		void reset();
	};

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

	void prepareAcq();
	void startAcq();
	void stopAcq();

	void registerTimeRangesChangedCallback(TimeRangesChangedCallback& cb);
	void unregisterTimeRangesChangedCallback(TimeRangesChangedCallback& cb);

	void getStats(Stats& stats);

	void setPixelDepthCPUAffinityMap(PixelDepthCPUAffinityMap aff_map);
	void getPixelDepthCPUAffinityMap(PixelDepthCPUAffinityMap& aff_map);

private:
	typedef RegEx::SingleMatchType SingleMatch;
	typedef RegEx::FullMatchType FullMatch;
	typedef RegEx::MatchListType MatchList;
	typedef MatchList::const_iterator MatchListIt;

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

	protected:
		virtual void start();
		virtual void threadFunction();

	private:
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
	friend class SystemCPUAffinity;

	void setModel(Model *model);

	AutoMutex lock()
	{ return AutoMutex(m_cond.mutex()); }

	void updateImageSize();
	void updateTimeRanges();

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

	IntList getSortedBadFrameList(int first_idx, int last_idx);
	IntList getSortedBadFrameList()
	{ return getSortedBadFrameList(0, m_bad_frame_list.size()); }

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
	IntList m_bad_frame_list;
	std::vector<Timestamp> m_stat_last_t0;
	std::vector<Timestamp> m_stat_last_t1;
	Stats m_stats;
	TimeRangesChangedCallback *m_time_ranges_cb;
	PixelDepthCPUAffinityMap m_cpu_affinity_map;
	SystemCPUAffinity m_system_cpu_affinity;
};

std::ostream& operator <<(std::ostream& os, Camera::State state);
std::ostream& operator <<(std::ostream& os, Camera::Type type);

std::ostream& operator <<(std::ostream& os, const Camera::FrameMap& m);
std::ostream& operator <<(std::ostream& os, const Camera::SortedIntList& l);
std::ostream& operator <<(std::ostream& os, const Camera::FrameArray& a);
std::ostream& operator <<(std::ostream& os, const Camera::SimpleStat& s);
std::ostream& operator <<(std::ostream& os, const Camera::Stats& s);
std::ostream& operator <<(std::ostream& os, const Camera::CPUAffinity& a);
std::ostream& operator <<(std::ostream& os, const Camera::SystemCPUAffinity& a);
std::ostream& operator <<(std::ostream& os, 
			  const Camera::PixelDepthCPUAffinityMap& m);

inline int operator <(const Camera::NumericGlob::IntString& a,
		      const Camera::NumericGlob::IntString& b)
{
	return (a.first < b.first);
}

inline
bool operator ==(const Camera::CPUAffinity& a, const Camera::CPUAffinity& b)
{
	uint64_t mask = Camera::CPUAffinity::allCPUs();
	return (uint64_t(a) & mask) == (uint64_t(b) & mask);
}

inline
bool operator !=(const Camera::CPUAffinity& a, const Camera::CPUAffinity& b)
{
	return !(a == b);
}


typedef PrettyList<Camera::IntList> PrettyIntList;
typedef PrettyList<Camera::SortedIntList> PrettySortedList;

} // namespace SlsDetector

} // namespace lima



#endif // __SLS_DETECTOR_CAMERA_H
