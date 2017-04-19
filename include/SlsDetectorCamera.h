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

#include "multiSlsDetector.h"
#include "multiSlsDetectorCommand.h"
#include "slsReceiverUsers.h"
#include "receiver_defs.h"

#include "processlib/LinkTask.h"

#include "lima/SizeUtils.h"
#include "lima/RegExUtils.h"
#include "lima/ThreadUtils.h"
#include "lima/MemUtils.h"
#include "lima/HwBufferMgr.h"
#include "lima/HwMaxImageSizeCallback.h"

#include <iostream>
#include <string>
#include <stdexcept>
#include <cstdlib>
#include <set>


namespace lima 
{

namespace SlsDetector
{

class Camera : public HwMaxImageSizeCallbackGen
{
	DEB_CLASS_NAMESPC(DebModCamera, "Camera", "SlsDetector");

public:
	enum State {
		Idle, Init, Starting, Running, StopReq, Stopping, Stopped,
	};

	enum Type {
		UnknownDet, GenericDet, EigerDet, JungfrauDet,
	};

	enum TrigMode {
		Auto            = slsDetectorDefs::AUTO_TIMING, 
		TriggerExposure = slsDetectorDefs::TRIGGER_EXPOSURE, 
		TriggerReadout  = slsDetectorDefs::TRIGGER_READOUT,
		Gating          = slsDetectorDefs::GATE_FIX_NUMBER, 
		TriggeredGating = slsDetectorDefs::GATE_WITH_START_TRIGGER,
		BurstTrigger    = slsDetectorDefs::BURST_TRIGGER,
	};

	enum Settings {
		Standard      = slsDetectorDefs::STANDARD,
		Fast          = slsDetectorDefs::FAST,
		HighGain      = slsDetectorDefs::HIGHGAIN,
		DynamicGain   = slsDetectorDefs::DYNAMICGAIN,
		LowGain       = slsDetectorDefs::LOWGAIN,
		MediumGain    = slsDetectorDefs::MEDIUMGAIN,
		VeryHighGain  = slsDetectorDefs::VERYHIGHGAIN,
		LowNoise      = slsDetectorDefs::LOWNOISE,
		DynamicHG0    = slsDetectorDefs::DYNAMICHG0,
		FixGain1      = slsDetectorDefs::FIXGAIN1,
		FixGain2      = slsDetectorDefs::FIXGAIN2,
		ForceSwitchG1 = slsDetectorDefs::FORCESWITCHG1,
		ForceSwitchG2 = slsDetectorDefs::FORCESWITCHG2,
		VeryLowGain   = slsDetectorDefs::VERYLOWGAIN,
		Undefined     = slsDetectorDefs::UNDEFINED,
		Unitialized   = slsDetectorDefs::UNINITIALIZED,
	};

	typedef uint64_t FrameType;

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

	protected:
		void putCmd(const std::string& s, int idx = -1);
		std::string getCmd(const std::string& s, int idx = -1);

		virtual bool checkSettings(Settings settings) = 0;

		virtual int getRecvPorts() = 0;

		virtual void prepareAcq() = 0;
		virtual void processRecvFileStart(int recv_idx,
						  uint32_t dsize) = 0;
		// TODO: add file finished callback
		virtual void processRecvPort(int recv_idx, FrameType frame, 
					     int port, char *dptr, 
					     uint32_t dsize, 
					     Mutex& lock, char *bptr) = 0;

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
		typedef std::set<int> List;
		typedef std::map<FrameType, List> Map;

		class Callback
		{
			DEB_CLASS_NAMESPC(DebModCamera, 
					  "Camera::FrameMap::Callback", 
					  "SlsDetector");

		public:
			Callback();
			virtual ~Callback();
		protected:
			virtual void frameFinished(FrameType frame) = 0;
		private:
			friend class FrameMap;
			FrameMap *m_map;
		};

		FrameMap(bool debug = false);
		~FrameMap();
		
		void setCallback(Callback *cb);
		void setNbItems(int nb_items);
		void clear();

		void frameItemFinished(FrameType frame, int item);
		
		FrameType getLastSeqFinishedFrame() const
		{ return m_last_seq_finished_frame; }

		const List& getNonSeqFinishedFrames() const
		{ return m_non_seq_finished_frames; }

		const Map& getFramePendingItemsMap() const
		{ return m_map; }

	private:
		friend class Callback;

		int m_nb_items;
		Map m_map;
		List m_non_seq_finished_frames;
		FrameType m_last_seq_finished_frame;
		Callback *m_cb;
		bool m_debug;
	};

	Camera(std::string config_fname);
	virtual ~Camera();

	Type getType();

	Model *getModel()
	{ return m_model; }

	void setBufferCbMgr(StdBufferCbMgr *buffer_cb_mgr)
	{ m_buffer_cb_mgr = buffer_cb_mgr; }

	void setSaveRaw(bool  save_raw);
	void getSaveRaw(bool& save_raw);

	State getState();
	void waitState(State state);
	State waitNotState(State state);

	int getNbDetModules()
	{ return m_input_data->host_name_list.size(); }

	ImageType getImageType() const
	{ return m_image_type; }

	void getFrameDim(FrameDim& frame_dim, bool raw = false)
	{ m_model->getFrameDim(frame_dim, raw); }

	const FrameMap& getRecvMap()
	{ return m_recv_map; }

	void putCmd(const std::string& s, int idx = -1);
	std::string getCmd(const std::string& s, int idx = -1);

	int getFramesCaught();
	std::string getStatus();

	void setTrigMode(TrigMode  trig_mode);
	void getTrigMode(TrigMode& trig_mode);
	void setNbFrames(FrameType  nb_frames);
	void getNbFrames(FrameType& nb_frames);
	void setExpTime(double  exp_time);
	void getExpTime(double& exp_time);
	void setFramePeriod(double  frame_period);
	void getFramePeriod(double& frame_period);

	// dat_idx -> slsDetectorDefs::dacIndex
	void setDAC(int dac_idx, int  val, bool milli_volt = false);
	void getDAC(int dac_idx, int& val, bool milli_volt = false);
	void setHighVoltage(int  hvolt);
	void getHighVoltage(int& hvolt);
	void setSettings(Settings  settings);
	void getSettings(Settings& settings);
	void setThresholdEnergy(int  thres);
	void getThresholdEnergy(int& thres);

	void prepareAcq();
	void startAcq();
	void stopAcq();

private:
	typedef RegEx::SingleMatchType SingleMatch;
	typedef RegEx::FullMatchType FullMatch;
	typedef RegEx::MatchListType MatchList;
	typedef MatchList::const_iterator MatchListIt;

	typedef std::vector<std::string> StringList;
	typedef StringList HostnameList;
	typedef std::map<int, int> RecvPortMap;
	typedef std::map<int, int> FrameRecvMap;

	typedef std::queue<int> FrameQueue;

	struct AppInputData
	{
		DEB_CLASS_NAMESPC(DebModCamera, "Camera::AppInputData", 
				  "SlsDetector");
	public:
		std::string config_file_name;
		HostnameList host_name_list;
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

		void prepareAcq();

	private:
		class FrameFinishedCallback : public FrameMap::Callback
		{
			DEB_CLASS_NAMESPC(DebModCamera, 
					  "Camera::Receiver"
					  "::FrameFinishedCallback", 
					  "SlsDetector");
		public:
			FrameFinishedCallback(Receiver *r);
		protected:
			virtual void frameFinished(FrameType frame);
		private:
			Receiver *m_recv;
		};

		friend class Camera;
		friend class FrameFinishedCallback;

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
		FrameMap m_port_map;
		Args m_args;
		AutoPtr<slsReceiverUsers> m_recv;
		AutoPtr<FrameFinishedCallback> m_cb;
	}; 

	typedef std::vector<AutoPtr<Receiver> > RecvList;

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

	class FrameFinishedCallback : public FrameMap::Callback
	{
		DEB_CLASS_NAMESPC(DebModCamera, 
				  "Camera::FrameFinishedCallback", 
				  "SlsDetector");
	public:
		FrameFinishedCallback(Camera *cam);
	protected:
		virtual void frameFinished(FrameType frame);
	private:
		Camera *m_cam;
	};

	friend class Model;

	void setModel(Model *model);

	AutoMutex lock()
	{ return AutoMutex(m_cond.mutex()); }

	int64_t NSec(double x)
	{ return int64_t(x * 1e9); }

	State getEffectiveState();

	char *getFrameBufferPtr(FrameType frame_nb);
	void removeSharedMem();
	void createReceivers();

	void receiverFrameFinished(FrameType frame, Receiver *recv);
	void frameFinished(FrameType frame);

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
	AutoPtr<multiSlsDetectorCommand> m_cmd;
	Mutex m_cmd_mutex;
	TrigMode m_trig_mode;
	FrameType m_nb_frames;
	double m_exp_time;
	double m_frame_period;
	Settings m_settings;
	FrameMap m_recv_map;
	AutoPtr<FrameFinishedCallback> m_frame_cb;
	StdBufferCbMgr *m_buffer_cb_mgr;
	ImageType m_image_type;
	bool m_save_raw;
	AutoPtr<AcqThread> m_acq_thread;
	volatile State m_state;
	FrameQueue m_frame_queue;
};

std::ostream& operator <<(std::ostream& os, Camera::State state);
std::ostream& operator <<(std::ostream& os, Camera::Type type);
std::ostream& operator <<(std::ostream& os, Camera::TrigMode trig_mode);
std::ostream& operator <<(std::ostream& os, Camera::Settings settings);

std::ostream& operator <<(std::ostream& os, const Camera::FrameMap& m);
std::ostream& operator <<(std::ostream& os, const Camera::FrameMap::List& l);
std::ostream& operator <<(std::ostream& os, const Camera::FrameMap::Map& m);


#define EIGER_PACKET_DATA_LEN	(4 * 1024)

class Eiger : public Camera::Model
{
	DEB_CLASS_NAMESPC(DebModCamera, "Eiger", "SlsDetector");

 public:
	typedef unsigned char Byte;
	typedef unsigned short Word;
	typedef unsigned int Long;

	typedef Camera::FrameType FrameType;

	enum CorrType {
		ChipBorder, Gap,
	};

	class CorrBase
	{
		DEB_CLASS_NAMESPC(DebModCamera, "Eiger::CorrBase", 
				  "SlsDetector");
	public:
		CorrBase(Eiger *eiger, CorrType type);
		virtual ~CorrBase();

		bool getRaw();
		int getNbModules()
		{ return m_nb_modules; }

		virtual void prepareAcq();
		virtual void correctFrame(FrameType frame, void *ptr) = 0;

	protected:
		friend class Eiger;
		Eiger *m_eiger;
		CorrType m_type;
		int m_nb_modules;
		FrameDim m_mod_frame_dim;
		Size m_frame_size;
		std::vector<int> m_inter_lines;
	};

	typedef std::map<CorrType, CorrBase *> CorrMap;

	class InterModGapCorr : public CorrBase
	{
		DEB_CLASS_NAMESPC(DebModCamera, "Eiger::InterModGapCorr", 
				  "SlsDetector");
	public:
		InterModGapCorr(Eiger *eiger);

		virtual void prepareAcq();
		virtual void correctFrame(FrameType frame, void *ptr);

	protected:
		typedef std::pair<int, int> Block;
		typedef std::vector<Block> BlockList;
		BlockList m_gap_list;
	};

	class Correction : public LinkTask
	{
		DEB_CLASS_NAMESPC(DebModCamera, "Eiger::Correction", 
				  "SlsDetector");
	public:
		Correction(Eiger *eiger);

		virtual Data process(Data& data);
	private:
		typedef std::vector<AutoPtr<CorrBase> > CorrList;
		CorrList m_corr_list;
	};

	Eiger(Camera *cam);
	~Eiger();
	
	virtual void getFrameDim(FrameDim& frame_dim, bool raw = false);

	virtual std::string getName();
	virtual void getPixelSize(double& x_size, double& y_size);

	// the returned object must be deleted by the caller
	Correction *createCorrectionTask();

 protected:
	virtual bool checkSettings(Camera::Settings settings);

	virtual int getRecvPorts();

	virtual void prepareAcq();
	virtual void processRecvFileStart(int recv_idx, uint32_t dsize);
	virtual void processRecvPort(int recv_idx, FrameType frame, int port,
				     char *dptr, uint32_t dsize, Mutex& lock, 
				     char *bptr);

 private:
	friend class Correction;
	friend class CorrBase;

	class RecvPortGeometry
	{
		DEB_CLASS_NAMESPC(DebModCamera, "Eiger::RecvPortGeometry", 
				  "SlsDetector");
	public:
		RecvPortGeometry(Eiger *eiger, int recv_idx, int port);

		void prepareAcq();
		void processRecvFileStart(uint32_t dsize);
		void processRecvPort(FrameType frame, char *dptr, char *bptr);
	private:
		Eiger *m_eiger;
		int m_port;
		bool m_top_half_recv;
		bool m_port_idx;
		bool m_raw;
		int m_recv_idx;
		int m_port_offset;
		int m_dlw;			// dest line width
		int m_plw;			// packet line width
		int m_clw;			// chip line width
		int m_pchips;
	};

	typedef std::vector<AutoPtr<RecvPortGeometry> > PortGeometryList;

	template <class T>
	class ChipBorderCorr : public CorrBase
	{
	public:
		ChipBorderCorr(Eiger *eiger)
			: CorrBase(eiger, ChipBorder)
		{}
		
		virtual void prepareAcq()
		{
			CorrBase::prepareAcq();

			m_f.resize(m_nb_modules);
			std::vector<BorderFactor>::iterator it = m_f.begin();
			for (int i = 0; i < m_nb_modules; ++i, ++it) {
				it->resize(2);
				(*it)[0] = m_eiger->getBorderCorrFactor(i, 0);
				(*it)[1] = m_eiger->getBorderCorrFactor(i, 1);
			}
		}

		virtual void correctFrame(FrameType frame, void *ptr)
		{
			correctBorderCols(ptr);
			correctBorderRows(ptr);
			correctInterChipCols(ptr);
			correctInterChipRows(ptr);
		}

	private:
		typedef std::vector<double> BorderFactor;

		static void correctInterChipLine(T *d, int offset, int nb_iter, 
						 int step) 
		{
			for (int i = 0; i < nb_iter; ++i, d += step)
				d[0] = d[offset] /= 2;
		}

		static void correctBorderLine(T *d, int nb_iter, int step,
					      double f) 
		{
			for (int i = 0; i < nb_iter; ++i, d += step)
				d[0] /= f;
		}

		void correctInterChipCols(void *ptr)
		{
			int width = m_frame_size.getWidth();
			int height= m_frame_size.getHeight();
			T *d = static_cast<T *>(ptr);
			for (int i = 0; i < HalfModuleChips - 1; ++i) {
				d += ChipSize;
				correctInterChipLine(d++, -1, height, width);
				correctInterChipLine(d++, 1, height, width);
			}
		}

		void correctInterChipRows(void *ptr)
		{
			int width = m_frame_size.getWidth();
			int mod_height = m_mod_frame_dim.getSize().getHeight();
			T *p = static_cast<T *>(ptr);
			for (int i = 0; i < m_nb_modules; ++i) {
				T *d = p + ChipSize * width;
				correctInterChipLine(d, -width, width, 1);
				d += width;
				correctInterChipLine(d, width, width, 1);
				if (i == m_nb_modules - 1)
					continue;
				p += (mod_height + m_inter_lines[i]) * width;
			}
		}

		void correctBorderCols(void *ptr)
		{
			int width = m_frame_size.getWidth();
			int height= m_frame_size.getHeight();
			T *d = static_cast<T *>(ptr);
			correctBorderLine(d, height, width, 2);
			d += width - 1;
			correctBorderLine(d, height, width, 2);
		}

		void correctBorderRows(void *ptr)
		{
			int width = m_frame_size.getWidth();
			int mod_height = m_mod_frame_dim.getSize().getHeight();
			T *p = static_cast<T *>(ptr);
			for (int i = 0; i < m_nb_modules; ++i) {
				double f0 = m_f[i][0], f1 = m_f[i][1];
				T *d = p;
				correctBorderLine(d, width, 1, f0);
				d += width;
				correctBorderLine(d, width, 1, f1);
				d += (mod_height - 1 - 2) * width;
				correctBorderLine(d, width, 1, f1);
				d += width;
				correctBorderLine(d, width, 1, f0);
				p += (mod_height + m_inter_lines[i]) * width;
			}
		}

		std::vector<BorderFactor> m_f;
	};

	void getRecvFrameDim(FrameDim& frame_dim, bool raw, bool geom);

	int getPortIndex(int recv_idx, int port)
	{ return recv_idx * RecvPorts + port; }

	CorrBase *createChipBorderCorr(ImageType image_type);
	CorrBase *createInterModGapCorr();

	void addCorr(CorrBase *corr);
	void removeCorr(CorrBase *corr);

	double getBorderCorrFactor(int det, int line);
	int getInterModuleGap(int det);

	static const int ChipSize;
	static const int ChipGap;
	static const int HalfModuleChips;
	static const int RecvPorts;

	int m_nb_det_modules;
	FrameDim m_recv_frame_dim;
	CorrMap m_corr_map;
	PortGeometryList m_port_geom_list;
};


} // namespace SlsDetector

} // namespace lima



#endif // __SLS_DETECTOR_CAMERA_H
