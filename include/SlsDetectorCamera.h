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


#define DebTypeCameraStart	DebType(1 << 16)
#define DebTypeCameraMap	DebType(1 << 17)
#define DebTypeCameraFrame	DebType(1 << 18)
#define DebTypeRecvFrame	DebType(1 << 19)
#define DebTypeRecvPacket	DebType(1 << 20)

#define DEB_CAMERA_START()	DEB_MSG(DebTypeCameraStart)
#define DEB_CAMERA_MAP()	DEB_MSG(DebTypeCameraMap)
#define DEB_CAMERA_FRAME()	DEB_MSG(DebTypeCameraFrame)
#define DEB_RECV_FRAME()	DEB_MSG(DebTypeRecvFrame)
#define DEB_RECV_PACKET()	DEB_MSG(DebTypeRecvPacket)

namespace lima 
{

namespace SlsDetector
{

class Args
{
	DEB_CLASS_NAMESPC(DebModCamera, "Args", "SlsDetector");

public:
	Args();
	Args(unsigned int argc, char *argv[]);
	Args(const std::string& s);
	Args(const Args& o);

	void set(const std::string& s);
	void clear();

	unsigned int size()
	{ return m_argc; }
	operator char **()
	{ return m_argv; }
	operator bool()
	{ return bool(m_argc); }
	char *operator[](int pos)
	{ return m_argv[pos]; }

	Args& operator =(const std::string& s);

	string pop_front();
	void erase(int pos);

private:
	typedef std::vector<std::string> StringList;

	void update_argc_argv();
	
	StringList m_arg_list;
	unsigned int m_argc;
	AutoPtr<char *, true> m_argv;
};

enum State {
	Idle, Init, Starting, Running, StopReq, Stopping, Stopped,
};

std::ostream& operator <<(std::ostream& os, State state);


class Camera : public HwMaxImageSizeCallbackGen
{
	DEB_CLASS_NAMESPC(DebModCamera, "Camera", "SlsDetector");

public:
	enum Type {
		UnknownDet, GenericDet, EigerDet, JungfrauDet,
	};
	
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

		virtual int getPacketLen() = 0;
		virtual int getRecvFramePackets() = 0;
	
		virtual void prepareAcq() = 0;
		virtual int processRecvStart(int recv_idx, int dsize) = 0;
		virtual int processRecvPacket(int recv_idx, int frame, 
					      char *dptr, int dsize, 
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
		typedef std::map<int, List> Map;

		class Callback
		{
			DEB_CLASS_NAMESPC(DebModCamera, 
					  "Camera::FrameMap::Callback", 
					  "SlsDetector");

		public:
			Callback();
			virtual ~Callback();
		protected:
			virtual void frameFinished(int frame) = 0;
		private:
			friend class FrameMap;
			FrameMap *m_map;
		};

		FrameMap(bool debug = false);
		~FrameMap();
		
		void setCallback(Callback *cb);
		void setNbItems(int nb_items);
		void clear();

		void frameItemFinished(int frame, int item);
		
		int getLastSeqFinishedFrame() const
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
		int m_last_seq_finished_frame;
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

	void setNbFrames(int  nb_frames);
	void getNbFrames(int& nb_frames);
	void setExpTime(double  exp_time);
	void getExpTime(double& exp_time);
	void setFramePeriod(double  frame_period);
	void getFramePeriod(double& frame_period);

	void setHighVoltage(int  hvolt);
	void getHighVoltage(int& hvolt);
	void setEnergyThreshold(int  thres);
	void getEnergyThreshold(int& thres);

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
		Receiver(Camera *cam, int idx, int rx_port, int mode);
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
			virtual void frameFinished(int frame);
		private:
			Receiver *m_recv;
		};

		friend class Camera;
		friend class FrameFinishedCallback;

		static int startCallback(char *fpath, char *fname, int fidx, 
					 int dsize, void *priv);

		static void frameCallback(int frame, char *dptr, int dsize, 
					  FILE *f, char *guidptr, void *priv);

		int startCallback(char *fpath, char *fname, int fidx, 
				  int dsize);
		void frameCallback(int frame, char *dptr, int dsize, FILE *f, 
				   char *guidptr);

		Camera *m_cam;
		int m_idx;
		int m_rx_port;
		int m_mode;
		FrameMap m_packet_map;
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
		bool newFrameReady(int frame);

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
		virtual void frameFinished(int frame);
	private:
		Camera *m_cam;
	};

	friend class Model;

	void setModel(Model *model);

	AutoMutex lock()
	{ return AutoMutex(m_cond.mutex()); }

	State getEffectiveState();

	char *getFrameBufferPtr(int frame_nb);
	void removeSharedMem();
	void createReceivers();

	void receiverFrameFinished(int frame, Receiver *recv);
	void frameFinished(int frame);

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
	int m_nb_frames;
	double m_exp_time;
	double m_frame_period;
	FrameMap m_recv_map;
	AutoPtr<FrameFinishedCallback> m_frame_cb;
	StdBufferCbMgr *m_buffer_cb_mgr;
	ImageType m_image_type;
	bool m_save_raw;
	AutoPtr<AcqThread> m_acq_thread;
	volatile State m_state;
	FrameQueue m_frame_queue;
};

std::ostream& operator <<(std::ostream& os, Camera::Type type);

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

	struct Packet 
	{
		struct pre 
		{
			Long frame;	
			Byte code;	// 0x6b=first, 0x69=others
			Byte len;	// 0x80
			Byte flags;	// 0x00=1st-half, 0x20=2nd-half
			Byte idx;	// 32-bit
		} pre;
		char data[EIGER_PACKET_DATA_LEN];
		struct post 
		{
			Long frame;
			Byte res_1[2];	// 0x00
			Byte next;	// 32-bit
			Byte res_2;     // 0x00
		} post;
	};

	class ChipBorderCorrBase
	{
		DEB_CLASS_NAMESPC(DebModCamera, "Eiger::ChipBorderCorrBase", 
				  "SlsDetector");
	public:
		ChipBorderCorrBase(Eiger *eiger);
		virtual ~ChipBorderCorrBase();

		bool getRaw();

		virtual void prepareAcq();
		virtual void correctFrame(int frame, void *ptr) = 0;

	protected:
		friend class Eiger;

		Eiger *m_eiger;
		int m_nb_modules;
		Size m_mod_size;
		Size m_size;
		std::vector<int> m_inter_lines;
	};

	class Correction : public LinkTask
	{
		DEB_CLASS_NAMESPC(DebModCamera, "Eiger::Correction", 
				  "SlsDetector");
	public:
		Correction(Eiger *eiger);
		virtual ~Correction();

		virtual Data process(Data& data);
	private:
		AutoPtr<ChipBorderCorrBase> m_corr;
	};

	Eiger(Camera *cam);
	~Eiger();
	
	virtual void getFrameDim(FrameDim& frame_dim, bool raw = false);

	virtual std::string getName();
	virtual void getPixelSize(double& x_size, double& y_size);

	// the returned object must be deleted by the caller
	Correction *createCorrectionTask();

 protected:
	virtual int getPacketLen();
	virtual int getRecvFramePackets();

	virtual void prepareAcq();
	virtual int processRecvStart(int recv_idx, int dsize);
	virtual int processRecvPacket(int recv_idx, int frame, 
				      char *dptr, int dsize, Mutex& lock, 
				      char *bptr);

 private:
	friend class Correction;
	friend class ChipBorderCorrBase;

	class RecvPacketGeometry
	{
		DEB_CLASS_NAMESPC(DebModCamera, "Eiger::RecvPacketGeometry", 
				  "SlsDetector");
	public:
		RecvPacketGeometry(Eiger *eiger, int recv_idx);

		void prepareAcq();
		void processRecvStart(int dsize);
		int processRecvPacket(int frame, Packet *p, char *bptr);
	private:
		Eiger *m_eiger;
		bool m_top_half_recv;
		bool m_raw;
		int m_recv_idx;
		int m_recv_offset;
		int m_packet_offset;
		int m_right_offset;
		int m_dlw;			// dest line width
		int m_plw;			// packet line width
		int m_clw;			// chip line width
		int m_pchips;
		int m_plines;
		int m_gap_size;
	};

	typedef std::vector<AutoPtr<RecvPacketGeometry> > RecvGeometryList;

	template <class T>
	class ChipBorderCorr : public ChipBorderCorrBase
	{
	public:
		ChipBorderCorr(Eiger *eiger)
			: ChipBorderCorrBase(eiger)
		{}
		
		virtual void prepareAcq()
		{
			ChipBorderCorrBase::prepareAcq();

			m_f.resize(m_nb_modules);
			std::vector<BorderFactor>::iterator it = m_f.begin();
			for (int i = 0; i < m_nb_modules; ++i, ++it) {
				it->resize(2);
				(*it)[0] = m_eiger->getBorderCorrFactor(i, 0);
				(*it)[1] = m_eiger->getBorderCorrFactor(i, 1);
			}
		}

		virtual void correctFrame(int frame, void *ptr)
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
			int width = m_size.getWidth();
			int height= m_size.getHeight();
			T *d = static_cast<T *>(ptr);
			for (int i = 0; i < HalfModuleChips - 1; ++i) {
				d += ChipSize;
				correctInterChipLine(d++, -1, height, width);
				correctInterChipLine(d++, 1, height, width);
			}
		}

		void correctInterChipRows(void *ptr)
		{
			int width = m_size.getWidth();
			int mod_height = m_mod_size.getHeight();
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
			int width = m_size.getWidth();
			int height= m_size.getHeight();
			T *d = static_cast<T *>(ptr);
			correctBorderLine(d, height, width, 2);
			d += width - 1;
			correctBorderLine(d, height, width, 2);
		}

		void correctBorderRows(void *ptr)
		{
			int width = m_size.getWidth();
			int mod_height = m_mod_size.getHeight();
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

	ChipBorderCorrBase *createChipBorderCorr(ImageType image_type);

	double getBorderCorrFactor(int det, int line);
	int getInterModuleGap(int det);

	static const int ChipSize;
	static const int ChipGap;
	static const int HalfModuleChips;

	int m_nb_det_modules;
	FrameDim m_recv_frame_dim;
	int m_recv_half_frame_packets;
	ChipBorderCorrBase *m_corr;
	RecvGeometryList m_recv_geom_list;
};


} // namespace SlsDetector

} // namespace lima



#endif // __SLS_DETECTOR_CAMERA_H
