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

#include "lima/SizeUtils.h"
#include "lima/RegExUtils.h"
#include "lima/ThreadUtils.h"
#include "lima/MemUtils.h"
#include "lima/HwBufferMgr.h"

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

#define CHIP_SIZE		256
#define HALF_MODULE_CHIPS	4
#define PACKET_DATA_LEN		(4 * 1024)

enum State {
	Idle, Init, Starting, Running, StopReq, Stopping, Stopped,
};

class Camera
{
	DEB_CLASS_NAMESPC(DebModCamera, "Camera", "SlsDetector");

public:
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

		FrameMap();
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
	};

	Camera(std::string config_fname);
	virtual ~Camera();

	void setBufferCbMgr(StdBufferCbMgr *buffer_cb_mgr)
	{ m_buffer_cb_mgr = buffer_cb_mgr; }

	void setSaveRaw(bool save_raw)
	{ m_save_raw = save_raw; }

	void setNbFrames(int  nb_frames);
	void getNbFrames(int& nb_frames);
	void setExpTime(double  exp_time);
	void getExpTime(double& exp_time);
	void setFramePeriod(double  frame_period);
	void getFramePeriod(double& frame_period);

	State getState();
	void waitState(State state);
	State waitNotState(State state);

	void prepareAcq();
	void startAcq();
	void stopAcq();

	int getNbHalfModules()
	{ return m_input_data->host_name_list.size(); }

	void getFrameDim(FrameDim& frame_dim, bool raw = false);

	int getFramesCaught();
	std::string getStatus();

	const FrameMap& getRecvMap()
	{ return m_recv_map; }

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
		typedef unsigned char Byte;
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
			char data[PACKET_DATA_LEN];
			struct post 
			{
				Long frame;
				Byte res_1[2];	// 0x00
				Byte next;	// 32-bit
				Byte res_2;     // 0x00
			} post;
		};

		static int getPacketLen()
		{ return sizeof(Packet); }

		void getFrameDim(FrameDim& frame_dim, bool raw = false);
		int getFramePackets();

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

	AutoMutex lock()
	{ return AutoMutex(m_cond.mutex()); }

	State getEffectiveState();

	char *getFrameBufferPtr(int frame_nb);
	void removeSharedMem();
	void createReceivers();

	void receiverFrameFinished(int frame, Receiver *recv);
	void frameFinished(int frame);

	void putCmd(const std::string& s);
	std::string getCmd(const std::string& s);

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

std::ostream& operator <<(std::ostream& os, const Camera::FrameMap& m);
std::ostream& operator <<(std::ostream& os, const Camera::FrameMap::List& l);
std::ostream& operator <<(std::ostream& os, const Camera::FrameMap::Map& m);

} // namespace SlsDetector

} // namespace lima



#endif // __SLS_DETECTOR_CAMERA_H
