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

#ifndef __TEST_SLS_DETECTOR_H
#define __TEST_SLS_DETECTOR_H

#include "multiSlsDetector.h"
#include "multiSlsDetectorCommand.h"
#include "slsReceiverUsers.h"
#include "receiver_defs.h"

#include "lima/RegExUtils.h"
#include "lima/ThreadUtils.h"

#include <iostream>
#include <string>
#include <stdexcept>
#include <cstdlib>
#include <set>

#define PRINT_POLICY_NONE	0
#define PRINT_POLICY_START	(1 << 0)
#define PRINT_POLICY_MAP	(1 << 1)
#define PRINT_POLICY_ACQ	(1 << 2)
#define PRINT_POLICY_RECV	(1 << 3)
#define PRINT_POLICY_PACKET	(1 << 4)

namespace lima 
{

class Args
{
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

class SlsDetectorAcq
{
public:
	typedef RegEx::SingleMatchType SingleMatch;
	typedef RegEx::FullMatchType FullMatch;
	typedef RegEx::MatchListType MatchList;
	typedef MatchList::const_iterator MatchListIt;

	typedef std::vector<std::string> StringList;
	typedef StringList HostnameList;
	typedef std::map<int, int> RecvPortMap;
	typedef std::map<int, int> FrameRecvMap;

	static const int FRAME_PACKETS;

	struct AppInputData
	{
		std::string config_file_name;
		HostnameList host_name_list;
		RecvPortMap recv_port_map;
		AppInputData(std::string cfg_fname);
		void parseConfigFile();
	};

	class FrameMap
	{
	public:
		typedef std::set<int> List;
		typedef std::map<int, List> Map;

		class Callback
		{
		public:
			Callback();
			virtual ~Callback();
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

	class ReceiverObj {
	public:
		ReceiverObj(SlsDetectorAcq *acq, 
			    int idx, int rx_port, int mode);
		~ReceiverObj();
		void start();

	private:
		class FrameFinishedCallback : public FrameMap::Callback
		{
		public:
			FrameFinishedCallback(ReceiverObj *r, int p);
			virtual void frameFinished(int frame);
		private:
			ReceiverObj *m_recv;
			int m_print_policy;
		};

		friend class SlsDetectorAcq;
		friend class FrameFinishedCallback;

		static int startCallback(char *fpath, char *fname, int fidx, 
					 int dsize, void *priv);

		static void frameCallback(int frame, char *dptr, int dsize, 
					  FILE *f, char *guidptr, void *priv);

		int startCallback(char *fpath, char *fname, int fidx, 
				  int dsize);
		void frameCallback(int frame, char *dptr, int dsize, FILE *f, 
				   char *guidptr);

		Mutex& m_mutex;
		SlsDetectorAcq *m_acq;
		int m_idx;
		int m_rx_port;
		int m_mode;
		FrameMap m_packet_map;
		int m_packet_idx;
		Args m_args;
		AutoPtr<slsReceiverUsers> m_recv;
		AutoPtr<FrameFinishedCallback> m_cb;
	}; 

	typedef std::vector<AutoPtr<ReceiverObj> > RecvList;

	SlsDetectorAcq(std::string config_fname);
	virtual ~SlsDetectorAcq();

	void setNbFrames(int nb_frames)
	{ m_nb_frames = nb_frames; }

	void setExpTime(double exp_time)
	{ m_exp_time = exp_time; }

	void setFramePeriod(double frame_period)
	{ m_frame_period = frame_period; }

	void setPrintPolicy(int print_policy)
	{ m_print_policy = print_policy; }

	void prepareAcq();
	void startAcq();
	void stopAcq();
	void waitAcq();

private:
	friend class ReceiverObj;

	class FrameFinishedCallback : public FrameMap::Callback
	{
	public:
		FrameFinishedCallback(SlsDetectorAcq *a);
		virtual void frameFinished(int frame);
	private:
		SlsDetectorAcq *m_acq;
	};

	void createReceivers();
	void receiverFrameFinished(int frame, ReceiverObj *recv);
	void frameFinished(int frame);

	void putCmd(const std::string& s);
	std::string getCmd(const std::string& s);

	Mutex m_mutex;
	int m_print_policy;
	AutoPtr<AppInputData> m_input_data;
	RecvList m_recv_list;
	AutoPtr<multiSlsDetector> m_det;
	AutoPtr<multiSlsDetectorCommand> m_cmd;
	int m_nb_frames;
	double m_exp_time;
	double m_frame_period;
	bool m_started;
	FrameMap m_recv_map;
	AutoPtr<FrameFinishedCallback> m_frame_cb;
};

std::ostream& operator <<(std::ostream& os, 
			  const SlsDetectorAcq::FrameMap& m);
std::ostream& operator <<(std::ostream& os, 
			  const SlsDetectorAcq::FrameMap::List& l);
std::ostream& operator <<(std::ostream& os, 
			  const SlsDetectorAcq::FrameMap::Map& m);

} // namespace lima



#endif // __TEST_SLS_DETECTOR_H
