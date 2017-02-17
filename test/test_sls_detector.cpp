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

using namespace std;
using namespace lima;

typedef vector<string> StringList;

class Args
{
public:
	Args();
	Args(const string& s);
	Args(const Args& o);

	void set(const string& s);
	void clear();

	unsigned int argc()
	{ return m_argc; }
	char **argv()
	{ return m_argv; }

	Args& operator =(const string& s);

private:
	void update_argc_argv();

	StringList m_arg_list;
	unsigned int m_argc;
	AutoPtr<char *, true> m_argv;
};

Args::Args() : m_argc(0)
{
}

Args::Args(const string& s)
{
	set(s);
}

Args::Args(const Args& o) : m_arg_list(o.m_arg_list)
{
	update_argc_argv();
}

void Args::set(const string& s)
{
	m_arg_list.clear();
	istringstream is(s);
	while (is) {
		string token;
		is >> token;
		m_arg_list.push_back(token);
	}
	update_argc_argv();
}

void Args::update_argc_argv()
{
	m_argc = m_arg_list.size();
	m_argv = new char *[m_argc];
	for (unsigned int i = 0; i < m_argc; ++i)
		m_argv[i] = const_cast<char *>(m_arg_list[i].c_str());
}


class SlsDetectorAcq
{
public:
	typedef RegEx::SingleMatchType SingleMatch;
	typedef RegEx::FullMatchType FullMatch;
	typedef RegEx::MatchListType MatchList;
	typedef MatchList::const_iterator MatchListIt;

	typedef StringList HostnameList;
	typedef map<int, int> RecvPortMap;
	typedef map<int, int> FrameRecvMap;

	static const int FRAME_PACKETS;

	struct AppInputData
	{
		string config_file_name;
		HostnameList host_name_list;
		RecvPortMap recv_port_map;
		AppInputData(string cfg_fname);
		void parseConfigFile();
	};

	class FrameMap
	{
	public:
		typedef map<int, int> Map;
		typedef list<int> List;

		class Callback
		{
		public:
			Callback(FrameMap *map);
			virtual ~Callback();
			virtual void finishedFrame(int frame) = 0;
		private:
			friend class FrameMap;
			FrameMap *m_map;
		};

		FrameMap();
		~FrameMap();
		
		void setNbItems(int nb_items);
		void clear();
		void frameItemFinished(int frame, int item);
		
		int getLastSeqFinishedFrame()
		{ return m_last_seq_finished_frame; }

		const List getNonSeqFinishedFrames()
		{ return m_non_seq_finished_frames; }

	private:
		friend class Callback;

		Map m_map;
		List m_non_seq_finished_frames;
		int m_last_seq_finished_frame;
		Callback *m_cb;
	};

	struct ReceiverData
	{
		typedef map<int, int> FramePacketMap;
		SlsDetectorAcq *acq;
		int idx;
		int rx_port;
		int mode;
		int frame;
		int size;
		FramePacketMap packets;
		ReceiverData(SlsDetectorAcq *a, int i) : acq(a), idx(i) 
		{}
	};

	struct ReceiverObj {
		ReceiverData data;
		Args args;
		AutoPtr<slsReceiverUsers> recv;

		ReceiverObj(const ReceiverData& d);
		~ReceiverObj();
		void start();
		void registerCallbacks();

		static int startCallback(char *fpath, char *fname, int fidx, 
					 int dsize, void *priv);

		static void frameCallback(int frame, char *dptr, int dsize, 
					  FILE *f, char *guidptr, void *priv);
	}; 

	typedef vector<AutoPtr<ReceiverObj> > RecvList;

	SlsDetectorAcq(string config_fname);
	virtual ~SlsDetectorAcq();

	void setNbFrames(int nb_frames)
	{ m_nb_frames = nb_frames; }

	void prepareAcq();
	void startAcq();
	void stopAcq();
	void waitAcq();

private:
	friend struct ReceiverObj;

	int startCallback(ReceiverData *data, char *fpath, char *fname, 
			  int fidx, int dsize);
	void frameCallback(ReceiverData *data, int frame, char *dptr, 
			   int dsize, FILE *f, char *guidptr);

	void createReceivers();

	void putCmd(const string& s);
	string getCmd(const string& s);

	Mutex m_mutex;
	bool m_print_frame;
	AutoPtr<AppInputData> m_input_data;
	RecvList m_recv_list;
	AutoPtr<multiSlsDetector> m_det;
	AutoPtr<multiSlsDetectorCommand> m_cmd;
	int m_nb_frames;
	bool m_started;
};

const int SlsDetectorAcq::FRAME_PACKETS = 128;

SlsDetectorAcq::AppInputData::AppInputData(string cfg_fname) 
  : config_file_name(cfg_fname) 
{
	parseConfigFile();
}

void SlsDetectorAcq::AppInputData::parseConfigFile()
{
	ifstream config_file(config_file_name.c_str());
	while (config_file) {
		string s;
		config_file >> s;
		RegEx re;
		FullMatch full_match;
		MatchList match_list;
		MatchListIt lit, lend;

		re = "hostname";
		if (re.match(s, full_match)) {
			string host_name;
			config_file >> host_name;
			RegEx re("([A-Za-z0-9]+)\\+?");
			re.multiSearch(host_name, match_list);
			lend = match_list.end();
			for (lit = match_list.begin(); lit != lend; ++lit) {
				const FullMatch& full_match = *lit;
				const SingleMatch& single_match = full_match[1];
				host_name_list.push_back(single_match);
			}
			continue;
		}

		re = "([0-9]+):rx_tcpport";
		if (re.match(s, full_match)) {
			istringstream is(full_match[1]);
			int id;
			is >> id;
			if (id < 0) {
				cerr << "Invalid detector id: " << id << endl;
				exit(1);
			}
			int rx_tcpport;
			config_file >> rx_tcpport;
			recv_port_map[id] = rx_tcpport;
			continue;
		}
	}
}


SlsDetectorAcq::FrameMap::Callback(FrameMap *map)
: m_map(map)
{
}

SlsDetectorAcq::FrameMap::~Callback()
{
	if (m_map)
		m_map->m_cb = NULL;
}

SlsDetectorAcq::FrameMap::FrameMap()
	: m_nb_items(0), m_last_seq_finished_frame(-1), m_cb(NULL)
{
}

SlsDetectorAcq::FrameMap::~FrameMap()
{
	if (m_cb)
		m_cb->m_map == NULL;
}

SlsDetectorAcq::FrameMap::setNbItems(int nb_items)
{
	m_nb_items = nb_items;
}

void SlsDetectorAcq::FrameMap::clear()
{
	m_map.clear();
	m_non_seq_finished_frames.clear();
	m_last_seq_finished_frame = -1;
}

void SlsDetectorAcq::FrameMap::frameItemFinished(int frame, int item)
{
	if (m_nb_items == 0)
		throw "SlsDetectorAcq::FrameMap::frameItemFinished: no items";
	xxxx
}

SlsDetectorAcq::ReceiverObj::ReceiverObj(const ReceiverData& d) : data(d)
{
	data.size = FRAME_PACKETS;
	
	ostringstream os;
	os << "slsReceiver"
	   << " --rx_tcpport " << data.rx_port
	   << " --mode " << data.mode;
	args.set(os.str());

	start();
	registerCallbacks();
}

SlsDetectorAcq::ReceiverObj::~ReceiverObj()
{
	recv->stop();
}

void SlsDetectorAcq::ReceiverObj::start()
{	
	int init_ret;
	recv = new slsReceiverUsers(args.argc(), args.argv(), init_ret);
	if (init_ret == slsReceiverDefs::FAIL)
		throw runtime_error("Error creating slsReceiver");
	if (recv->start() == slsReceiverDefs::FAIL) 
		throw runtime_error("Error starting slsReceiver");
}

void SlsDetectorAcq::ReceiverObj::registerCallbacks()
{
	recv->registerCallBackStartAcquisition(startCallback, &data);
	recv->registerCallBackRawDataReady(frameCallback, &data);
}

int SlsDetectorAcq::ReceiverObj::startCallback(char *fpath, char *fname, 
					       int fidx, int dsize, void *priv)
{
	ReceiverData *data = static_cast<ReceiverData *>(priv);
	SlsDetectorAcq *acq = data->acq;
	return acq->startCallback(data, fpath, fname, fidx, dsize);
}

void SlsDetectorAcq::ReceiverObj::frameCallback(int frame, char *dptr, 
						int dsize, FILE *f, 
						char *guidptr, void *priv)
{
	ReceiverData *data = static_cast<ReceiverData *>(priv);
	SlsDetectorAcq *acq = data->acq;
	acq->frameCallback(data, frame, dptr, dsize, f, guidptr);
}

SlsDetectorAcq::SlsDetectorAcq(string config_fname) 
	: m_print_frame(true), m_nb_frames(1), m_started(false)
{
	m_input_data = new AppInputData(config_fname);

	createReceivers();

	cout << "+++ Creating the multiSlsDetector object ..." << endl;
	m_det = new multiSlsDetector(0);
	cout << "+++ Reading configuration file ..." << endl;
	const char *fname = m_input_data->config_file_name.c_str();
	m_det->readConfigurationFile(fname);

	cout << "+++ Creating the multiSlsDetectorCommand ..." << endl;
	m_cmd = new multiSlsDetectorCommand(m_det);
}

SlsDetectorAcq::~SlsDetectorAcq()
{
	cout << "+++ Starting cleanup ..." << endl;
	stopAcq();
}

void SlsDetectorAcq::createReceivers()
{
	cout << "+++ Receivers:" << endl;
	const RecvPortMap& recv_port_map = m_input_data->recv_port_map;
	RecvPortMap::const_iterator mit, mend = recv_port_map.end();
	int idx = 0;
	for (mit = recv_port_map.begin(); mit != mend; ++mit, ++idx) {
		ReceiverData data(this, idx);
		unsigned int id = mit->first;
		if (id >= m_input_data->host_name_list.size()) {
			cerr << "Detector id too high: " << id << endl;
			exit(1);
		}
		const string& host_name = m_input_data->host_name_list[id];
		data.rx_port = mit->second;
		data.mode = (id % 2);
		cout << "  " << host_name << ": "
		     << "receiver port=" << data.rx_port << ", "
		     << "mode=" << data.mode << endl;

		AutoPtr<ReceiverObj> recv_obj = new ReceiverObj(data);
		m_recv_list.push_back(recv_obj);
	}
}

void SlsDetectorAcq::putCmd(const string& s)
{
	Args args(s);
	m_cmd->putCommand(args.argc(), args.argv());
}

string SlsDetectorAcq::getCmd(const string& s)
{
	Args args(s);
	return m_cmd->getCommand(args.argc(), args.argv());
}

void SlsDetectorAcq::prepareAcq()
{
	ostringstream os;

	cout << "+++ Setting timming ..." << endl;
	putCmd("timing auto");

	cout << "+++ Setting frames ..." << endl;
	os.str("");
	os << "frames " << m_nb_frames;
	putCmd(os.str());

	cout << "+++ Setting exptime ..." << endl;
	putCmd("exptime 0.4e-3");

	cout << "+++ Setting period ..." << endl;
	putCmd("period 1.0e-3");

	cout << "+++ Querying status ..." << endl;
        cout << "  status=" << getCmd("status") << endl;

	cout << "+++ Starting receivers ..." << endl;
	putCmd("receiver start");
}

void SlsDetectorAcq::startAcq()
{
	cout << "+++ Starting acq ..." << endl;
	putCmd("status start");

	cout << "+++ Querying status ..." << endl;
        cout << "  status=" << getCmd("status") << endl;

	m_started = true;
}

void SlsDetectorAcq::stopAcq()
{
	if (!m_started)
		return;

	cout << "+++ Stopping acq ..." << endl;
	putCmd("status stop");

	cout << "+++ Stopping receivers ..." << endl;
	putCmd("receiver stop");

	cout << "+++ Querying status ..." << endl;
        cout << "  status=" << getCmd("status") << endl;

	m_started = false;
}

void SlsDetectorAcq::waitAcq()
{
	if (!m_started)
		return;

	cout << "+++ Waiting for end ..." << endl;
	{
		int frames_caught = 0;
		while (frames_caught < m_nb_frames) {
			string ans = getCmd("framescaught");
			istringstream is(ans);
			is >> frames_caught;
			{
				AutoLock<Mutex> l(m_mutex);
				cout << "  framescaught=" << ans << endl;
			}
			usleep(100000);
		}
	}
}

int SlsDetectorAcq::startCallback(ReceiverData *data, char *fpath, 
				  char *fname, int fidx, int dsize)
{
	AutoLock<Mutex> l(m_mutex);

	if (m_print_frame) {
		cout << "********* Start! *******" << endl;
		cout << "fpath=" << fpath << ", fname=" << fname << ", " 
		     << "fidex=" << fidx << ", dsize=" << dsize 
		     << "idx=" << data->idx << endl;
	}

	return DO_NOTHING;
}

void SlsDetectorAcq::frameCallback(ReceiverData *data, int frame, char *dptr,
				   int dsize, FILE *f, char *guidptr)
{
	AutoLock<Mutex> l(m_mutex);
	
	ReceiverData::FramePacketMap& packets = data->packets;
	ReceiverData::FramePacketMap::iterator it, end = packets.end();
	it = packets.find(frame);
	if (it == end) {
		packets.insert(make_pair(frame, 1));
	} else if (++it->second == data->size) {
		if (m_print_frame) {
			cout << "********* Frame! *******" << endl;
			cout << "frame=" << frame << ", "
			     << "dsize=" << dsize << ", "
			     << "idx=" << data->idx << endl;
		}
		packets.erase(it);
	}
}


int main(int argc, char *argv[])
{
	if (argc < 2) {
		cerr << "Missing config file" << endl;
		exit(1);
	}
	string config_fname = argv[1];

	int nb_frames = 5;
	if (argc == 3) {
		istringstream is(argv[2]);
		is >> nb_frames;
	}

	SlsDetectorAcq acq(config_fname);
	acq.setNbFrames(nb_frames);
	acq.prepareAcq();
	acq.startAcq();
	acq.waitAcq();

	return 0;
}
