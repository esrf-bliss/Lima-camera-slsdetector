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

typedef RegEx::SingleMatchType SingleMatch;
typedef RegEx::FullMatchType FullMatch;
typedef RegEx::MatchListType MatchList;
typedef MatchList::const_iterator MatchListIt;

const int FRAME_PACKETS = 128;

Mutex mutex;

bool print_frame = false;

struct ReceiverData
{
	int idx;
	int rx_port;
	int mode;
	int frame;
	int size;
	int packets;
};	

int start_callback(char *fpath, char *fname, int fidx, int dsize, void *priv)
{
	AutoLock<Mutex> l(mutex);

	ReceiverData *data = static_cast<ReceiverData *>(priv);
	if (print_frame) {
		cout << "********* Start! *******" << endl;
		cout << "fpath=" << fpath << ", fname=" << fname << ", " 
		     << "fidex=" << fidx << ", dsize=" << dsize 
		     << "idx=" << data->idx << endl;
	}
	data->packets = 0;

	return DO_NOTHING;
}

void frame_callback(int frame, char *dptr, int dsize, FILE *f, char *guidptr,
		    void *priv)
{
	AutoLock<Mutex> l(mutex);

	ReceiverData *data = static_cast<ReceiverData *>(priv);
	if (++data->packets == data->size) {
		if (print_frame) {
			cout << "********* Frame! *******" << endl;
			cout << "frame=" << frame << ", "
			     << "dsize=" << dsize << ", "
			     << "idx=" << data->idx << endl;
		}
		data->packets = 0;
	}
}

struct ReceiverObj {
	ReceiverData data;
	vector<string> argv_list;
	AutoPtr<char *> argv;
	AutoPtr<slsReceiverUsers> recv;

	ReceiverObj(const ReceiverData& d) : data(d)
	{
		argv_list.push_back("slsReceiver");
		ostringstream os;
		argv_list.push_back("--rx_tcpport");
		os << data.rx_port;
		argv_list.push_back(os.str());
		argv_list.push_back("--mode");
		os.str("");
		os << data.mode;
		argv_list.push_back(os.str());
		unsigned int argc = argv_list.size();
		argv = new char *[argc];
		cout << "  argv[" << argc << "]=\"";
		for (unsigned int i = 0; i < argc; ++i) {
			argv[i] = const_cast<char *>(argv_list[i].c_str());
			cout << (i ? " " : "") << argv[i];
		}
		cout << endl;
		data.size = FRAME_PACKETS;
		int init_ret;
		recv = new slsReceiverUsers(argc, argv, init_ret);
		if (init_ret == slsReceiverDefs::FAIL)
			throw runtime_error("Error creating slsReceiver");
		if (recv->start() == slsReceiverDefs::FAIL) 
			throw runtime_error("Error starting slsReceiver");
		recv->registerCallBackStartAcquisition(start_callback, &data);
		recv->registerCallBackRawDataReady(frame_callback, &data);
	}

	~ReceiverObj()
	{
		recv->stop();
	}
}; 


int main(int argc, char *argv[])
{
	if (argc < 2) {
		cerr << "Missing config file" << endl;
		return 1;
	}

	int nb_frames = 5;
	if (argc == 3) {
		istringstream is(argv[2]);
		is >> nb_frames;
	}

	string config_file_name = argv[1];
	vector<string> host_name_list;
	map<int, int> recv_port_map;
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
				return 1;
			}
			int rx_tcpport;
			config_file >> rx_tcpport;
			recv_port_map[id] = rx_tcpport;
			continue;
		}
	}

	vector<AutoPtr<ReceiverObj> >recv_list;
	cout << "+++ Receivers:" << endl;
	map<int, int>::const_iterator mit, mend = recv_port_map.end();
	int idx = 0;
	for (mit = recv_port_map.begin(); mit != mend; ++mit, ++idx) {
		ReceiverData data;
		data.idx = idx;
		unsigned int id = mit->first;
		if (id >= host_name_list.size()) {
			cerr << "Detector id too high: " << id << endl;
			return 1;
		}
		const string& host_name = host_name_list[id];
		data.rx_port = mit->second;
		data.mode = (id % 2);
		cout << "  " << host_name << ": "
		     << "receiver port=" << data.rx_port << ", "
		     << "mode=" << data.mode << endl;

		AutoPtr<ReceiverObj> recv_obj = new ReceiverObj(data);
		recv_list.push_back(recv_obj);
	}

	cout << "+++ Creating the multiSlsDetector object ..." << endl;
	multiSlsDetector det(0);
	cout << "+++ Reading configuration file ..." << endl;
	det.readConfigurationFile(config_file_name.c_str());

	cout << "+++ Creating the multiSlsDetectorCommand ..." << endl;
	multiSlsDetectorCommand cmd(&det);

	char **args = new char*[10];
	string ans;

	cout << "+++ Setting timming ..." << endl;
	args[0] = "timing";
	args[1] = "auto";
	cmd.putCommand(2, args);

	{
		ostringstream os;
		os << nb_frames;
		string s = os.str();
		cout << "+++ Setting frames ..." << endl;
		args[0] = "frames";
		args[1] = const_cast<char *>(s.c_str());
		cmd.putCommand(2, args);
	}

	cout << "+++ Setting exptime ..." << endl;
	args[0] = "exptime";
	args[1] = "0.4e-3";
	cmd.putCommand(2, args);

	cout << "+++ Setting period ..." << endl;
	args[0] = "period";
	args[1] = "1.0e-3";
	cmd.putCommand(2, args);

	cout << "+++ Querying status ..." << endl;
	args[0] = "status";
	ans = cmd.getCommand(1, args);
        cout << "  status=" << ans << endl;

	cout << "+++ Starting receivers ..." << endl;
	args[0] = "receiver";
	args[1] = "start";
	ans = cmd.putCommand(2, args);

	cout << "+++ Starting acq ..." << endl;
	args[0] = "status";
	args[1] = "start";
	cmd.putCommand(2, args);

	cout << "+++ Querying status ..." << endl;
	args[0] = "status";
	ans = cmd.getCommand(1, args);
        cout << "  status=" << ans << endl;

	cout << "+++ Waiting for end ..." << endl;
	{
		int frames_caught = 0;
		while (frames_caught < nb_frames) {
			args[0] = "framescaught";
			ans = cmd.getCommand(1, args);
			{
				AutoLock<Mutex> l(mutex);
				cout << "  framescaught=" << ans << endl;
			}
			istringstream is(ans);
			is >> frames_caught;
			usleep(100000);
		}
	}

	cout << "+++ Stopping acq ..." << endl;
	args[0] = "status";
	args[1] = "stop";
	cmd.putCommand(2, args);

	cout << "+++ Stopping receivers ..." << endl;
	args[0] = "receiver";
	args[1] = "stop";
	cmd.putCommand(2, args);

	cout << "+++ Querying status ..." << endl;
	args[0] = "status";
	ans = cmd.getCommand(1, args);
        cout << "  status=" << ans << endl;

	cout << "+++ Starting cleanup ..." << endl;

	return 0;
}
