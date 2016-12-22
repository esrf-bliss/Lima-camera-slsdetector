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
#include "slsReceiver.h"

#include "lima/RegExUtils.h"

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

struct ReceiverObj {
	vector<string> argv_list;
	AutoPtr<char *> argv;
	AutoPtr<slsReceiver> recv;

	ReceiverObj(int rx_port, int mode)
	{
		argv_list.push_back("slsReceiver");
		ostringstream os;
		argv_list.push_back("--rx_tcpport");
		os << rx_port;
		argv_list.push_back(os.str());
		argv_list.push_back("--mode");
		os.str("");
		os << mode;
		argv_list.push_back(os.str());
		unsigned int argc = argv_list.size();
		argv = new char *[argc];
		cout << "  argv[" << argc << "]=\"";
		for (unsigned int i = 0; i < argc; ++i) {
			argv[i] = const_cast<char *>(argv_list[i].c_str());
			cout << (i ? " " : "") << argv[i];
		}
		cout << endl;
		int init_ret;
		recv = new slsReceiver(argc, argv, init_ret);
		if (init_ret == slsReceiverDefs::FAIL)
			throw runtime_error("Error creating slsReceiver");
		if (recv->start() == slsReceiverDefs::FAIL) 
			throw runtime_error("Error starting slsReceiver");
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
	cout << "Receivers:" << endl;
	map<int, int>::const_iterator mit, mend = recv_port_map.end();
	for (mit = recv_port_map.begin(); mit != mend; ++mit) {
		unsigned int id = mit->first;
		if (id >= host_name_list.size()) {
			cerr << "Detector id too high: " << id << endl;
			return 1;
		}
		const string& host_name = host_name_list[id];
		int rx_port = mit->second;
		int mode = (id % 2);
		cout << "  " << host_name << ": "
		     << "receiver port=" << rx_port << ", "
		     << "mode=" << mode << endl;

		AutoPtr<ReceiverObj> recv_obj = new ReceiverObj(rx_port, mode);
		recv_list.push_back(recv_obj);
	}

	multiSlsDetector det(0);
	det.readConfigurationFile(config_file_name.c_str());

	return 0;
}
