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

#include "test_sls_detector.h"
#include "lima/Timestamp.h"

#define THROW(x)					\
	do {						\
		throw runtime_error(x);			\
	} while (0)


using namespace std;
using namespace lima;


Args::Args()
{
	update_argc_argv();
}

Args::Args(unsigned int argc, char *argv[])
{
	for (unsigned int i = 0; i < argc; ++i)
		m_arg_list.push_back(argv[i]);
	update_argc_argv();
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

void Args::clear()
{
	m_arg_list.clear();
	update_argc_argv();
}

Args& Args::operator =(const std::string& s)
{
	set(s);
	return *this;
}

string Args::pop_front()
{
	string s = m_arg_list[0];
	erase(0);
	return s;
}

void Args::erase(int pos)
{
	m_arg_list.erase(m_arg_list.begin() + pos);
	update_argc_argv();
}

void Args::update_argc_argv()
{
	m_argc = m_arg_list.size();
	m_argv = new char *[m_argc + 1];
	for (unsigned int i = 0; i < m_argc; ++i)
		m_argv[i] = const_cast<char *>(m_arg_list[i].c_str());
	m_argv[m_argc] = NULL;
}


const double SlsDetectorAcq::WAIT_SLEEP_TIME = 0.2;

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


SlsDetectorAcq::FrameMap::Callback::Callback()
	: m_map(NULL)
{
}

SlsDetectorAcq::FrameMap::Callback::~Callback()
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
		m_cb->m_map = NULL;
}

void SlsDetectorAcq::FrameMap::setNbItems(int nb_items)
{
	m_nb_items = nb_items;
}

void SlsDetectorAcq::FrameMap::clear()
{
	m_map.clear();
	m_non_seq_finished_frames.clear();
	m_last_seq_finished_frame = -1;
}

void SlsDetectorAcq::FrameMap::setCallback(Callback *cb)
{ 
	if (m_cb)
		m_cb->m_map = NULL;
	m_cb = cb; 
	if (m_cb)
		m_cb->m_map = this;
}

void SlsDetectorAcq::FrameMap::frameItemFinished(int frame, int item)
{
	if (m_nb_items == 0)		
		THROW("SlsDetectorAcq::FrameMap::frameItemFinished: no items");
	else if ((item < 0) || (item >= m_nb_items))
		THROW("SlsDetectorAcq::FrameMap::frameItemFinished: bad item");
	Map::iterator mit = m_map.find(frame);
	if (mit == m_map.end()) {
		for (int i = 0; i < m_nb_items; ++i)
			if (i != item)
				m_map[frame].insert(i);
		return;
	}

	List& item_list = mit->second;
	List::iterator lit = item_list.find(item);
	if (lit == item_list.end()) {
		ostringstream os;
		os << "SlsDetectorAcq::FrameMap::frameItemFinished: item "
		   << item << " already finished for frame " << frame;
		THROW(os.str());
	}

	item_list.erase(lit);
	if (!item_list.empty())
		return;
	m_map.erase(mit);

	int &last = m_last_seq_finished_frame;
	List &waiting = m_non_seq_finished_frames;
	if (frame == last + 1) {
		++last;
		while ((lit = waiting.find(last + 1)) != waiting.end()) {
			waiting.erase(lit);
			++last;
		}
	} else {
		waiting.insert(frame);
	}
	if (m_cb)
		m_cb->frameFinished(frame);
}

ostream& lima::operator <<(ostream& os, const SlsDetectorAcq::FrameMap& m)
{
	os << "<";
	os << "LastSeqFinishedFrame=" << m.getLastSeqFinishedFrame() << ", "
	   << "NonSeqFinishedFrames=" << m.getNonSeqFinishedFrames() << ", "
	   << "FramePendingItemsMap=" << m.getFramePendingItemsMap();
	return os << ">";
}

ostream& lima::operator <<(ostream& os, const SlsDetectorAcq::FrameMap::List& l)
{
	os << "[";
	typedef SlsDetectorAcq::FrameMap::List List;
	List::const_iterator it, end = l.end();
	bool first;
	for (it = l.begin(), first = true; it != end; ++it, first = false)
		os << (first ? "" : ", ") << *it;
	return os << "]";
}

ostream& lima::operator <<(ostream& os, const SlsDetectorAcq::FrameMap::Map& m)
{
	os << "{";
	typedef SlsDetectorAcq::FrameMap::Map Map;
	Map::const_iterator it, end = m.end();
	bool first;
	for (it = m.begin(), first = true; it != end; ++it, first = false)
		os << (first ? "" : ", ") << it->first << ": " << it->second;
	return os << "}";
}

SlsDetectorAcq::
ReceiverObj::FrameFinishedCallback::FrameFinishedCallback(ReceiverObj *r, 
							  int p) 
	: m_recv(r), m_print_policy(p)
{
}

void SlsDetectorAcq::
ReceiverObj::FrameFinishedCallback::frameFinished(int frame) 
{
	if (m_print_policy & PRINT_POLICY_RECV) {
		cout << "********* End! *******" << endl;
		cout << "frame=" << frame << ", "
		     << "idx=" << m_recv->m_idx << endl;
	}

	m_recv->m_acq->receiverFrameFinished(frame, m_recv);
}

SlsDetectorAcq::ReceiverObj::ReceiverObj(SlsDetectorAcq *acq, 
					 int idx, int rx_port, int mode)
	: m_mutex(acq->m_mutex), m_acq(acq), 
	  m_idx(idx), m_rx_port(rx_port), m_mode(mode)
{
	m_cb = new FrameFinishedCallback(this, m_acq->m_print_policy);

	int nb_packets = getFramePackets();
	m_packet_map.setNbItems(nb_packets);
	m_packet_map.setCallback(m_cb);

	ostringstream os;
	os << "slsReceiver"
	   << " --rx_tcpport " << m_rx_port << " --mode " << m_mode;
	m_args.set(os.str());

	start();

	m_recv->registerCallBackStartAcquisition(startCallback, this);
	m_recv->registerCallBackRawDataReady(frameCallback, this);
}

SlsDetectorAcq::ReceiverObj::~ReceiverObj()
{
	m_recv->stop();
}

void SlsDetectorAcq::ReceiverObj::start()
{	
	int init_ret;
	m_recv = new slsReceiverUsers(m_args.size(), m_args, init_ret);
	if (init_ret == slsReceiverDefs::FAIL)
		THROW("Error creating slsReceiver");
	if (m_recv->start() == slsReceiverDefs::FAIL) 
		THROW("Error starting slsReceiver");
}

int SlsDetectorAcq::ReceiverObj::startCallback(char *fpath, char *fname, 
					       int fidx, int dsize, void *priv)
{
	ReceiverObj *recv = static_cast<ReceiverObj *>(priv);
	return recv->startCallback(fpath, fname, fidx, dsize);
}

void SlsDetectorAcq::ReceiverObj::frameCallback(int frame, char *dptr, 
						int dsize, FILE *f, 
						char *guidptr, void *priv)
{
	ReceiverObj *recv = static_cast<ReceiverObj *>(priv);
	recv->frameCallback(frame, dptr, dsize, f, guidptr);
}

int SlsDetectorAcq::ReceiverObj::startCallback(char *fpath, char *fname, 
					       int fidx, int dsize)
{
	AutoLock<Mutex> l(m_mutex);

	if (dsize != getPacketLen()) {
		cerr << "!!!! Warning !!!!" << endl;
		cerr << "dsize=" << dsize << ", PACKET_LEN=" << getPacketLen() 
		     << endl;
	}

	if (m_acq->m_print_policy & PRINT_POLICY_START) {
		cout << "********* Start! *******" << endl;
		cout << "fpath=" << fpath << ", fname=" << fname << ", " 
		     << "fidex=" << fidx << ", dsize=" << dsize << ", "
		     << "idx=" << m_idx << endl;
	}

	return DO_NOTHING;
}

void SlsDetectorAcq::ReceiverObj::frameCallback(int frame, char *dptr, 
						int dsize, FILE *f, 
						char *guidptr)
{
	Packet *p = static_cast<Packet *>(static_cast<void *>(dptr));
	int packet_idx = p->pre.idx;
	if (p->pre.flags & 0x20)
		packet_idx += 0x40;
	
	AutoLock<Mutex> l(m_mutex);
	if (m_acq->m_print_policy & PRINT_POLICY_PACKET) {
		cout << "********* Frame! *******" << endl;
		cout << "frame=" << frame << ", dsize=" << dsize << ", "
		     << "PACKET_LEN=" << getPacketLen() << ", "
		     << "idx=" << m_idx << ", packet=" << packet_idx << endl;
	}
	char *buffer = m_acq->getBufferPtr(frame);
	bool raw = m_acq->m_save_raw;
	l.unlock();

	int recv_size = raw ? getRawImageSize() : getImageSize();
	int xfer_len = raw ? sizeof(*p) : sizeof(p->data);
	void *src = raw ? dptr : p->data;
	void *dest = buffer + recv_size * m_idx + xfer_len * packet_idx;
	memcpy(dest, src, xfer_len);

	l.lock();
	m_packet_map.frameItemFinished(frame, packet_idx);
}

SlsDetectorAcq::FrameFinishedCallback::FrameFinishedCallback(SlsDetectorAcq *a)
	 : m_acq(a)
{
}

void SlsDetectorAcq::FrameFinishedCallback::frameFinished(int frame)
{
	m_acq->frameFinished(frame);
}

SlsDetectorAcq::SlsDetectorAcq(string config_fname) 
	: m_print_policy(PRINT_POLICY_NONE), 
	  m_nb_frames(1), 
	  m_exp_time(0.99),
	  m_frame_period(1.0), 
	  m_started(false), 
	  m_pixel_depth(2), 
	  m_save_raw(true)
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
		unsigned int id = mit->first;
		if (id >= m_input_data->host_name_list.size()) {
			cerr << "Detector id too high: " << id << endl;
			exit(1);
		}
		const string& host_name = m_input_data->host_name_list[id];
		int rx_port = mit->second;
		int mode = (id % 2);
		cout << "  " << host_name << ": "
		     << "receiver port=" << rx_port << ", "
		     << "mode=" << mode << endl;

		AutoPtr<ReceiverObj> recv_obj = new ReceiverObj(this, idx, 
								rx_port, mode);
		m_recv_list.push_back(recv_obj);
	}

	m_frame_cb = new FrameFinishedCallback(this);
	m_recv_map.setNbItems(recv_port_map.size());
	m_recv_map.setCallback(m_frame_cb);
}

void SlsDetectorAcq::putCmd(const string& s)
{
	Args args(s);
	m_cmd->putCommand(args.size(), args);
}

string SlsDetectorAcq::getCmd(const string& s)
{
	Args args(s);
	return m_cmd->getCommand(args.size(), args);
}

void SlsDetectorAcq::prepareAcq()
{
	ostringstream os;

	allocBuffers();

	cout << "+++ Setting timming ..." << endl;
	putCmd("timing auto");

	cout << "+++ Setting frames ..." << endl;
	os.str("");
	os << "frames " << m_nb_frames;
	putCmd(os.str());

	cout << "+++ Setting exptime ..." << endl;
	os.str("");
	os << "exptime " << m_exp_time;
	putCmd(os.str());

	cout << "+++ Setting period ..." << endl;
	os.str("");
	os << "period " << m_frame_period;
	putCmd(os.str());

	cout << "+++ Querying status ..." << endl;
        cout << "  status=" << getCmd("status") << endl;

	cout << "+++ Starting receivers ..." << endl;
	putCmd("receiver start");
}

void SlsDetectorAcq::allocBuffers()
{
	long buffer_size = getImageSize();
	cout << "+++ Allocating " << m_nb_frames << " buffers "
	     << "(" << buffer_size << ") ..." << endl;
	m_buffer_list = new AutoPtr<MemBuffer>[m_nb_frames];
	for (int i = 0; i < m_nb_frames; ++i)
		m_buffer_list[i] = new MemBuffer(buffer_size);
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
	int finished_frames = 0;
	string last_msg;
	Timestamp last_change = Timestamp::now();
	double timeout = 2;
	unsigned long usec = WAIT_SLEEP_TIME * 1e6 + 0.1;

	while (finished_frames < m_nb_frames) {
		if (Timestamp::now() - last_change > timeout) {
			cout << "!!!!! Blocked "
			     << "(" << timeout <<"s) !!!!!" << endl;
			break;
		}

		usleep(usec);

		AutoLock<Mutex> l(m_mutex);
		int frames_caught;
		string ans = getCmd("framescaught");
		istringstream is(ans);
		is >> frames_caught;
		finished_frames = m_recv_map.getLastSeqFinishedFrame() + 1;
		ostringstream os;
		os << "  framescaught=" << frames_caught << ", "
		   << "finished_frames=" << finished_frames;
		if (m_print_policy & PRINT_POLICY_MAP) {
			os << ", " << "m_recv_map=" << m_recv_map;
		}
		l.unlock();

		string msg = os.str();
		cout << msg << endl;
		if (msg != last_msg)
			last_change = Timestamp::now();
		last_msg = msg;
	}
}

void SlsDetectorAcq::receiverFrameFinished(int frame, ReceiverObj *recv)
{
	m_recv_map.frameItemFinished(frame, recv->m_idx);
}

void SlsDetectorAcq::frameFinished(int frame)
{
	if (m_print_policy & PRINT_POLICY_ACQ) {
		cout << "********* Finished! *******" << endl;
		cout << "frame=" << frame << endl;
	}
}

int SlsDetectorAcq::getImageSize()
{
	ReceiverObj *recv = m_recv_list[0];
	int size = m_save_raw ? recv->getRawImageSize() : recv->getImageSize();
	return size * getNbHalfModules(); 
}

void save_raw_data(string out_dir, int start_frame, int nb_frames, 
		   SlsDetectorAcq& acq)
{
	for (int i = 0; i < nb_frames; ++i) {
		ostringstream os;
		os << out_dir << "/eiger.bin." << i;
		cout << "+++ Saving raw to " << os.str() << " ..." << endl;
		ofstream of(os.str().c_str());
		char *buffer = acq.getBufferPtr(start_frame + i);
		of.write(buffer, acq.getImageSize());
	}
}

class EdfHeaderKey
{
public:
	EdfHeaderKey(const string& key) : m_key(key)
	{}
private:
	friend ostream& operator <<(ostream& os, const EdfHeaderKey& h);
	string m_key;
};

ostream& operator <<(ostream& os, const EdfHeaderKey& h)
{
	return os << setiosflags(ios::left) << resetiosflags(ios::right)
		  << setw(14) << setfill(' ') << h.m_key << " = ";
}

void save_edf_frame(ofstream& of, int acq_idx, int edf_idx, SlsDetectorAcq& acq)
{
	ostringstream os;
	os << "{" << endl;
	os << EdfHeaderKey("HeaderID") << setiosflags(ios::right) 
	   << "EH:" << setfill('0') << setw(6) << (edf_idx + 1) 
	   << ":" << setfill('0') << setw(6) << 0 
	   << ":" << setfill('0') << setw(6) << 0 << "; " << endl;
	os << EdfHeaderKey("ByteOrder") << "LowByteFirst" << "; " << endl;
	os << EdfHeaderKey("DataType") << "UnsignedShort" << "; " << endl;
	os << EdfHeaderKey("Size") << acq.getImageSize() << "; " << endl;
	os << EdfHeaderKey("Dim_1") << (CHIP_SIZE * 4) << "; " << endl;
	os << EdfHeaderKey("Dim_2") << (CHIP_SIZE * acq.getNbHalfModules()) 
	   << "; " << endl;
	os << EdfHeaderKey("Image") << edf_idx << "; " << endl;
	os << EdfHeaderKey("acq_frame_nb") << edf_idx << "; " << endl;

	const int HEADER_BLOCK = 1024;
	int rem = (HEADER_BLOCK - 2) - os.str().size() % HEADER_BLOCK;
	if (rem < 0)
		rem += HEADER_BLOCK;
	os << string(rem, '\n') << "}" << endl;
	of << os.str();

	of.write(acq.getBufferPtr(acq_idx), acq.getImageSize());
}

void save_edf_data(string out_dir, int start_frame, int nb_frames, 
		   SlsDetectorAcq& acq)
{
	ostringstream os;
	os << out_dir << "/eiger.edf";
	cout << "+++ Saving EDF to " << os.str() << " ..." << endl;
	ofstream of(os.str().c_str());
	for (int i = 0; i < nb_frames; ++i)
		save_edf_frame(of, start_frame + i, i, acq);
}

AppPars::AppPars()
{
	loadDefaults();
	loadOpts();
}

void AppPars::loadDefaults()
{
	nb_frames = 10;
	exp_time = 2.0e-3;
	frame_period = 2.5e-3;
	print_policy = PRINT_POLICY_NONE;
	save_raw = false;
	out_dir = "/tmp";
}

void AppPars::loadOpts()
{
	AutoPtr<ArgOptBase> o;

	o = new ArgOpt<string>(config_fname, "-c", "--config", 
			       "config file name");
	m_opt_list.insert(o);

	o = new ArgOpt<int>(nb_frames, "-n", "--nb-frames", 
			       "number of frames");
	m_opt_list.insert(o);

	o = new ArgOpt<double>(exp_time, "-e", "--exp-time", 
			       "exposure time");
	m_opt_list.insert(o);

	o = new ArgOpt<double>(frame_period, "-p", "--frame-period", 
			       "frame period");
	m_opt_list.insert(o);

	o = new ArgOpt<int>(print_policy, "-l", "--print-policy", 
			    "print policy");
	m_opt_list.insert(o);

	o = new ArgOpt<bool>(save_raw, "-r", "--save-raw");
	m_opt_list.insert(o);

	o = new ArgOpt<string>(out_dir, "-o", "--out-dir",
			       "out_dir");
	m_opt_list.insert(o);
}

void AppPars::parseArgs(Args& args)
{
	string prog_name = args.pop_front();

	while (args && (*args[0] == '-')) {
		OptList::iterator it, end = m_opt_list.end();
		for (it = m_opt_list.begin(); it != end; ++it) {
			if ((*it)->check(args))
				break;
		}
	}

	if (config_fname.empty()) {
		cerr << "Missing config file" << endl;
		exit(1);
	}
}

int main(int argc, char *argv[])
{
	AppPars pars;
	Args args(argc, argv);
	pars.parseArgs(args);

	try {
		SlsDetectorAcq acq(pars.config_fname);
		acq.setPrintPolicy(pars.print_policy);
		acq.setNbFrames(pars.nb_frames);
		acq.setExpTime(pars.exp_time);
		acq.setFramePeriod(pars.frame_period);
		acq.setSaveRaw(pars.save_raw);
		acq.prepareAcq();
		acq.startAcq();
		acq.waitAcq();
		if (pars.save_raw) {
			save_raw_data(pars.out_dir, 0, pars.nb_frames, acq);
		} else {
			save_edf_data(pars.out_dir, 0, pars.nb_frames, acq);
		}
	} catch (string s) {
		cerr << "Exception: " << s << endl;
		exit(1);
	} catch (...) {
		cerr << "Exception" << endl;
		exit(1);
	}

	return 0;
}
