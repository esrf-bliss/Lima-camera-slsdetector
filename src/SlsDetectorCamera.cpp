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

#include "SlsDetectorCamera.h"
#include "lima/Timestamp.h"

#define THROW(x)					\
	do {						\
		throw runtime_error(x);			\
	} while (0)


using namespace std;
using namespace lima;
using namespace lima::SlsDetector;


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


const double Camera::WAIT_SLEEP_TIME = 0.2;

Camera::AppInputData::AppInputData(string cfg_fname) 
	: config_file_name(cfg_fname) 
{
	parseConfigFile();
}

void Camera::AppInputData::parseConfigFile()
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


Camera::FrameMap::Callback::Callback()
	: m_map(NULL)
{
}

Camera::FrameMap::Callback::~Callback()
{
	if (m_map)
		m_map->m_cb = NULL;
}

Camera::FrameMap::FrameMap()
	: m_nb_items(0), m_last_seq_finished_frame(-1), m_cb(NULL)
{
}

Camera::FrameMap::~FrameMap()
{
	if (m_cb)
		m_cb->m_map = NULL;
}

void Camera::FrameMap::setNbItems(int nb_items)
{
	m_nb_items = nb_items;
}

void Camera::FrameMap::clear()
{
	m_map.clear();
	m_non_seq_finished_frames.clear();
	m_last_seq_finished_frame = -1;
}

void Camera::FrameMap::setCallback(Callback *cb)
{ 
	if (m_cb)
		m_cb->m_map = NULL;
	m_cb = cb; 
	if (m_cb)
		m_cb->m_map = this;
}

void Camera::FrameMap::frameItemFinished(int frame, int item)
{
	if (m_nb_items == 0)		
		THROW("Camera::FrameMap::frameItemFinished: no items");
	else if ((item < 0) || (item >= m_nb_items))
		THROW("Camera::FrameMap::frameItemFinished: bad item");
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
		os << "Camera::FrameMap::frameItemFinished: item "
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

ostream& lima::SlsDetector::operator <<(ostream& os, 
					const Camera::FrameMap& m)
{
	os << "<";
	os << "LastSeqFinishedFrame=" << m.getLastSeqFinishedFrame() << ", "
	   << "NonSeqFinishedFrames=" << m.getNonSeqFinishedFrames() << ", "
	   << "FramePendingItemsMap=" << m.getFramePendingItemsMap();
	return os << ">";
}

ostream& lima::SlsDetector::operator <<(ostream& os, 
					const Camera::FrameMap::List& l)
{
	os << "[";
	typedef Camera::FrameMap::List List;
	List::const_iterator it, end = l.end();
	bool first;
	for (it = l.begin(), first = true; it != end; ++it, first = false)
		os << (first ? "" : ", ") << *it;
	return os << "]";
}

ostream& lima::SlsDetector::operator <<(ostream& os, 
					const Camera::FrameMap::Map& m)
{
	os << "{";
	typedef Camera::FrameMap::Map Map;
	Map::const_iterator it, end = m.end();
	bool first;
	for (it = m.begin(), first = true; it != end; ++it, first = false)
		os << (first ? "" : ", ") << it->first << ": " << it->second;
	return os << "}";
}

Camera::Receiver::FrameFinishedCallback::FrameFinishedCallback(Receiver *r, int p) 
	: m_recv(r), m_print_policy(p)
{
}

void Camera::Receiver::FrameFinishedCallback::frameFinished(int frame) 
{
	if (m_print_policy & PRINT_POLICY_RECV) {
		cout << "********* End! *******" << endl;
		cout << "frame=" << frame << ", "
		     << "idx=" << m_recv->m_idx << endl;
	}

	m_recv->m_cam->receiverFrameFinished(frame, m_recv);
}

Camera::Receiver::Receiver(Camera *cam, int idx, int rx_port, int mode)
	: m_mutex(cam->m_mutex), m_cam(cam), 
	  m_idx(idx), m_rx_port(rx_port), m_mode(mode)
{
	m_cb = new FrameFinishedCallback(this, m_cam->m_print_policy);

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

Camera::Receiver::~Receiver()
{
	m_recv->stop();
}

void Camera::Receiver::getFrameDim(FrameDim& frame_dim, bool raw)
{
	if (raw) {
		frame_dim.setImageType(Bpp8);
		frame_dim.setSize(Size(getPacketLen() * getFramePackets(), 1));
	} else {
		frame_dim.setImageType(m_cam->m_image_type);
		frame_dim.setSize(Size(HALF_MODULE_CHIPS * CHIP_SIZE, 
				       CHIP_SIZE));
	}
}

int Camera::Receiver::getFramePackets()
{ 
	FrameDim frame_dim;
	getFrameDim(frame_dim, false);
	return frame_dim.getMemSize() / sizeof(Packet::data); 
}

void Camera::Receiver::start()
{	
	int init_ret;
	m_recv = new slsReceiverUsers(m_args.size(), m_args, init_ret);
	if (init_ret == slsReceiverDefs::FAIL)
		THROW("Error creating slsReceiver");
	if (m_recv->start() == slsReceiverDefs::FAIL) 
		THROW("Error starting slsReceiver");
}

int Camera::Receiver::startCallback(char *fpath, char *fname, 
				 int fidx, int dsize, void *priv)
{
	Receiver *recv = static_cast<Receiver *>(priv);
	return recv->startCallback(fpath, fname, fidx, dsize);
}

void Camera::Receiver::frameCallback(int frame, char *dptr, int dsize, FILE *f, 
				  char *guidptr, void *priv)
{
	Receiver *recv = static_cast<Receiver *>(priv);
	recv->frameCallback(frame, dptr, dsize, f, guidptr);
}

int Camera::Receiver::startCallback(char *fpath, char *fname, 
				 int fidx, int dsize)
{
	AutoLock<Mutex> l(m_mutex);

	if (dsize != getPacketLen()) {
		cerr << "!!!! Warning !!!!" << endl;
		cerr << "dsize=" << dsize << ", PACKET_LEN=" << getPacketLen() 
		     << endl;
	}

	if (m_cam->m_print_policy & PRINT_POLICY_START) {
		cout << "********* Start! *******" << endl;
		cout << "fpath=" << fpath << ", fname=" << fname << ", " 
		     << "fidex=" << fidx << ", dsize=" << dsize << ", "
		     << "idx=" << m_idx << endl;
	}

	return DO_NOTHING;
}

void Camera::Receiver::frameCallback(int frame, char *dptr, int dsize, FILE *f, 
				  char *guidptr)
{
	Packet *p = static_cast<Packet *>(static_cast<void *>(dptr));
	bool second_half = p->pre.flags & 0x20;
	int packet_idx = p->pre.idx;
	if (second_half)
		packet_idx += getFramePackets() / 2;
	bool top_half = (m_idx % 2 == 0);

	AutoLock<Mutex> l(m_mutex);
	if (m_cam->m_print_policy & PRINT_POLICY_PACKET) {
		cout << "********* Frame! *******" << endl;
		cout << "frame=" << frame << ", dsize=" << dsize << ", "
		     << "PACKET_LEN=" << getPacketLen() << ", "
		     << "idx=" << m_idx << ", packet=" << packet_idx << endl;
	}
	char *buffer = m_cam->getBufferPtr(frame);
	bool raw = m_cam->m_save_raw;
	l.unlock();

	FrameDim frame_dim;
	getFrameDim(frame_dim, raw);
	int recv_size = frame_dim.getMemSize();

	if (raw) {
		int xfer_len = sizeof(*p);
		char *dest = buffer + recv_size * m_idx + xfer_len * packet_idx;
		memcpy(dest, dptr, xfer_len);
	} else {
		int xfer_len = sizeof(p->data);
		char *src = p->data;
		int pline = 2 * CHIP_SIZE * frame_dim.getDepth();
		int plines = xfer_len / pline;
		int dline = 2 * pline;
		char *dest = buffer + recv_size * m_idx;
		if (top_half) {
			dest += (CHIP_SIZE - 1) * dline;
			dline *= -1;
		}
		int right_side = !second_half ^ top_half;
		dest += p->pre.idx * dline * plines + right_side * pline;
		for (int i = 0; i < plines; ++i, src += pline, dest += dline)
			memcpy(dest, src, pline);
	}

	l.lock();
	m_packet_map.frameItemFinished(frame, packet_idx);
}

Camera::FrameFinishedCallback::FrameFinishedCallback(Camera *cam)
	 : m_cam(cam)
{
}

void Camera::FrameFinishedCallback::frameFinished(int frame)
{
	m_cam->frameFinished(frame);
}

Camera::Camera(string config_fname) 
	: m_print_policy(PRINT_POLICY_NONE), 
	  m_nb_frames(1), 
	  m_exp_time(0.99),
	  m_frame_period(1.0), 
	  m_started(false), 
	  m_image_type(Bpp16), 
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

Camera::~Camera()
{
	cout << "+++ Starting cleanup ..." << endl;
	stopAcq();
}

void Camera::createReceivers()
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

		AutoPtr<Receiver> recv_obj = new Receiver(this, idx, rx_port, 
							  mode);
		m_recv_list.push_back(recv_obj);
	}

	m_frame_cb = new FrameFinishedCallback(this);
	m_recv_map.setNbItems(recv_port_map.size());
	m_recv_map.setCallback(m_frame_cb);
}

void Camera::putCmd(const string& s)
{
	Args args(s);
	m_cmd->putCommand(args.size(), args);
}

string Camera::getCmd(const string& s)
{
	Args args(s);
	return m_cmd->getCommand(args.size(), args);
}

void Camera::prepareAcq()
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

void Camera::allocBuffers()
{
	FrameDim frame_dim;
	getFrameDim(frame_dim, m_save_raw);
	long buffer_size = frame_dim.getMemSize();
	cout << "+++ Allocating " << m_nb_frames << " buffers "
	     << "(" << buffer_size << ") ..." << endl;
	m_buffer_list = new AutoPtr<MemBuffer>[m_nb_frames];
	for (int i = 0; i < m_nb_frames; ++i)
		m_buffer_list[i] = new MemBuffer(buffer_size);
}

void Camera::startAcq()
{
	cout << "+++ Starting acq ..." << endl;
	putCmd("status start");

	cout << "+++ Querying status ..." << endl;
        cout << "  status=" << getCmd("status") << endl;

	m_started = true;
}

void Camera::stopAcq()
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

void Camera::waitAcq()
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

void Camera::receiverFrameFinished(int frame, Receiver *recv)
{
	m_recv_map.frameItemFinished(frame, recv->m_idx);
}

void Camera::frameFinished(int frame)
{
	if (m_print_policy & PRINT_POLICY_CAMERA) {
		cout << "********* Finished! *******" << endl;
		cout << "frame=" << frame << endl;
	}
}

void Camera::getFrameDim(FrameDim& frame_dim, bool raw)
{
	Receiver *recv = m_recv_list[0];
	recv->getFrameDim(frame_dim, raw);
	frame_dim *= Point(1, getNbHalfModules());
}

