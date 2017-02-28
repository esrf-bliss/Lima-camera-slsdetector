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

using namespace std;
using namespace lima;
using namespace lima::SlsDetector;


Args::Args()
{
	DEB_CONSTRUCTOR();
	update_argc_argv();
}

Args::Args(unsigned int argc, char *argv[])
{
	DEB_CONSTRUCTOR();
	for (unsigned int i = 0; i < argc; ++i)
		m_arg_list.push_back(argv[i]);
	update_argc_argv();
}

Args::Args(const string& s)
{
	DEB_CONSTRUCTOR();
	set(s);
}

Args::Args(const Args& o) : m_arg_list(o.m_arg_list)
{
	DEB_CONSTRUCTOR();
	update_argc_argv();
}

void Args::set(const string& s)
{
	DEB_MEMBER_FUNCT();
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
	DEB_MEMBER_FUNCT();
	m_arg_list.clear();
	update_argc_argv();
}

Args& Args::operator =(const std::string& s)
{
	DEB_MEMBER_FUNCT();
	set(s);
	return *this;
}

string Args::pop_front()
{
	DEB_MEMBER_FUNCT();
	string s = m_arg_list[0];
	erase(0);
	return s;
}

void Args::erase(int pos)
{
	DEB_MEMBER_FUNCT();
	m_arg_list.erase(m_arg_list.begin() + pos);
	update_argc_argv();
}

void Args::update_argc_argv()
{
	DEB_MEMBER_FUNCT();
	m_argc = m_arg_list.size();
	m_argv = new char *[m_argc + 1];
	for (unsigned int i = 0; i < m_argc; ++i)
		m_argv[i] = const_cast<char *>(m_arg_list[i].c_str());
	m_argv[m_argc] = NULL;
}


Camera::AppInputData::AppInputData(string cfg_fname) 
	: config_file_name(cfg_fname) 
{
	DEB_CONSTRUCTOR();
	parseConfigFile();
}

void Camera::AppInputData::parseConfigFile()
{
	DEB_MEMBER_FUNCT();

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
			if (id < 0)
				THROW_HW_FATAL(InvalidValue) << 
					"Invalid detector id: " << id;
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
	DEB_CONSTRUCTOR();
}

Camera::FrameMap::Callback::~Callback()
{
	DEB_DESTRUCTOR();
	if (m_map)
		m_map->m_cb = NULL;
}

Camera::FrameMap::FrameMap()
	: m_nb_items(0), m_last_seq_finished_frame(-1), m_cb(NULL)
{
	DEB_CONSTRUCTOR();
}

Camera::FrameMap::~FrameMap()
{
	DEB_DESTRUCTOR();
	if (m_cb)
		m_cb->m_map = NULL;
}

void Camera::FrameMap::setNbItems(int nb_items)
{
	DEB_MEMBER_FUNCT();
	m_nb_items = nb_items;
}

void Camera::FrameMap::clear()
{
	DEB_MEMBER_FUNCT();
	m_map.clear();
	m_non_seq_finished_frames.clear();
	m_last_seq_finished_frame = -1;
}

void Camera::FrameMap::setCallback(Callback *cb)
{ 
	DEB_MEMBER_FUNCT();
	if (m_cb)
		m_cb->m_map = NULL;
	m_cb = cb; 
	if (m_cb)
		m_cb->m_map = this;
}

void Camera::FrameMap::frameItemFinished(int frame, int item)
{
	DEB_MEMBER_FUNCT();

	if (m_nb_items == 0)		
		THROW_HW_ERROR(InvalidValue) << "No items defined";
	else if ((item < 0) || (item >= m_nb_items))
		THROW_HW_ERROR(InvalidValue) << DEB_VAR2(item, m_nb_items);
	Map::iterator mit = m_map.find(frame);
	if (mit == m_map.end()) {
		for (int i = 0; i < m_nb_items; ++i)
			if (i != item)
				m_map[frame].insert(i);
		return;
	}

	List& item_list = mit->second;
	List::iterator lit = item_list.find(item);
	if (lit == item_list.end())
		THROW_HW_ERROR(Error) << "item " << item << " already finished "
				      << "for frame " << frame;

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

Camera::Receiver::FrameFinishedCallback::FrameFinishedCallback(Receiver *r) 
	: m_recv(r)
{
	DEB_CONSTRUCTOR();
}

void Camera::Receiver::FrameFinishedCallback::frameFinished(int frame) 
{
	DEB_MEMBER_FUNCT();
	DEB_RECV_FRAME() << DEB_VAR2(frame, m_recv->m_idx); 
	m_recv->m_cam->receiverFrameFinished(frame, m_recv);
}

Camera::Receiver::Receiver(Camera *cam, int idx, int rx_port, int mode)
	: m_cam(cam), m_idx(idx), m_rx_port(rx_port), m_mode(mode)
{
	DEB_CONSTRUCTOR();

	m_cb = new FrameFinishedCallback(this);

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
	DEB_DESTRUCTOR();
	m_recv->stop();
}

void Camera::Receiver::getFrameDim(FrameDim& frame_dim, bool raw)
{
	DEB_MEMBER_FUNCT();
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
	DEB_MEMBER_FUNCT();
	FrameDim frame_dim;
	getFrameDim(frame_dim, false);
	return frame_dim.getMemSize() / sizeof(Packet::data); 
}

void Camera::Receiver::start()
{	
	DEB_MEMBER_FUNCT();
	int init_ret;
	m_recv = new slsReceiverUsers(m_args.size(), m_args, init_ret);
	if (init_ret == slsReceiverDefs::FAIL)
		THROW_HW_ERROR(Error) << "Error creating slsReceiver";
	if (m_recv->start() == slsReceiverDefs::FAIL) 
		THROW_HW_ERROR(Error) << "Error starting slsReceiver";
}

void Camera::Receiver::prepareAcq()
{
	DEB_MEMBER_FUNCT();
	m_packet_map.clear();
}

int Camera::Receiver::startCallback(char *fpath, char *fname, 
				    int fidx, int dsize, void *priv)
{
	DEB_STATIC_FUNCT();
	Receiver *recv = static_cast<Receiver *>(priv);
	return recv->startCallback(fpath, fname, fidx, dsize);
}

void Camera::Receiver::frameCallback(int frame, char *dptr, int dsize, FILE *f, 
				     char *guidptr, void *priv)
{
	DEB_STATIC_FUNCT();
	Receiver *recv = static_cast<Receiver *>(priv);
	recv->frameCallback(frame, dptr, dsize, f, guidptr);
}

int Camera::Receiver::startCallback(char *fpath, char *fname, 
				    int fidx, int dsize)
{
	DEB_MEMBER_FUNCT();
	DEB_CAMERA_START() << DEB_VAR5(fpath, fname, fidx, dsize, m_idx);

	if (dsize != getPacketLen())
		DEB_WARNING() << "!!!! Warning !!!! " 
			      << DEB_VAR2(dsize, getPacketLen());

	return DO_NOTHING;
}

void Camera::Receiver::frameCallback(int frame, char *dptr, int dsize, FILE *f, 
				     char *guidptr)
{
	DEB_MEMBER_FUNCT();

	Packet *p = static_cast<Packet *>(static_cast<void *>(dptr));
	bool second_half = p->pre.flags & 0x20;
	int packet_idx = p->pre.idx;
	if (second_half)
		packet_idx += getFramePackets() / 2;
	bool top_half = (m_idx % 2 == 0);

	DEB_RECV_PACKET() << DEB_VAR5(frame, dsize, getPacketLen(), m_idx, 
				      packet_idx);

	bool raw;
	char *buffer;
	{
		AutoMutex l = m_cam->lock();
		buffer = m_cam->getFrameBufferPtr(frame);
		raw = m_cam->m_save_raw;
	}

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
		int plw = 2 * CHIP_SIZE * frame_dim.getDepth();
		int plnb = xfer_len / plw;
		int dlw = 2 * plw;
		char *dest = buffer + recv_size * m_idx;
		if (top_half) {
			dest += (CHIP_SIZE - 1) * dlw;
			dlw *= -1;
		}
		int right_side = !second_half ^ top_half;
		dest += p->pre.idx * dlw * plnb + right_side * plw;
		for (int i = 0; i < plnb; ++i, src += plw, dest += dlw)
			memcpy(dest, src, plw);
	}

	{
		AutoMutex l = m_cam->lock();
		m_packet_map.frameItemFinished(frame, packet_idx);
	}
}

Camera::FrameFinishedCallback::FrameFinishedCallback(Camera *cam)
	 : m_cam(cam)
{
	DEB_CONSTRUCTOR();
}

void Camera::FrameFinishedCallback::frameFinished(int frame)
{
	DEB_MEMBER_FUNCT();
	m_cam->frameFinished(frame);
}

Camera::AcqThread::AcqThread(Camera *cam)
	: m_cam(cam), m_cond(m_cam->m_cond), m_state(m_cam->m_state),
	  m_frame_queue(m_cam->m_frame_queue)
{
	DEB_CONSTRUCTOR();
	m_state = Starting;
	start();
	while (m_state != Running)
		m_cond.wait();
}

void Camera::AcqThread::stop(bool wait)
{
	DEB_MEMBER_FUNCT();
	m_state = StopReq;
	m_cond.broadcast();
	while (wait && (m_state != Stopped))
		m_cond.wait();
}

void Camera::AcqThread::threadFunction()
{
	DEB_MEMBER_FUNCT();

	AutoMutex l = m_cam->lock();
	{
		AutoMutexUnlock u(l);
		m_cam->putCmd("receiver start");
		m_cam->putCmd("status start");
	}
	m_state = Running;
	m_cond.broadcast();

	do {
		while ((m_state != StopReq) && m_frame_queue.empty())
			m_cond.wait();
		if (!m_frame_queue.empty()) {
			int frame = m_frame_queue.front();
			m_frame_queue.pop();
			bool cont_acq;
			{
				AutoMutexUnlock u(l);
				DEB_TRACE() << DEB_VAR1(frame);
				cont_acq = newFrameReady(frame);
			}
			if (!cont_acq)
				m_state = StopReq;
		}
	} while (m_state != StopReq);

	m_state = Stopping;
	{
		AutoMutexUnlock u(l);
		m_cam->putCmd("status stop");
		m_cam->putCmd("receiver stop");
	}
	m_state = Stopped;
	m_cond.broadcast();
}

bool Camera::AcqThread::newFrameReady(int frame)
{
	DEB_MEMBER_FUNCT();

	HwFrameInfoType frame_info;
	frame_info.acq_frame_nb = frame;
	bool cont_acq = m_cam->m_buffer_cb_mgr->newFrameReady(frame_info);
	return cont_acq && (frame < m_cam->m_nb_frames - 1);
}

Camera::Camera(string config_fname) 
	: m_image_type(Bpp16), 
	  m_save_raw(false),
	  m_state(Idle)
{
	DEB_CONSTRUCTOR();

	m_input_data = new AppInputData(config_fname);

	removeSharedMem();
	createReceivers();

	DEB_TRACE() << "Creating the multiSlsDetector object";
	m_det = new multiSlsDetector(0);
	DEB_TRACE() << "Reading configuration file";
	const char *fname = m_input_data->config_file_name.c_str();
	m_det->readConfigurationFile(fname);

	DEB_TRACE() << "Creating the multiSlsDetectorCommand";
	m_cmd = new multiSlsDetectorCommand(m_det);

	setNbFrames(1);
	setExpTime(0.99);
	setFramePeriod(1.0);
}

Camera::~Camera()
{
	DEB_DESTRUCTOR();
	stopAcq();
}

char *Camera::getFrameBufferPtr(int frame_nb)
{
	DEB_MEMBER_FUNCT();

	StdBufferCbMgr *cb_mgr = m_buffer_cb_mgr;
	if (!cb_mgr)
		THROW_HW_ERROR(InvalidValue) << "No BufferCbMgr defined";
	int nb_buffers, concat_frames;
	cb_mgr->getNbBuffers(nb_buffers);
	cb_mgr->getNbConcatFrames(concat_frames);
	int buffer_nb = frame_nb / concat_frames % nb_buffers;
	void *ptr = cb_mgr->getBufferPtr(buffer_nb, frame_nb % concat_frames);
	return static_cast<char *>(ptr);
}

void Camera::removeSharedMem()
{
	DEB_MEMBER_FUNCT();
	const char *cmd = "ipcs -m | "
		"grep -E '^0x000016[0-9a-z]{2}' | "
		"awk '{print $2}' | while read m; do ipcrm -m $m; done";
	system(cmd);
}

void Camera::createReceivers()
{
	DEB_MEMBER_FUNCT();

	DEB_TRACE() << "Receivers:";
	const RecvPortMap& recv_port_map = m_input_data->recv_port_map;
	RecvPortMap::const_iterator mit, mend = recv_port_map.end();
	int idx = 0;
	for (mit = recv_port_map.begin(); mit != mend; ++mit, ++idx) {
		unsigned int id = mit->first;
		if (id >= m_input_data->host_name_list.size())
			THROW_HW_FATAL(InvalidValue) << DEB_VAR1(id) 
						     << "too high";
		const string& host_name = m_input_data->host_name_list[id];
		int rx_port = mit->second;
		int mode = (id % 2);
		DEB_TRACE() << "  " << host_name << ": " 
			    << DEB_VAR2(rx_port, mode);

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
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << "s=\"" << s << "\"";
	Args args(s);
	AutoMutex l(m_cmd_mutex);
	m_cmd->putCommand(args.size(), args);
}

string Camera::getCmd(const string& s)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << "s=\"" << s << "\"";
	Args args(s);
	AutoMutex l(m_cmd_mutex);
	string r = m_cmd->getCommand(args.size(), args);
	DEB_RETURN() << "r=\"" << r << "\"";
	return r;
}

void Camera::setNbFrames(int nb_frames)
{
	DEB_MEMBER_FUNCT();
	putCmd("timing auto");
	ostringstream os;
	os << "frames " << nb_frames;
	putCmd(os.str());
	m_nb_frames = nb_frames;
}

void Camera::getNbFrames(int& nb_frames)
{
	DEB_MEMBER_FUNCT();
	nb_frames = m_nb_frames;
}

void Camera::setExpTime(double exp_time)
{
	DEB_MEMBER_FUNCT();
	ostringstream os;
	os << "exptime " << exp_time;
	putCmd(os.str());
	m_exp_time = exp_time;
}

void Camera::getExpTime(double& exp_time)
{ 
	DEB_MEMBER_FUNCT();
	exp_time = m_exp_time;
}

void Camera::setFramePeriod(double frame_period)
{
	DEB_MEMBER_FUNCT();
	ostringstream os;
	os << "period " << frame_period;
	putCmd(os.str());
	m_frame_period = frame_period;
}

void Camera::getFramePeriod(double& frame_period)
{
	DEB_MEMBER_FUNCT();
	frame_period = m_frame_period;
}

Camera::State Camera::getState()
{
	DEB_MEMBER_FUNCT();
	AutoMutex l = lock();
	return getEffectiveState();
}

Camera::State Camera::getEffectiveState()
{
	if (m_state == Stopped) {
		m_acq_thread = NULL;
		m_state = Idle;
	}
	return m_state;
}

void Camera::waitState(State state)
{
	DEB_MEMBER_FUNCT();
	AutoMutex l = lock();
	while (getEffectiveState() != state)
		m_cond.wait();
}

Camera::State Camera::waitNotState(State state)
{
	DEB_MEMBER_FUNCT();
	AutoMutex l = lock();
	while (getEffectiveState() == state)
		m_cond.wait();
	return getEffectiveState();
}

void Camera::prepareAcq()
{
	DEB_MEMBER_FUNCT();

	if (!m_buffer_cb_mgr)
		THROW_HW_ERROR(Error) << "No BufferCbMgr defined";

	waitNotState(Stopping);
	if (getState() != Idle)
		THROW_HW_ERROR(Error) << "Camera is not idle";

	{
		AutoMutex l = lock();
		RecvList::iterator it, end = m_recv_list.end();
		for (it = m_recv_list.begin(); it != end; ++it)
			(*it)->prepareAcq();
		m_recv_map.clear();
		DEB_ALWAYS() << DEB_VAR1(m_frame_queue.size());
		while (!m_frame_queue.empty())
			m_frame_queue.pop();
	}

	putCmd("resetframescaught");
}

void Camera::startAcq()
{
	DEB_MEMBER_FUNCT();

	AutoMutex l = lock();
	if (m_acq_thread)
		THROW_HW_ERROR(Error) << "Must call prepareAcq first";

	m_buffer_cb_mgr->setStartTimestamp(Timestamp::now());

	m_acq_thread = new AcqThread(this);
}

void Camera::stopAcq()
{
	DEB_MEMBER_FUNCT();

	AutoMutex l = lock();
	if (getEffectiveState() != Running)
		return;

	m_acq_thread->stop(true);
	if (getEffectiveState() != Idle)
		THROW_HW_ERROR(Error) << "Camera not Idle";
}

void Camera::receiverFrameFinished(int frame, Receiver *recv)
{
	DEB_MEMBER_FUNCT();
	m_recv_map.frameItemFinished(frame, recv->m_idx);
}

void Camera::frameFinished(int frame)
{
	DEB_MEMBER_FUNCT();
	DEB_CAMERA_FRAME() << DEB_VAR1(frame);

	m_frame_queue.push(frame);
	m_cond.broadcast();
}

void Camera::getFrameDim(FrameDim& frame_dim, bool raw)
{
	DEB_MEMBER_FUNCT();
	Receiver *recv = m_recv_list[0];
	recv->getFrameDim(frame_dim, raw);
	frame_dim *= Point(1, getNbHalfModules());
	DEB_RETURN() << DEB_VAR1(frame_dim);
}

int Camera::getFramesCaught()
{
	DEB_MEMBER_FUNCT();
	int frames_caught;
	string ans = getCmd("framescaught");
	istringstream is(ans);
	is >> frames_caught;
	return frames_caught;
}

string Camera::getStatus()
{
	DEB_MEMBER_FUNCT();
	return getCmd("status");
}
