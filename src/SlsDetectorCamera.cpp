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

#include "multiSlsDetectorCommand.h"

using namespace std;
using namespace lima;
using namespace lima::SlsDetector;


Camera::Model::Model(Camera *cam, Type type)
	: m_cam(cam), m_type(type)
{
	DEB_CONSTRUCTOR();
	DEB_PARAM() << DEB_VAR1(type);
}

Camera::Model::~Model()
{
	DEB_DESTRUCTOR();

	if (m_cam)
		m_cam->setModel(NULL);
}

void Camera::Model::updateCameraModel()
{
	DEB_MEMBER_FUNCT();
	m_cam->setModel(this);	
}

void Camera::Model::putCmd(const string& s, int idx)
{
	DEB_MEMBER_FUNCT();
	m_cam->putCmd(s, idx);
}

string Camera::Model::getCmd(const string& s, int idx)
{
	DEB_MEMBER_FUNCT();
	return m_cam->getCmd(s, idx);
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

Camera::FrameMap::FrameMap(bool debug)
	: m_nb_items(0), m_last_seq_finished_frame(-1), m_cb(NULL),
	  m_debug(debug)
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

void Camera::FrameMap::checkFinishedFrameItem(FrameType frame, int item)
{
	DEB_MEMBER_FUNCT();

	if (m_nb_items == 0)		
		THROW_HW_ERROR(InvalidValue) << "No items defined";
	else if ((item < 0) || (item >= m_nb_items))
		THROW_HW_ERROR(InvalidValue) << DEB_VAR2(item, m_nb_items);

	FrameType &last = m_last_seq_finished_frame;
	List &waiting = m_non_seq_finished_frames;
	bool frame_finished = (isValidFrame(last) && (frame <= last));
	bool frame_waiting = (waiting.find(frame) != waiting.end());
	if (frame_finished || frame_waiting)
		THROW_HW_ERROR(Error) << DEB_VAR1(frame) << " finished already";

	Map::iterator mit = m_map.find(frame);
	if ((mit == m_map.end()) || !m_debug)
		return;

	List& item_list = mit->second;
	List::iterator lit = item_list.find(item);
	if (lit == item_list.end())
		THROW_HW_ERROR(Error) << "item " << item << " already "
				      << "finished for frame " << frame;
}

void Camera::FrameMap::frameItemFinished(FrameType frame, int item, 
					 bool no_check)
{
	DEB_MEMBER_FUNCT();

	if (!no_check)
		checkFinishedFrameItem(frame, item);

	Map::iterator mit = m_map.find(frame);
	if (mit == m_map.end()) {
		if (m_debug) {
			for (int i = 0; i < m_nb_items; ++i)
				if (i != item)
					m_map[frame].insert(i);
		} else {
			m_map[frame].insert(1);
		}
		return;
	}

	List& item_list = mit->second;
	bool last_item;
	if (m_debug) {
		List::iterator lit = item_list.find(item);
		item_list.erase(lit);
		last_item = item_list.empty();
	} else {
		List::iterator lit = item_list.begin();
		int nb_items = *lit + 1;
		item_list.erase(lit);
		item_list.insert(nb_items);
		last_item = (nb_items == m_nb_items);
	}
	if (!last_item)
		return
	m_map.erase(mit);

	FrameType &last = m_last_seq_finished_frame;
	List &waiting = m_non_seq_finished_frames;
	if (frame == last + 1) {
		int next = ++last + 1;
		List::iterator lit;
		while (!waiting.empty() && (*(lit = waiting.begin()) == next)) {
			waiting.erase(lit);
			last = next++;
		}
	} else {
		waiting.insert(frame);
	}
	if (m_cb)
		m_cb->frameFinished(frame);
}

ostream& lima::SlsDetector::operator <<(ostream& os, Camera::State state)
{
	const char *name = "Unknown";
	switch (state) {
	case Camera::Idle:	name = "Idle";		break;
	case Camera::Init:	name = "Init";		break;
	case Camera::Starting:	name = "Starting";	break;
	case Camera::Running:	name = "Running";	break;
	case Camera::StopReq:	name = "StopReq";	break;
	case Camera::Stopping:	name = "Stopping";	break;
	case Camera::Stopped:	name = "Stopped";	break;
	}
	return os << name;
}

ostream& lima::SlsDetector::operator <<(ostream& os, Camera::Type type)
{
	const char *name = "Invalid";
	switch (type) {
	case Camera::UnknownDet:	name = "Unknown";	break;
	case Camera::GenericDet:	name = "Generic";	break;
	case Camera::EigerDet:		name = "Eiger";		break;
	case Camera::JungfrauDet:	name = "Jungfrau";	break;
	}
	return os << name;
}

ostream& lima::SlsDetector::operator <<(ostream& os, 
					const Camera::FrameMap& m)
{
	os << "<";
	long last_seq_finished_frame = m.getLastSeqFinishedFrame();
	os << "LastSeqFinishedFrame=" << last_seq_finished_frame << ", "
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

void Camera::Receiver::FrameFinishedCallback::frameFinished(FrameType frame) 
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR2(frame, m_recv->m_idx); 
	m_recv->m_cam->receiverFrameFinished(frame, m_recv);
}

Camera::Receiver::Receiver(Camera *cam, int idx, int rx_port)
	: m_cam(cam), m_idx(idx), m_rx_port(rx_port)
{
	DEB_CONSTRUCTOR();

	m_cb = new FrameFinishedCallback(this);

	m_port_map.setCallback(m_cb);

	ostringstream os;
	os << "slsReceiver"
	   << " --rx_tcpport " << m_rx_port;
	m_args.set(os.str());

	start();

	m_recv->registerCallBackStartAcquisition(fileStartCallback, this);
	m_recv->registerCallBackRawDataReady(portCallback, this);
}

Camera::Receiver::~Receiver()
{
	DEB_DESTRUCTOR();
	m_recv->stop();
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
	int recv_ports = m_cam->m_model->getRecvPorts();
	m_port_map.setNbItems(recv_ports);
	m_port_map.clear();
}

int Camera::Receiver::fileStartCallback(char *fpath, char *fname, 
					uint64_t fidx, uint32_t dsize, 
					void *priv)
{
	DEB_STATIC_FUNCT();
	Receiver *recv = static_cast<Receiver *>(priv);
	return recv->fileStartCallback(fpath, fname, fidx, dsize);
}

void Camera::Receiver::portCallback(FrameType frame, 
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
				    void *priv)
{
	DEB_STATIC_FUNCT();
	Receiver *recv = static_cast<Receiver *>(priv);
	int port = (x % 2);
	FrameType lima_frame = frame - 1;
	DEB_PARAM() << DEB_VAR2(frame, lima_frame);
	recv->portCallback(lima_frame, port, dptr, dsize);
}

int Camera::Receiver::fileStartCallback(char *fpath, char *fname, 
					uint64_t fidx, uint32_t dsize)
{
	DEB_MEMBER_FUNCT();
	Model *model = m_cam->m_model;
	if (model)
		model->processRecvFileStart(m_idx, dsize);
	return 0;
}

void Camera::Receiver::portCallback(FrameType frame, int port, char *dptr, 
				    uint32_t dsize)
{
	DEB_MEMBER_FUNCT();

	Model *model = m_cam->m_model;
	if (!model || (m_cam->getState() == Stopping))
		return;

	char *bptr = m_cam->getFrameBufferPtr(frame);
	Mutex& lock = m_cam->m_cond.mutex();
	try {
		AutoMutex l(lock);
		m_port_map.checkFinishedFrameItem(frame, port);
		{
			AutoMutexUnlock u(l);
			model->processRecvPort(m_idx, frame, port, dptr, dsize,
					       lock, bptr);
		}
		m_port_map.frameItemFinished(frame, port, true);
	} catch (Exception& e) {
		ostringstream err_msg;
		err_msg << "Receiver::frameCallback: " << e;
		Event::Code err_code = Event::CamOverrun;
		Event *event = new Event(Hardware, Event::Error, Event::Camera, 
					 err_code, err_msg.str());
		DEB_EVENT(*event) << DEB_VAR1(*event);
		m_cam->reportEvent(event);
	}
}

Camera::FrameFinishedCallback::FrameFinishedCallback(Camera *cam)
	 : m_cam(cam)
{
	DEB_CONSTRUCTOR();
}

void Camera::FrameFinishedCallback::frameFinished(FrameType frame)
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

	multiSlsDetector *det = m_cam->m_det;

	AutoMutex l = m_cam->lock();
	{
		AutoMutexUnlock u(l);
		DEB_TRACE() << "calling startReceiver";
		det->startReceiver();
		DEB_TRACE() << "calling startAcquisition";
		det->startAcquisition();
	}
	m_state = Running;
	m_cond.broadcast();

	do {
		while ((m_state != StopReq) && m_frame_queue.empty())
			m_cond.wait();
		if (!m_frame_queue.empty()) {
			FrameType frame = m_frame_queue.front();
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
		DEB_TRACE() << "calling stopAcquisition";
		det->stopAcquisition();
		DEB_TRACE() << "calling stopReceiver";
		det->stopReceiver();
	}
	m_state = Stopped;
	m_cond.broadcast();
}

bool Camera::AcqThread::newFrameReady(FrameType frame)
{
	DEB_MEMBER_FUNCT();
	HwFrameInfoType frame_info;
	frame_info.acq_frame_nb = frame;
	bool cont_acq = m_cam->m_buffer_cb_mgr->newFrameReady(frame_info);
	return cont_acq && (frame < m_cam->m_nb_frames - 1);
}

Camera::Camera(string config_fname) 
	: m_model(NULL),
	  m_pixel_depth(PixelDepth16), 
	  m_image_type(Bpp16), 
	  m_raw_mode(false),
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

	m_pixel_depth = PixelDepth(m_det->setDynamicRange(-1));
	if (m_pixel_depth == PixelDepth4)
		m_pixel_depth = PixelDepth8;

	setSettings(Defs::Standard);
	setTrigMode(Defs::Auto);
	setNbFrames(1);
	setExpTime(0.99);
	setFramePeriod(1.0);
}

Camera::~Camera()
{
	DEB_DESTRUCTOR();

	if (!m_model)
		return;

	stopAcq();
	m_model->m_cam = NULL;
}

Camera::Type Camera::getType()
{
	DEB_MEMBER_FUNCT();
	string type_resp = getCmd("type");
	ostringstream os;
	os << "(([^+]+)\\+){" << getNbDetModules() << "}";
	DEB_TRACE() << DEB_VAR1(os.str());
	RegEx re(os.str());
	FullMatch full_match;
	if (!re.match(type_resp, full_match))
		THROW_HW_ERROR(Error) << "Invalid type response: " << type_resp;
	string type_str = full_match[2];
	Type det_type = UnknownDet;
	if (type_str == "Generic") {
		det_type = GenericDet;
	} else if (type_str == "Eiger") {
		det_type = EigerDet;
	} else if (type_str == "Jungfrau") {
		det_type = JungfrauDet;
	}
	DEB_RETURN() << DEB_VAR1(det_type);
	return det_type;
}

void Camera::setModel(Model *model)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR2(model, m_model);
	
	if (model && (model->getType() != getType()))
		THROW_HW_ERROR(InvalidValue) << DEB_VAR2(getType(), 
							 model->getType());
	if (m_model == model)
		return;
	if (m_model)
		m_model->m_cam = NULL;
	m_model = model;
	if (!m_model)
		return;

	setPixelDepth(m_pixel_depth);
	setSettings(m_settings);
}

char *Camera::getFrameBufferPtr(FrameType frame_nb)
{
	DEB_MEMBER_FUNCT();

	StdBufferCbMgr *cb_mgr = m_buffer_cb_mgr;
	if (!cb_mgr)
		THROW_HW_ERROR(InvalidValue) << "No BufferCbMgr defined";
	void *ptr = cb_mgr->getFrameBufferPtr(frame_nb);
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
		DEB_TRACE() << "  " << host_name << ": " << DEB_VAR1(rx_port);

		AutoPtr<Receiver> recv_obj = new Receiver(this, idx, rx_port);
		m_recv_list.push_back(recv_obj);
	}

	m_frame_cb = new FrameFinishedCallback(this);
	m_recv_map.setNbItems(recv_port_map.size());
	m_recv_map.setCallback(m_frame_cb);
}

void Camera::putCmd(const string& s, int idx)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << "s=\"" << s << "\"";
	Args args(s);
	multiSlsDetectorCommand cmd(m_det);
	cmd.putCommand(args.size(), args, idx);
}

string Camera::getCmd(const string& s, int idx)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << "s=\"" << s << "\"";
	Args args(s);
	multiSlsDetectorCommand cmd(m_det);
	string r = cmd.getCommand(args.size(), args, idx);
	DEB_RETURN() << "r=\"" << r << "\"";
	return r;
}

void Camera::setTrigMode(TrigMode trig_mode)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR1(trig_mode);
	typedef slsDetectorDefs::externalCommunicationMode ExtComMode;
	ExtComMode mode = static_cast<ExtComMode>(trig_mode);
	m_det->setExternalCommunicationMode(mode);
	m_trig_mode = trig_mode;
	setNbFrames(m_nb_frames);
}

void Camera::getTrigMode(TrigMode& trig_mode)
{
	DEB_MEMBER_FUNCT();
	trig_mode = m_trig_mode;
	DEB_RETURN() << DEB_VAR1(trig_mode);
}

void Camera::setNbFrames(FrameType nb_frames)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR1(nb_frames);
	bool trig_exp = (m_trig_mode == Defs::TriggerExposure);
	int cam_frames = trig_exp ? 1 : nb_frames;
	int cam_triggers = trig_exp ? nb_frames : 1;
	m_det->setNumberOfFrames(cam_frames);
	m_det->setNumberOfCycles(cam_triggers);
	m_nb_frames = nb_frames;
}

void Camera::getNbFrames(FrameType& nb_frames)
{
	DEB_MEMBER_FUNCT();
	nb_frames = m_nb_frames;
	DEB_RETURN() << DEB_VAR1(nb_frames);
}

void Camera::setExpTime(double exp_time)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR1(exp_time);
	m_det->setExposureTime(NSec(exp_time));
	m_exp_time = exp_time;
}

void Camera::getExpTime(double& exp_time)
{ 
	DEB_MEMBER_FUNCT();
	exp_time = m_exp_time;
	DEB_RETURN() << DEB_VAR1(exp_time);
}

void Camera::setFramePeriod(double frame_period)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR1(frame_period);
	m_det->setExposurePeriod(NSec(frame_period));
	m_frame_period = frame_period;
}

void Camera::getFramePeriod(double& frame_period)
{
	DEB_MEMBER_FUNCT();
	frame_period = m_frame_period;
	DEB_RETURN() << DEB_VAR1(frame_period);
}

void Camera::updateImageSize()
{
	DEB_MEMBER_FUNCT();
	m_model->updateImageSize();
	FrameDim frame_dim;
	getFrameDim(frame_dim, m_raw_mode);
	DEB_TRACE() << "MaxImageSizeChanged: " << DEB_VAR1(frame_dim);
	maxImageSizeChanged(frame_dim.getSize(), frame_dim.getImageType());
}

void Camera::setPixelDepth(PixelDepth pixel_depth)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR1(pixel_depth);

	if (getState() != Idle)
		THROW_HW_FATAL(Error) << "Camera is not idle";

	switch (pixel_depth) {
	case PixelDepth4:
		THROW_HW_ERROR(NotSupported) << "PixelDepth4 not supported yet";
	case PixelDepth8:
		m_image_type = Bpp8;	break;
	case PixelDepth16:
		m_image_type = Bpp16;	break;
	case PixelDepth32:
		m_image_type = Bpp32;	break;
	default:
		THROW_HW_ERROR(InvalidValue) << DEB_VAR1(pixel_depth);
	}
	m_det->setDynamicRange(pixel_depth);
	m_pixel_depth = pixel_depth;

	if (m_model)
		updateImageSize();
}

void Camera::getPixelDepth(PixelDepth& pixel_depth)
{
	DEB_MEMBER_FUNCT();
	pixel_depth = m_pixel_depth; 
	DEB_RETURN() << DEB_VAR1(pixel_depth);
}

void Camera::setRawMode(bool raw_mode)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR1(raw_mode);

	if (raw_mode == m_raw_mode)
		return;
	m_raw_mode = raw_mode;

	updateImageSize();
}

void Camera::getRawMode(bool& raw_mode)
{
	DEB_MEMBER_FUNCT();
	raw_mode = m_raw_mode; 
	DEB_RETURN() << DEB_VAR1(raw_mode);
}

Camera::State Camera::getState()
{
	DEB_MEMBER_FUNCT();
	AutoMutex l = lock();
	State state = getEffectiveState();
	DEB_RETURN() << DEB_VAR1(state);
	return state;
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
	DEB_PARAM() << DEB_VAR1(state);
	AutoMutex l = lock();
	while (getEffectiveState() != state)
		m_cond.wait();
}

Camera::State Camera::waitNotState(State state)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR1(state);
	AutoMutex l = lock();
	while (getEffectiveState() == state)
		m_cond.wait();
	state = getEffectiveState();
	DEB_RETURN() << DEB_VAR1(state);
	return state;
}

void Camera::prepareAcq()
{
	DEB_MEMBER_FUNCT();

	if (!m_buffer_cb_mgr)
		THROW_HW_ERROR(Error) << "No BufferCbMgr defined";
	if (!m_model)
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
		DEB_TRACE() << DEB_VAR1(m_frame_queue.size());
		while (!m_frame_queue.empty())
			m_frame_queue.pop();
	}

	m_model->prepareAcq();

	// recv->resetAcquisitionCount()
	m_det->resetFramesCaught();
	m_det->enableWriteToFile(0);
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

void Camera::receiverFrameFinished(FrameType frame, Receiver *recv)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR2(frame, recv->m_idx);
	m_recv_map.frameItemFinished(frame, recv->m_idx);
}

void Camera::frameFinished(FrameType frame)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR1(frame);

	m_frame_queue.push(frame);
	m_cond.broadcast();
}

int Camera::getFramesCaught()
{
	DEB_MEMBER_FUNCT();
	// recv->getTotalFramesCaught()
	int frames_caught = m_det->getFramesCaughtByReceiver();
	DEB_RETURN() << DEB_VAR1(frames_caught);
	return frames_caught;
}

string Camera::getStatus()
{
	DEB_MEMBER_FUNCT();
	return getCmd("status");
}

void Camera::setDAC(int sub_mod_idx, DACIndex dac_idx, int val, bool milli_volt)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR4(sub_mod_idx, dac_idx, val, milli_volt);

	if ((sub_mod_idx < -1) || (sub_mod_idx >= getNbDetSubModules()))
		THROW_HW_ERROR(InvalidValue) << DEB_VAR1(sub_mod_idx);

	typedef slsDetectorDefs::dacIndex SlsDACIndex;
	SlsDACIndex idx = static_cast<SlsDACIndex>(dac_idx);
	dacs_t ret = m_det->setDAC(val, idx, milli_volt, sub_mod_idx);
	if (ret == MultiSlsDetectorErr)
		THROW_HW_ERROR(Error) << "Error setting DAC " << dac_idx 
				      << " on (sub)module " << sub_mod_idx;
}

void Camera::getDAC(int sub_mod_idx, DACIndex dac_idx, int& val, bool milli_volt)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR3(sub_mod_idx, dac_idx, milli_volt);

	if ((sub_mod_idx < 0) || (sub_mod_idx >= getNbDetSubModules()))
		THROW_HW_ERROR(InvalidValue) << DEB_VAR1(sub_mod_idx);

	typedef slsDetectorDefs::dacIndex SlsDACIndex;
	SlsDACIndex idx = static_cast<SlsDACIndex>(dac_idx);
	dacs_t ret = m_det->setDAC(-1, idx, milli_volt, sub_mod_idx);
	if (ret == MultiSlsDetectorErr)
		THROW_HW_ERROR(Error) << "Error getting DAC " << dac_idx 
				      << " on (sub)module " << sub_mod_idx;
	val = ret;
	DEB_RETURN() << DEB_VAR1(val);
}

void Camera::getDACList(DACIndex dac_idx, IntList& val_list, bool milli_volt)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR2(dac_idx, milli_volt);

	int nb_sub_modules = getNbDetSubModules();
	val_list.resize(nb_sub_modules);
	for (int i = 0; i < nb_sub_modules; ++i)
		getDAC(i, dac_idx, val_list[i], milli_volt);
}

void Camera::getADC(int sub_mod_idx, ADCIndex adc_idx, int& val)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR2(sub_mod_idx, adc_idx);

	if ((sub_mod_idx < 0) || (sub_mod_idx >= getNbDetSubModules()))
		THROW_HW_ERROR(InvalidValue) << DEB_VAR1(sub_mod_idx);

	typedef slsDetectorDefs::dacIndex SlsDACIndex;
	SlsDACIndex idx = static_cast<SlsDACIndex>(adc_idx);
	dacs_t ret = m_det->getADC(idx, sub_mod_idx);
	if (ret == MultiSlsDetectorErr)
		THROW_HW_ERROR(Error) << "Error getting ADC " << adc_idx 
				      << " on (sub)module " << sub_mod_idx;
	val = ret;
	DEB_RETURN() << DEB_VAR1(val);
}

void Camera::getADCList(ADCIndex adc_idx, IntList& val_list)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR1(adc_idx);

	int nb_sub_modules = getNbDetSubModules();
	val_list.resize(nb_sub_modules);
	for (int i = 0; i < nb_sub_modules; ++i)
		getADC(i, adc_idx, val_list[i]);
}

void Camera::setAllTrimBits(int sub_mod_idx, int val)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR2(sub_mod_idx, val);

	if ((sub_mod_idx < -1) || (sub_mod_idx >= getNbDetSubModules()))
		THROW_HW_ERROR(InvalidValue) << DEB_VAR1(sub_mod_idx);

	int ret = m_det->setAllTrimbits(val, sub_mod_idx);
	if (ret == MultiSlsDetectorErr)
		THROW_HW_ERROR(Error) << "Error setting all trim bits"
				      << " on (sub)module " << sub_mod_idx;
}

void Camera::getAllTrimBits(int sub_mod_idx, int& val)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR1(sub_mod_idx);

	if ((sub_mod_idx < 0) || (sub_mod_idx >= getNbDetSubModules()))
		THROW_HW_ERROR(InvalidValue) << DEB_VAR1(sub_mod_idx);

	int ret = m_det->setAllTrimbits(-1, sub_mod_idx);
	if (ret == MultiSlsDetectorErr)
		THROW_HW_ERROR(Error) << "Error getting all trim bits"
				      << " on (sub)module " << sub_mod_idx;
	val = ret;
	DEB_RETURN() << DEB_VAR1(val);
}

void Camera::getAllTrimBitsList(IntList& val_list)
{
	DEB_MEMBER_FUNCT();
	int nb_sub_modules = getNbDetSubModules();
	val_list.resize(nb_sub_modules);
	for (int i = 0; i < nb_sub_modules; ++i)
		getAllTrimBits(i, val_list[i]);
}

void Camera::setSettings(Settings settings)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR1(settings);
	if (m_model) {
		if (!m_model->checkSettings(settings))
			THROW_HW_ERROR(InvalidValue) << DEB_VAR1(settings);
		typedef slsDetectorDefs::detectorSettings  DetSettings;
		DetSettings cam_settings = DetSettings(settings);
		m_det->setSettings(cam_settings);
	}
	m_settings = settings;
}

void Camera::getSettings(Settings& settings)
{
	DEB_MEMBER_FUNCT();
	settings = m_settings;
	DEB_RETURN() << DEB_VAR1(settings);
}

void Camera::setThresholdEnergy(int thres)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR1(thres);
	m_det->setThresholdEnergy(thres);
}

void Camera::getThresholdEnergy(int& thres)
{
	DEB_MEMBER_FUNCT();
	thres = m_det->getThresholdEnergy();
	DEB_RETURN() << DEB_VAR1(thres);
}

void Camera::setClockDiv(ClockDiv clock_div)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR1(clock_div);
	m_det->setSpeed(slsDetectorDefs::CLOCK_DIVIDER, clock_div);
}

void Camera::getClockDiv(ClockDiv& clock_div)
{
	DEB_MEMBER_FUNCT();
	int ret = m_det->setSpeed(slsDetectorDefs::CLOCK_DIVIDER, -1);
	if (ret == MultiSlsDetectorErr)
		THROW_HW_ERROR(Error) << "Error getting clock divider";
	clock_div = ClockDiv(ret);
	DEB_RETURN() << DEB_VAR1(clock_div);
}

void Camera::setReadoutFlags(ReadoutFlags flags)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR1(flags);

	if (!m_model)
		return;

	IntList flags_list;
	if (!m_model->checkReadoutFlags(flags, flags_list))
		THROW_HW_ERROR(InvalidValue) << DEB_VAR1(flags);

	IntList::const_iterator it, end = flags_list.end();
	for (it = flags_list.begin(); it != end; ++it) {
		typedef slsDetectorDefs::readOutFlags DetFlags;
		DetFlags det_flags = static_cast<DetFlags>(*it);
		m_det->setReadOutFlags(det_flags);
	}
}

void Camera::getReadoutFlags(ReadoutFlags& flags)
{
	DEB_MEMBER_FUNCT();
	typedef slsDetectorDefs::readOutFlags DetFlags;
	DetFlags det_flags = static_cast<DetFlags>(-1);
	int ret = m_det->setReadOutFlags(det_flags);
	if (ret == MultiSlsDetectorErr)
		THROW_HW_ERROR(Error) << "Error getting flags";
	flags = ReadoutFlags(ret);
	DEB_RETURN() << DEB_VAR1(flags);
}

void Camera::addValidReadoutFlags(DebObj *deb_ptr, ReadoutFlags flags, 
				  IntList& flag_list, NameList& flag_name_list)
{
	DEB_FROM_PTR(deb_ptr);
	ostringstream os;
	os << flags;
	DEB_RETURN() << DEB_VAR2(flags, os.str());
	flag_list.push_back(flags);
	flag_name_list.push_back(os.str());
}

void Camera::getValidReadoutFlags(IntList& flag_list, NameList& flag_name_list)
{
	DEB_MEMBER_FUNCT();
	flag_list.clear();
	flag_name_list.clear();

#define add_valid_flags(x) \
	do { \
		addValidReadoutFlags(&deb, x, flag_list, flag_name_list); \
	} while (0)

	if (!m_model)
		return;

	IntList aux_list;
	ReadoutFlags flags = Defs::Normal;
	if (m_model->checkReadoutFlags(flags, aux_list, true))
		add_valid_flags(flags);

	ReadoutFlags flag_mask = m_model->getReadoutFlagsMask();
	IntList det_flags;
	const unsigned int nb_bits = sizeof(ReadoutFlags) * 8;
	for (unsigned int i = 0; i < nb_bits; ++i) {
		int flags = (1 << i);
		if (flag_mask & flags)
			det_flags.push_back(flags);
	}

	int max_flags = (1 << det_flags.size());
	for (int n = 0; n < max_flags; ++n) {
		flags = ReadoutFlags(0);
		for (unsigned int i = 0; i < nb_bits; ++i) {
			if (n & (1 << i))
				flags = ReadoutFlags(flags | det_flags[i]);
		}
		if (m_model->checkReadoutFlags(flags, aux_list, true))
			add_valid_flags(flags);
	}
}
