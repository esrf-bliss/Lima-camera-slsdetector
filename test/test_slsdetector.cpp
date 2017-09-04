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

#include "test_slsdetector.h"

using namespace std;
using namespace lima;
using namespace lima::SlsDetector;

DEB_GLOBAL(DebModTest);

int main(int argc, char *argv[])
{
	DEB_GLOBAL_FUNCT();
	TestApp app(argc, argv);
	app.run();
	return 0;
};

TestApp::Pars::Pars()
{
	DEB_CONSTRUCTOR();
	loadDefaults();
	loadOpts();
}

void TestApp::Pars::loadDefaults()
{
	DEB_MEMBER_FUNCT();
	nb_frames = 10;
	exp_time = 2.0e-3;
	frame_period = 2.5e-3;
	raw_mode = false;
	debug_type_flags = 0;
	out_dir = "/tmp";
}

void TestApp::Pars::loadOpts()
{
	DEB_MEMBER_FUNCT();

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

	o = new ArgOpt<bool>(raw_mode, "-r", "--save-raw");
	m_opt_list.insert(o);

	o = new ArgOpt<int>(debug_type_flags, "-d", "--debug-type-flags", 
			    "debug type flags");
	m_opt_list.insert(o);

	o = new ArgOpt<string>(out_dir, "-o", "--out-dir",
			       "out_dir");
	m_opt_list.insert(o);
}

void TestApp::Pars::parseArgs(Args& args)
{
	DEB_MEMBER_FUNCT();

	string prog_name = args.pop_front();

	while (args && (*args[0] == '-')) {
		OptList::iterator it, end = m_opt_list.end();
		bool ok = false;
		for (it = m_opt_list.begin(); (it != end) && !ok; ++it)
			ok = ((*it)->check(args));
		if (!ok) {
			cerr << "Unknown option: " << args[0] << endl;
			exit(1);
		}
	}

	if (config_fname.empty()) {
		cerr << "Missing config file" << endl;
		exit(1);
	}
}

TestApp::FrameCallback::FrameCallback(TestApp *app)
	: m_app(app)
{
	DEB_CONSTRUCTOR();
}

bool TestApp::FrameCallback::newFrameReady(const HwFrameInfoType& frame_info)
{
	DEB_MEMBER_FUNCT();
	return m_app->newFrameReady(frame_info);
}

const double TestApp::WAIT_SLEEP_TIME = 0.2;

TestApp::TestApp(int argc, char *argv[])
	: m_cb(this)
{
	DEB_CONSTRUCTOR();

	Args args(argc, argv);
	m_pars.parseArgs(args);

	DebParams::enableTypeFlags(m_pars.debug_type_flags);

	m_cam = new Camera(m_pars.config_fname);
	m_alloc_mgr = new SoftBufferAllocMgr();
	m_buffer_mgr = new StdBufferCbMgr(*m_alloc_mgr);
	m_buffer_mgr->registerFrameCallback(m_cb);
}

void TestApp::run()
{
	DEB_MEMBER_FUNCT();

	try {
		m_cam->setNbFrames(m_pars.nb_frames);
		m_cam->setExpTime(m_pars.exp_time);
		m_cam->setFramePeriod(m_pars.frame_period);
		m_cam->setRawMode(m_pars.raw_mode);

		FrameDim frame_dim;
		m_cam->getFrameDim(frame_dim);
		int max_buffers = m_buffer_mgr->getMaxNbBuffers(frame_dim, 1);
		int nb_buffers = min(m_pars.nb_frames, max_buffers);
		m_buffer_mgr->allocBuffers(nb_buffers, 1, frame_dim);
		m_cam->setBufferCbMgr(m_buffer_mgr);

		m_cam->prepareAcq();
		m_last_msg_timestamp = Timestamp::now();

		m_state.set(AcqState::Acquiring);
		m_cam->startAcq();
		m_state.waitNot(AcqState::Acquiring);

		int first = max(0, m_pars.nb_frames - nb_buffers);
		int save_frames = min(m_pars.nb_frames, nb_buffers);
		if (m_pars.raw_mode) {
			save_raw_data(first, save_frames);
		} else {
			save_edf_data(first, save_frames);
		}
	} catch (string s) {
		cerr << "Exception: " << s << endl;
		exit(1);
	} catch (...) {
		cerr << "Exception" << endl;
		exit(1);
	}
}

bool TestApp::newFrameReady(const HwFrameInfoType& frame_info)
{
	DEB_MEMBER_FUNCT();

	if (frame_info.acq_frame_nb == m_pars.nb_frames - 1)
		m_state.set(AcqState::Finished);

	Timestamp timestamp = Timestamp::now();
	if (timestamp - m_last_msg_timestamp > WAIT_SLEEP_TIME) {
		int frames_caught = m_cam->getFramesCaught();
		const Camera::FrameMap& frame_map = m_cam->getFrameMap();
		int finished_frames = frame_map.getLastFinishedFrame() + 1;
		DEB_ALWAYS() << DEB_VAR3(frame_info.acq_frame_nb, 
					 frames_caught, finished_frames);
		DEB_PARAM() << DEB_VAR1(frame_map);
		m_last_msg_timestamp = timestamp;
	}

	return true;
}

void TestApp::save_raw_data(int start_frame, int nb_frames)
{
	DEB_MEMBER_FUNCT();
	FrameDim frame_dim = m_buffer_mgr->getFrameDim();

	for (int i = 0; i < nb_frames; ++i) {
		ostringstream os;
		os << m_pars.out_dir << "/eiger.bin." << i;
		DEB_TRACE() << "Saving raw to " << os.str();
		ofstream of(os.str().c_str());
		void *buffer = m_buffer_mgr->getFrameBufferPtr(start_frame + i);
		of.write(static_cast<char *>(buffer), frame_dim.getMemSize());
	}
}

void TestApp::save_edf_data(int start_frame, int nb_frames)
{
	DEB_MEMBER_FUNCT();
	ostringstream os;
	os << m_pars.out_dir << "/eiger.edf";
	DEB_TRACE() << "Saving EDF to " << os.str();
	ofstream of(os.str().c_str());
	for (int i = 0; i < nb_frames; ++i)
		save_edf_frame(of, start_frame + i, i);
}

void TestApp::save_edf_frame(ofstream& of, int acq_idx, int edf_idx)
{
	DEB_MEMBER_FUNCT();
	FrameDim frame_dim = m_buffer_mgr->getFrameDim();
	Size frame_size = frame_dim.getSize();
	int image_bytes = frame_dim.getMemSize();
	
	ostringstream os;
	os << "{" << endl;
	os << EdfHeaderKey("HeaderID") << setiosflags(ios::right) 
	   << "EH:" << setfill('0') << setw(6) << (edf_idx + 1) 
	   << ":" << setfill('0') << setw(6) << 0 
	   << ":" << setfill('0') << setw(6) << 0 << "; " << endl;
	os << EdfHeaderKey("ByteOrder") << "LowByteFirst" << "; " << endl;
	os << EdfHeaderKey("DataType") << "UnsignedShort" << "; " << endl;
	os << EdfHeaderKey("Size") << image_bytes << "; " << endl;
	os << EdfHeaderKey("Dim_1") << frame_size.getWidth() << "; " << endl;
	os << EdfHeaderKey("Dim_2") << frame_size.getHeight() << "; " << endl;
	os << EdfHeaderKey("Image") << edf_idx << "; " << endl;
	os << EdfHeaderKey("acq_frame_nb") << edf_idx << "; " << endl;

	const int HEADER_BLOCK = 1024;
	int rem = (HEADER_BLOCK - 2) - os.str().size() % HEADER_BLOCK;
	if (rem < 0)
		rem += HEADER_BLOCK;
	os << string(rem, '\n') << "}" << endl;
	of << os.str();

	void *buffer = m_buffer_mgr->getFrameBufferPtr(acq_idx);
	of.write(static_cast<char *>(buffer), image_bytes);
}

