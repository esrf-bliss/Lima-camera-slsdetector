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

using namespace std;
using namespace lima;
using namespace lima::SlsDetector;


void save_raw_data(string out_dir, int start_frame, int nb_frames, Camera& cam)
{
	FrameDim frame_dim;
	cam.getFrameDim(frame_dim, true);

	for (int i = 0; i < nb_frames; ++i) {
		ostringstream os;
		os << out_dir << "/eiger.bin." << i;
		cout << "+++ Saving raw to " << os.str() << " ..." << endl;
		ofstream of(os.str().c_str());
		char *buffer = cam.getBufferPtr(start_frame + i);
		of.write(buffer, frame_dim.getMemSize());
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

void save_edf_frame(ofstream& of, int acq_idx, int edf_idx, Camera& cam)
{
	FrameDim frame_dim;
	cam.getFrameDim(frame_dim);
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

	of.write(cam.getBufferPtr(acq_idx), image_bytes);
}

void save_edf_data(string out_dir, int start_frame, int nb_frames, Camera& cam)
{
	ostringstream os;
	os << out_dir << "/eiger.edf";
	cout << "+++ Saving EDF to " << os.str() << " ..." << endl;
	ofstream of(os.str().c_str());
	for (int i = 0; i < nb_frames; ++i)
		save_edf_frame(of, start_frame + i, i, cam);
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
		Camera cam(pars.config_fname);
		cam.setPrintPolicy(pars.print_policy);
		cam.setNbFrames(pars.nb_frames);
		cam.setExpTime(pars.exp_time);
		cam.setFramePeriod(pars.frame_period);
		cam.setSaveRaw(pars.save_raw);
		cam.prepareAcq();
		cam.startAcq();
		cam.waitAcq();
		if (pars.save_raw) {
			save_raw_data(pars.out_dir, 0, pars.nb_frames, cam);
		} else {
			save_edf_data(pars.out_dir, 0, pars.nb_frames, cam);
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
