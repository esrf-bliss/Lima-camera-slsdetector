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

#include "SlsDetectorCamera.h"
#include "SlsDetectorEiger.h"
#include "lima/AcqState.h"

#include <cstdlib>
#include <iomanip>
#include <fstream>

namespace lima 
{

namespace SlsDetector
{

class TestApp
{
	DEB_CLASS_NAMESPC(DebModTest, "TestApp", "SlsDetector");

 public:
	static const double WAIT_SLEEP_TIME;

	class Pars 
	{
		DEB_CLASS_NAMESPC(DebModTest, "TestApp::Pars", "SlsDetector");
	public:
		std::string config_fname;
		int nb_frames;
		double exp_time;
		double frame_period;
		bool raw_mode;
		int debug_type_flags;
		std::string out_dir;

		Pars();
		void parseArgs(Args& args);

	private:
		class ArgOptBase
		{
		public:
		ArgOptBase(string sopt, string lopt, string extra = "")
			: m_sopt(sopt), m_lopt(lopt), m_extra(extra)
			{}
			virtual ~ArgOptBase()
				{}

			virtual bool check(Args& args) = 0;

			bool hasExtra()
			{ return !m_extra.empty(); }

		protected:
			string m_sopt;
			string m_lopt;
			string m_extra;
		};

		template <class T>
			class ArgOpt : public ArgOptBase
		{
		public:
		ArgOpt(T& var, string sopt, string lopt, string extra = "") 
			: ArgOptBase(sopt, lopt, extra), m_var(var)
			{
				if (!hasExtra()) {
					istringstream is("0");
					is >> m_var;
				}
			}

			virtual bool check(Args& args)
			{
				string s = args[0];
				if ((s != m_sopt) && (s != m_lopt))
					return false;
				args.pop_front();
				if (hasExtra() && !args) {
					cerr << "Missing " << m_extra << endl;
					exit(1);
				}
				s = hasExtra() ? args.pop_front() : string("1");
				istringstream is(s);
				is >> m_var;
				return true;	
			}

		protected:
			T& m_var;
		};

		typedef std::set<AutoPtr<ArgOptBase> > OptList;

		void loadDefaults();
		void loadOpts();

		OptList m_opt_list;
	};

	class EdfHeaderKey
	{
	public:
	EdfHeaderKey(const string& key) : m_key(key)
		{}
	private:
		friend ostream& operator <<(ostream& os, const EdfHeaderKey& h);
		string m_key;
	};

	TestApp(int argc, char *argv[]);
	void run();

 private:
	class FrameCallback : public HwFrameCallback
	{
		DEB_CLASS_NAMESPC(DebModTest, "TestApp::FrameCallback", 
				  "SlsDetector");
	public:
		FrameCallback(TestApp *app);
	protected:
		virtual bool newFrameReady(const HwFrameInfoType& frame_info);
	private:
		TestApp *m_app;
	};

	friend class FrameCallback;

	bool newFrameReady(const HwFrameInfoType& frame_info);

	void save_raw_data(int start_frame, int nb_frames);
	void save_edf_data(int start_frame, int nb_frames);
	void save_edf_frame(ofstream& of, int acq_idx, int edf_idx);

	Pars m_pars;
	AutoPtr<SoftBufferAllocMgr> m_alloc_mgr;
	AutoPtr<StdBufferCbMgr> m_buffer_mgr;
	AutoPtr<Camera> m_cam;
	AutoPtr<Model> m_model;
	AcqState m_state;
	FrameCallback m_cb;
	Timestamp m_last_msg_timestamp;
};

ostream& operator <<(ostream& os, const TestApp::EdfHeaderKey& h)
{
	return os << setiosflags(ios::left) << resetiosflags(ios::right)
		  << setw(14) << setfill(' ') << h.m_key << " = ";
}



} // namespace SlsDetector


} // namespace lima



#endif // __TEST_SLS_DETECTOR_H
