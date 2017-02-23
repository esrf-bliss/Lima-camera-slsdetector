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

#include "SlsDetectorAcq.h"

namespace lima 
{

namespace SlsDetector
{

class AppPars 
{
 public:
	std::string config_fname;
	int nb_frames;
	double exp_time;
	double frame_period;
	int print_policy;
	bool save_raw;
	std::string out_dir;

	AppPars();
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


} // namespace SlsDetector


} // namespace lima



#endif // __TEST_SLS_DETECTOR_H
