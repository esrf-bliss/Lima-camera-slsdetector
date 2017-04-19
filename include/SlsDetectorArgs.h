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

#ifndef __SLS_DETECTOR_ARGS_H
#define __SLS_DETECTOR_ARGS_H

#include "lima/AutoObj.h"
#include "lima/Debug.h"

#include <string>
#include <vector>

namespace lima 
{

namespace SlsDetector
{

class Args
{
	DEB_CLASS_NAMESPC(DebModCamera, "Args", "SlsDetector");

public:
	Args();
	Args(unsigned int argc, char *argv[]);
	Args(const std::string& s);
	Args(const Args& o);

	void set(const std::string& s);
	void clear();

	unsigned int size()
	{ return m_argc; }
	operator char **()
	{ return m_argv; }
	operator bool()
	{ return bool(m_argc); }
	char *operator[](int pos)
	{ return m_argv[pos]; }

	Args& operator =(const std::string& s);

	std::string pop_front();
	void erase(int pos);

private:
	typedef std::vector<std::string> StringList;

	void update_argc_argv();
	
	StringList m_arg_list;
	unsigned int m_argc;
	AutoPtr<char *, true> m_argv;
};

} // namespace SlsDetector

} // namespace lima


#endif // __SLS_DETECTOR_ARGS_H
