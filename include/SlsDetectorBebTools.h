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

#ifndef __SLS_DETECTOR_BEB_TOOLS_H
#define __SLS_DETECTOR_BEB_TOOLS_H

#include "SlsDetectorCPUAffinity.h"

namespace lima 
{

namespace SlsDetector
{

class BebShell
{
	DEB_CLASS_NAMESPC(DebModCamera, "BebShell", "SlsDetector");
 public:
	BebShell(std::string hostname, std::string user = "root");
	~BebShell();

	std::string exec(std::string cmd);
 
 private:
	std::string readUntilPrompt();

	AutoMutex lock()
	{ return m_mutex; }

	Mutex m_mutex;
	AutoPtr<SystemCmdPipe> m_cmd;
	bool m_echo;
	std::string m_prompt;
};

class BebFpgaMem
{
	DEB_CLASS_NAMESPC(DebModCamera, "BebFpgaMem", "SlsDetector");
 public:
	BebFpgaMem(BebShell& shell);

	unsigned long read(unsigned long addr);
	void write(unsigned long addr, unsigned long val);

 private:
	BebShell& m_shell;
	std::string m_zmem_rd;
	std::string m_zmem_wr;
};

} // namespace SlsDetector

} // namespace lima



#endif // __SLS_DETECTOR_BEB_TOOLS_H
