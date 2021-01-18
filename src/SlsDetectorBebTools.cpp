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

#include "SlsDetectorBebTools.h"

using namespace std;
using namespace lima;
using namespace lima::SlsDetector;

BebShell::BebShell(string hostname, string user)
	: m_echo(true)
{
	DEB_CONSTRUCTOR();
	DEB_PARAM() << DEB_VAR2(hostname, user);

	const string user_host = user + "@" + hostname;

	string cmd = string("ssh -xatt " + user_host); // -tt -> force TTY
	m_cmd = new SystemCmdPipe(cmd, string("BebShell") + user_host, false);
	m_cmd->setPipe(SystemCmdPipe::StdIn, SystemCmdPipe::DoPipe);
	m_cmd->setPipe(SystemCmdPipe::StdOut, SystemCmdPipe::DoPipe);
	m_cmd->start();

	m_prompt = user_host + ":~" + ((user == "root") ? "# " : "$ ");
	readUntilPrompt();	
	m_prompt = "# ";
	exec(string("PS1='") + m_prompt + "'");
	exec("stty -echo -onlcr");
	m_echo = false;
}

BebShell::~BebShell()
{
	DEB_DESTRUCTOR();
	AutoMutex l = lock();
	Pipe& in = m_cmd->getPipe(SystemCmdPipe::StdIn);
	in.write("exit\n");
}

string BebShell::exec(string cmd)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR1(cmd);

	AutoMutex l = lock();

	Pipe& in = m_cmd->getPipe(SystemCmdPipe::StdIn);
	in.write(cmd + "\n");
	if (m_echo) {
		Pipe& out = m_cmd->getPipe(SystemCmdPipe::StdOut);
		out.read(cmd.size() + 2);
	}
		
	string s = readUntilPrompt();
	DEB_RETURN() << DEB_VAR1(s);
	return s;
}

string BebShell::readUntilPrompt()
{
	DEB_MEMBER_FUNCT();

	if (m_prompt.empty())
		return string();

	string s;
	Pipe& out = m_cmd->getPipe(SystemCmdPipe::StdOut);
	while ((s.size() < m_prompt.size()) || 
	       (s.rfind(m_prompt) != (s.size() - m_prompt.size())))
		s += out.read(1);
	return s.substr(0, s.size() - m_prompt.size());
}

BebFpgaMem::BebFpgaMem(BebShell& shell)
	: m_shell(shell), m_zmem_rd("executables/z_mem"),
	  m_zmem_wr("executables/z_mem_write")
{
	DEB_CONSTRUCTOR();
}

unsigned long BebFpgaMem::read(unsigned long addr)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR1(DEB_HEX(addr));

	ostringstream cmd;
	cmd << m_zmem_rd << " " << hex << showbase << addr;
	string res = m_shell.exec(cmd.str());
	istringstream is(res.substr(res.find("\n") + 1));
	unsigned long phys_addr, virt_addr, val;
	is >> hex >> phys_addr >> virt_addr >> val;
	if (phys_addr != addr)
		THROW_HW_ERROR(Error) << "addr mismatch: " 
				      << DEB_VAR2(DebHex(phys_addr),
						  DebHex(addr));
	DEB_RETURN() << DEB_VAR1(DEB_HEX(val));
	return val;
}

void BebFpgaMem::write(unsigned long addr, unsigned long val)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR2(DEB_HEX(addr), DEB_HEX(val));

	ostringstream cmd;
	cmd << m_zmem_wr << " " << hex << showbase << addr << " " << val;
	m_shell.exec(cmd.str());
}
