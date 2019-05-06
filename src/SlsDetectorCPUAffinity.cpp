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
#include "lima/CtAcquisition.h"
#include "lima/CtSaving.h"
#include "lima/SoftOpExternalMgr.h"
#include "lima/RegExUtils.h"
#include "lima/MiscUtils.h"

#include <signal.h>
#include <unistd.h>
#include <sys/wait.h>
#include <sys/types.h>
#include <pwd.h>
#include <numa.h>
#include <iomanip>

using namespace std;
using namespace lima;
using namespace lima::SlsDetector;

bool SystemCmd::UseSudo = true;

SystemCmd::SystemCmd(string cmd, string desc, bool try_sudo)
	: m_cmd(cmd), m_desc(desc), m_try_sudo(try_sudo),
	  m_stdin(NULL), m_stdout(NULL), m_stderr(NULL)
{
	DEB_CONSTRUCTOR();
	DEB_PARAM() << DEB_VAR3(m_cmd, m_desc, m_try_sudo);
}

SystemCmd::SystemCmd(const SystemCmd& o)
	: m_cmd(o.m_cmd), m_desc(o.m_desc), m_try_sudo(o.m_try_sudo),
	  m_stdin(o.m_stdin), m_stdout(o.m_stdout), m_stderr(o.m_stderr)
{
	DEB_CONSTRUCTOR();
	DEB_PARAM() << DEB_VAR3(m_cmd, m_desc, m_try_sudo);
	DEB_PARAM() << DEB_VAR3(m_stdin, m_stdout, m_stderr);
}

void SystemCmd::setUseSudo(bool use_sudo)
{
	UseSudo = use_sudo;
}

bool SystemCmd::getUseSudo()
{
	return UseSudo;
}

void SystemCmd::checkSudo()
{
	DEB_STATIC_FUNCT();

	typedef map<string, bool> CacheMap;
	static CacheMap cache_map;
	CacheMap::iterator it = cache_map.find(m_cmd);
	if (it == cache_map.end()) {
		SystemCmd sudo("sudo", "", false);
		sudo.args() << "-l " << m_cmd;
		bool ok = (sudo.execute() == 0);
		CacheMap::value_type entry(m_cmd, ok);
		pair<CacheMap::iterator, bool> v = cache_map.insert(entry);
		if (!v.second)
			THROW_HW_ERROR(Error) << "Error inserting cache entry";
		it = v.first;
	}
	if (it->second)
		return;

	const char *user = "Unknown";
	uid_t uid = getuid();
	DEB_TRACE() << DEB_VAR1(uid);
	struct passwd pw, *res;
	char buffer[4 * 1024];
	int ret = getpwuid_r(uid, &pw, buffer, sizeof(buffer), &res);
	if (ret != 0)
		DEB_WARNING() << "getpwuid_r failed: " << strerror(errno);
	else if (res == NULL)
		DEB_WARNING() << "could not get passwd entry for uid=" << uid;
	else
		user = pw.pw_name;

	DEB_ERROR() << "The command '" << m_cmd << "' is not allowed for "
		    << user << " in the sudoers database. ";
	DEB_ERROR() << "Check sudoers(5) man page and restart this process";
	if (!m_desc.empty())
		DEB_ERROR() << m_desc;

	THROW_HW_ERROR(Error) << "Cannot execute sudo " << m_cmd << "! "
			      << "See output for details";
}

void SystemCmd::setPipes(Pipe *stdin, Pipe *stdout, Pipe *stderr)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR3(stdin, stdout, stderr);
	m_stdin = stdin;
	m_stdout = stdout;
	m_stderr = stderr;
}

int SystemCmd::execute()
{
	DEB_MEMBER_FUNCT();
	string args = m_args.str();
	DEB_PARAM() << DEB_VAR3(m_cmd, args, m_try_sudo);

	ostringstream os;
	if (m_try_sudo && getUseSudo()) {
		checkSudo();
		os << "sudo -n ";
	}
	os << m_cmd << " " << args;
	if (!DEB_CHECK_ANY(DebTypeTrace) && !m_stdout)
		os << " > /dev/null";
	if (sameOutErr())
		os << " 2>&1";
	DEB_TRACE() << "executing: '" << os.str() << "'";
	preparePipes();
	int ret = system(os.str().c_str());
	restorePipes();
	DEB_RETURN() << DEB_VAR1(ret);
	return ret;
}

void SystemCmd::preparePipes()
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR3(m_stdin, m_stdout, m_stderr);
	if (m_stdin) {
		DEB_TRACE() << "Duplicating PIPE into STDIN";
		m_stdin->dupInto(Pipe::ReadFd, 0);
	}
	if (m_stderr && !sameOutErr()) {
		DEB_TRACE() << "Duplicating PIPE into STDERR";
		m_stderr->dupInto(Pipe::WriteFd, 2);
	}
	if (m_stdout) {
		DEB_TRACE() << "Duplicating PIPE into STDOUT";
		m_stdout->dupInto(Pipe::WriteFd, 1);
	}
}

void SystemCmd::restorePipes()
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR3(m_stdin, m_stdout, m_stderr);
	if (m_stdout) {
		m_stdout->restoreDup(Pipe::WriteFd);
		DEB_TRACE() << "Restored STDOUT";
	}
	if (m_stderr && !sameOutErr()) {
		m_stderr->restoreDup(Pipe::WriteFd);
		DEB_TRACE() << "Restored STDERR";
	}
	if (m_stdin) {
		DEB_TRACE() << "Restored STDIN";
		m_stdin->restoreDup(Pipe::ReadFd);
	}
}

SystemCmdPipe::SystemCmdPipe(string cmd, string desc, bool try_sudo)
	: m_pipe_list(NbPipes), m_child_pid(-1), m_cmd(cmd, desc, try_sudo)
{
	DEB_CONSTRUCTOR();
}

SystemCmdPipe::~SystemCmdPipe()
{
	DEB_DESTRUCTOR();

	if (m_child_pid >= 0)
		wait();
}

void SystemCmdPipe::start()
{
	DEB_MEMBER_FUNCT();

	if (m_child_pid >= 0)
		THROW_HW_ERROR(Error) << "cmd already running";

	m_cmd.setPipes(m_pipe_list[StdIn].ptr,
		       m_pipe_list[StdOut].ptr,
		       m_pipe_list[StdErr].ptr);

	m_child_pid = fork();
	if (m_child_pid == 0) {
		m_pipe_list[StdIn].close(Pipe::WriteFd);
		m_pipe_list[StdOut].close(Pipe::ReadFd);
		m_pipe_list[StdErr].close(Pipe::ReadFd);

		int ret = m_cmd.execute();
		DEB_RETURN() << DEB_VAR1(ret);
		_exit(ret);
	} else {
		m_pipe_list[StdIn].close(Pipe::ReadFd);
		m_pipe_list[StdOut].close(Pipe::WriteFd);
		m_pipe_list[StdErr].close(Pipe::WriteFd);
	}
}

void SystemCmdPipe::wait()
{
	DEB_MEMBER_FUNCT();
	if (m_child_pid < 0)
		THROW_HW_ERROR(Error) << "cmd not running";
	int child_ret;
	waitpid(m_child_pid, &child_ret, 0);
	DEB_TRACE() << DEB_VAR1(child_ret);
	m_child_pid = -1;
}

void SystemCmdPipe::setPipe(PipeIdx idx, PipeType type)
{
	DEB_MEMBER_FUNCT();
	m_pipe_list[idx] = PipeData(type);
}

Pipe& SystemCmdPipe::getPipe(PipeIdx idx)
{
	DEB_MEMBER_FUNCT();
	if (!m_pipe_list[idx])
		THROW_HW_ERROR(InvalidValue) << "null pipe " << DEB_VAR1(idx);
	return *m_pipe_list[idx].ptr;
}

int CPUAffinity::findNbSystemCPUs()
{
	DEB_STATIC_FUNCT();
	int nb_cpus = 0;
	const char *proc_file_name = "/proc/cpuinfo";
	ifstream proc_file(proc_file_name);
	while (proc_file) {
		char buffer[1024];
		proc_file.getline(buffer, sizeof(buffer));
		istringstream is(buffer);
		string t;
		is >> t;
		if (t == "processor")
			++nb_cpus;
	}
	DEB_RETURN() << DEB_VAR1(nb_cpus);
	return nb_cpus;
}

int CPUAffinity::findMaxNbSystemCPUs()
{
	DEB_STATIC_FUNCT();
	NumericGlob proc_glob("/proc/sys/kernel/sched_domain/cpu");
	int max_nb_cpus = proc_glob.getNbEntries();
	DEB_RETURN() << DEB_VAR1(max_nb_cpus);
	return max_nb_cpus;
}

int CPUAffinity::getNbSystemCPUs(bool max_nb)
{
	static int nb_cpus = 0;
	EXEC_ONCE(nb_cpus = findNbSystemCPUs());
	static int max_nb_cpus = 0;
	EXEC_ONCE(max_nb_cpus = findMaxNbSystemCPUs());
	int cpus = (max_nb && max_nb_cpus) ? max_nb_cpus : nb_cpus;
	return cpus;
}

std::string CPUAffinity::getProcDir(bool local_threads)
{
	DEB_STATIC_FUNCT();
	DEB_PARAM() << DEB_VAR1(local_threads);
	ostringstream os;
	os << "/proc/";
	if (local_threads)
		os << getpid() << "/task/";
	string proc_dir = os.str();
	DEB_RETURN() << DEB_VAR1(proc_dir);
	return proc_dir;
}

std::string CPUAffinity::getTaskProcDir(pid_t task, bool is_thread)
{
	DEB_STATIC_FUNCT();
	DEB_PARAM() << DEB_VAR2(task, is_thread);
	ostringstream os;
	os << getProcDir(is_thread) << task << "/";
	string proc_dir = os.str();
	DEB_RETURN() << DEB_VAR1(proc_dir);
	return proc_dir;
}

void CPUAffinity::initCPUSet(cpu_set_t& cpu_set) const
{
	CPU_ZERO(&cpu_set);
	uint64_t mask = getMask();
	for (unsigned int i = 0; i < sizeof(mask) * 8; ++i) {
		if ((mask >> i) & 1)
			CPU_SET(i, &cpu_set);
	}
}

void CPUAffinity::applyToTask(pid_t task, bool incl_threads,
			      bool use_taskset) const
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR4(*this, task, incl_threads, use_taskset);

	string proc_status = getTaskProcDir(task, !incl_threads) + "status";
	if (access(proc_status.c_str(), F_OK) != 0)
		return;

	if (use_taskset)
		applyWithTaskset(task, incl_threads);
	else
		applyWithSetAffinity(task, incl_threads);
}

void CPUAffinity::applyWithTaskset(pid_t task, bool incl_threads) const
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR3(*this, task, incl_threads);

	SystemCmd taskset("taskset");
	const char *all_tasks_opt = incl_threads ? "-a " : "";
	taskset.args() << all_tasks_opt << "-p " << *this << " " << task;
	if (taskset.execute() != 0) {
		const char *th = incl_threads ? "and threads " : "";
		THROW_HW_ERROR(Error) << "Error setting task " << task 
				      << " " << th << "CPU affinity";
	}
}

void CPUAffinity::applyWithSetAffinity(pid_t task, bool incl_threads) const
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR3(*this, task, incl_threads);

	IntList task_list;
	if (incl_threads) {
		string proc_dir = getTaskProcDir(task, false) + "task/";
		NumericGlob proc_glob(proc_dir);
		typedef NumericGlob::IntStringList IntStringList;
		IntStringList list = proc_glob.getIntPathList();
		if (list.empty())
			THROW_HW_ERROR(Error) << "Cannot find task " << task 
					      << " threads";
		IntStringList::const_iterator it, end = list.end();
		for (it = list.begin(); it != end; ++it)
			task_list.push_back(it->first);
	} else {
		task_list.push_back(task);
	}

	cpu_set_t cpu_set;
	initCPUSet(cpu_set);
	IntList::const_iterator it, end = task_list.end();
	for (it = task_list.begin(); it != end; ++it) {
		DEB_TRACE() << "setting " << task << " CPU mask: " << *this;
		int ret = sched_setaffinity(task, sizeof(cpu_set), &cpu_set);
		if (ret != 0) {
			const char *th = incl_threads ? "and threads " : "";
			THROW_HW_ERROR(Error) << "Error setting task " << task 
					      << " " << th << "CPU affinity";
		}
	}
}

void CPUAffinity::getNUMANodeMask(vector<unsigned long>& node_mask,
				  int& max_node)
{
	DEB_MEMBER_FUNCT();

	typedef vector<unsigned long> Array;

	int nb_nodes = numa_max_node() + 1;
	const int item_bits = sizeof(Array::reference) * 8;
	int nb_items = nb_nodes / item_bits;
	if (nb_nodes % item_bits != 0)
		++nb_items;

	DEB_PARAM() << DEB_VAR2(*this, nb_items);
	max_node = nb_nodes + 1;

	node_mask.assign(nb_items, 0);

	uint64_t mask = getMask();
	for (unsigned int i = 0; i < sizeof(mask) * 8; ++i) {
		if ((mask >> i) & 1) {
			unsigned int n = numa_node_of_cpu(i);
			Array::reference v = node_mask[n / item_bits];
			v |= 1L << (n % item_bits);
		}
	}

	if (DEB_CHECK_ANY(DebTypeReturn)) {
		ostringstream os;
		os << hex << "0x" << setw(nb_nodes / 4) << setfill('0');
		bool first = true;
		Array::reverse_iterator it, end = node_mask.rend();
		for (it = node_mask.rbegin(); it != end; ++it, first = false)
			os << (!first ? "," : "") << *it;
		DEB_RETURN() << "node_mask=" << os.str() << ", "
			     << DEB_VAR1(max_node);
	}
}

bool IrqMgr::m_irqbalance_stopped = false;
StringList IrqMgr::m_dev_list;

IrqMgr::IrqMgr(string net_dev)
{
	DEB_CONSTRUCTOR();
	DEB_PARAM() << DEB_VAR1(net_dev);

	if (!net_dev.empty())
		setDev(net_dev);
}

IrqMgr::~IrqMgr()
{
	DEB_DESTRUCTOR();
	DEB_PARAM() << DEB_VAR1(m_net_dev);

	if (isManaged(m_net_dev))
		updateRxQueueIrqAffinity(true);
}

void IrqMgr::setDev(string net_dev)
{
	DEB_CONSTRUCTOR();
	DEB_PARAM() << DEB_VAR1(net_dev);

	if (!m_net_dev.empty())
		THROW_HW_ERROR(Error) << "device already set";
	if (isManaged(net_dev))
		THROW_HW_ERROR(Error) << net_dev << " already managed";

	m_net_dev = net_dev;
}

bool IrqMgr::isManaged(string net_dev)
{
	if (net_dev.empty())
		return false;
	StringList::iterator it, end = m_dev_list.end();
	it = find(m_dev_list.begin(), end, net_dev);
	return (it != end);
}

IntList IrqMgr::getIrqList()
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR1(m_net_dev);

	string pci_irq_dir = (string("/sys/class/net/") + m_net_dev + 
			      "/device/msi_irqs/");
	NumericGlob pci_irqs(pci_irq_dir);
	typedef NumericGlob::IntStringList IntStringList;
	IntStringList dev_irqs = pci_irqs.getIntPathList();

	IntList act_irqs;
	ifstream proc_ints("/proc/interrupts");
	ostringstream os;
	int nb_cpus = CPUAffinity::getNbSystemCPUs();
	os << "[ \t]*"
	   << "(?P<irq>[0-9]+):[ \t]+"
	   << "(?P<counts>(([0-9]+)[ \t]+){" << nb_cpus << "})"
	   << "(?P<type>[-A-Za-z0-9_]+)[ \t]+"
	   << "(?P<name>[-A-Za-z0-9_]+)";
	DEB_TRACE() << DEB_VAR1(os.str());
	RegEx int_re(os.str());
	while (proc_ints) {
		char buffer[1024];
		proc_ints.getline(buffer, sizeof(buffer));
		string s = buffer;
		DEB_TRACE() << DEB_VAR1(s);
		RegEx::FullNameMatchType match;
		if (!int_re.matchName(s, match))
			continue;
		int irq;
		istringstream(match["irq"]) >> irq;
		DEB_TRACE() << "found match: " << irq << ": " 
			     << string(match["name"]);
		IntStringList::iterator it, end = dev_irqs.end();
		bool ok = false;
		for (it = dev_irqs.begin(); !ok && (it != end); ++it)
			ok = (it->first == irq);
		if (ok)
			act_irqs.push_back(irq);
	}

	DEB_RETURN() << PrettyIntList(act_irqs);
	return act_irqs;
}

void IrqMgr::updateRxQueueIrqAffinity(bool default_affinity)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR2(m_net_dev, default_affinity);

	if (isManaged(m_net_dev) && default_affinity) {
		StringList::iterator it, end = m_dev_list.end();
		it = find(m_dev_list.begin(), end, m_net_dev);
		m_dev_list.erase(it);
		if (m_dev_list.empty())
			restoreIrqBalance();
	} else if (!isManaged(m_net_dev) && !default_affinity) {
		stopIrqBalance();
		m_dev_list.push_back(m_net_dev);
	}
}

void IrqMgr::stopIrqBalance()
{
	DEB_STATIC_FUNCT();
	DEB_PARAM() << DEB_VAR1(m_irqbalance_stopped);
	if (!getIrqBalanceActive()) {
		DEB_TRACE() << "nothing to do";
		return;
	} else if (m_irqbalance_stopped) {
		DEB_WARNING() << "irqbalance stopped and still running!";
	}
	setIrqBalanceActive(false);
	m_irqbalance_stopped = true;
}

void IrqMgr::restoreIrqBalance()
{
	DEB_STATIC_FUNCT();
	DEB_PARAM() << DEB_VAR1(m_irqbalance_stopped);
	if (!m_irqbalance_stopped)
		return;
	setIrqBalanceActive(true);
	m_irqbalance_stopped = false;
}

bool IrqMgr::getIrqBalanceActive()
{
	DEB_STATIC_FUNCT();
	SystemCmd bash("bash", "", false);
	ConstStr cmd = "ps -ef | grep -v grep | grep irqbalance";
	bash.args() << "-c '" << cmd << "'";
	bool act = (bash.execute() == 0);
	DEB_RETURN() << DEB_VAR1(act);
	return act;
}

void IrqMgr::setIrqBalanceActive(bool act)
{
	DEB_STATIC_FUNCT();
	DEB_PARAM() << DEB_VAR1(act);
	SystemCmd service("service");
	ConstStr cmd = act ? "start" : "stop";
	DEB_ALWAYS() << "irqbalance: executing " << cmd;
	service.args() << "irqbalance " << cmd;
	if (service.execute() != 0)
		THROW_HW_ERROR(Error) << "Could not " << cmd << " "
				      << "irqbalance service";
	DEB_ALWAYS() << "Done!";
}


NetDevRxQueueMgr::NetDevRxQueueMgr(string dev)
	: m_dev(dev), m_irq_mgr(dev)
{
	DEB_CONSTRUCTOR();
	DEB_PARAM() << DEB_VAR1(m_dev);
}

NetDevRxQueueMgr::~NetDevRxQueueMgr()
{
	DEB_DESTRUCTOR();
	if (!NetDevRxQueueAffinityMap_isDefault(m_aff_map))
		apply(NetDevRxQueueAffinityMap());
}

void NetDevRxQueueMgr::setDev(string dev)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR1(dev);
	if (m_dev.empty()) {
		m_dev = dev;
		m_irq_mgr.setDev(dev);
	} else if (dev != m_dev) {
		THROW_HW_ERROR(InvalidValue) << "name mismatch: "
					     << DEB_VAR2(dev, m_dev);
	}
}

void NetDevRxQueueMgr::checkDev()
{
	DEB_MEMBER_FUNCT();
	if (m_dev.empty())
		THROW_HW_ERROR(InvalidValue) << "no device defined yet";
}

void NetDevRxQueueMgr::apply(int queue, const Affinity& queue_affinity)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR3(m_dev, queue, queue_affinity);

	checkDev();
	apply(Irq, queue, queue_affinity.irq);	
	m_irq_mgr.updateRxQueueIrqAffinity(queue_affinity.irq.isDefault());
	apply(Processing, queue, queue_affinity.processing);
	m_aff_map[queue] = queue_affinity;
}

void NetDevRxQueueMgr::apply(const AffinityMap& affinity_map)
{
	m_aff_map.clear();
	if (NetDevRxQueueAffinityMap_isDefault(affinity_map)) {
		apply(-1, NetDevRxQueueCPUAffinity());
	} else {
		AffinityMap::const_iterator qit, qend = affinity_map.end();
		for (qit = affinity_map.begin(); qit != qend; ++qit)
			apply(qit->first, qit->second);
	}
}

void NetDevRxQueueMgr::apply(Task task, int queue, CPUAffinity a)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR3(m_dev, queue, a);

	FileSetterData file_setter;
	if (task == Irq)
		getIrqFileSetterData(queue, file_setter);
	else
		getProcessingFileSetterData(queue, file_setter);

	static bool did_error_files[NbTasks] = {false, false};
	if (!did_error_files[task]) {
		const StringList& list = file_setter.file;
		StringList::const_iterator it, end = list.end();
		bool ok = true;
		for (it = list.begin(); ok && (it != end); ++it)
			ok = applyWithFile(*it, a);
		if (ok)
			return;
		DEB_WARNING() << "Could not write to files. Will try setter...";
		did_error_files[task] = true;
	}

	static bool did_error_setter[NbTasks] = {false, false};
	if (!did_error_setter[task]) {
		const StringList& list = file_setter.setter;
		StringList::const_iterator it, end = list.end();
		bool ok = true;
		for (it = list.begin(); ok && (it != end); ++it)
			ok = applyWithSetter(task, *it, a);
		if (ok)
			return;
		DEB_ERROR() << "Could not use setter";
		did_error_setter[task] = true;
	}
}

void NetDevRxQueueMgr::getIrqFileSetterData(int queue,
					    FileSetterData& file_setter)
{
	DEB_MEMBER_FUNCT();

	if (queue != -1)
		THROW_HW_ERROR(NotSupported) << "only all queues (-1) mode "
					     << "is supported";

	IntList act_irqs = m_irq_mgr.getIrqList();
	IntList::const_iterator it, end = act_irqs.end();
	for (it = act_irqs.begin(); it != end; ++it) {
		ostringstream os1, os2;
		os1 << "/proc/irq/" << *it << "/smp_affinity";
		file_setter.file.push_back(os1.str());
		os2 << *it;
		file_setter.setter.push_back(os2.str());
	}
}

void NetDevRxQueueMgr::getProcessingFileSetterData(int queue,
						   FileSetterData& file_setter)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR1(m_dev);

	enum {
		File, Setter, NbLists,
	};

	string glob_str(string("/sys/class/net/" + m_dev + "/queues/rx-"));
	NumericGlob rps_cpus_glob(glob_str, "/rps_cpus");

	typedef NumericGlob::IntStringList IntStringList;
	IntStringList list_array[NbLists];
	list_array[File] = rps_cpus_glob.getIntPathList();
	list_array[Setter] = rps_cpus_glob.getIntSubPathList(6);
	IntList filter_list;
	if (queue == -2)
		filter_list = getRxQueueList();
	else if (queue != -1)
		filter_list.push_back(queue);
	for (int i = 0; !filter_list.empty() && (i < NbLists); ++i) {
		IntStringList& list = list_array[i];
		while (true) {
			IntStringList::iterator lit, lend = list.end();
			bool ok = true;
			for (lit = list.begin(); ok && (lit != lend); ++lit) {
				IntList::iterator fit, fend;
				fend = filter_list.end();
				fit = find(filter_list.begin(), fend,
					   lit->first);
				ok = (fit != fend);
				if (!ok)
					list.erase(lit);
			}
			if (ok)
				break;
		}
	}
	if (list_array[File].size() != list_array[Setter].size()) {
		THROW_HW_ERROR(Error) << "File and Setter lists differ";
	} else if (list_array[File].empty()) {
		DEB_WARNING() << "No Rx queue (" << queue << ") for " << m_dev;
		return;
	}

	StringList *file_setter_list[NbLists] = {&file_setter.file,
						 &file_setter.setter};
	for (int i = 0; i < NbLists; ++i) {
		const IntStringList& list = list_array[i];
		IntStringList::const_iterator it, end = list.end();
		for (it = list.begin(); it != end; ++it)
			file_setter_list[i]->push_back(it->second);
	}
}

bool NetDevRxQueueMgr::applyWithFile(const string& fname, CPUAffinity a)
{
	DEB_MEMBER_FUNCT();

	ostringstream os;
	os << hex << a.getZeroDefaultMask();
	DEB_TRACE() << "writing " << os.str() << " to " << fname;
	ofstream aff_file(fname.c_str());
	if (aff_file)
		aff_file << os.str();
	if (aff_file)
		aff_file.close();
	bool file_ok(aff_file);
	DEB_RETURN() << DEB_VAR1(file_ok);
	return file_ok;
}

bool NetDevRxQueueMgr::applyWithSetter(Task task, const string& irq_queue,
				       CPUAffinity a)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR2(m_dev, irq_queue);

	static string desc = getSetterSudoDesc();
	SystemCmd setter(AffinitySetterName, desc);
	ConstStr task_opt = (task == Irq) ? "-i" : "-r";
	setter.args() << task_opt << " " << m_dev << " " << irq_queue << " "
		      << hex << "0x" << a.getZeroDefaultMask();
	bool setter_ok = (setter.execute() == 0);
	DEB_RETURN() << DEB_VAR1(setter_ok);
	return setter_ok;
}

string NetDevRxQueueMgr::getSetterSudoDesc()
{
	DEB_STATIC_FUNCT();

	const string& setter_name = AffinitySetterName;
	string dir = "/tmp";
	string fname = dir + "/" + setter_name + ".c";
	ofstream src_file(fname.c_str());
	if (src_file) {
		const StringList& SrcList = AffinitySetterSrc;
		StringList::const_iterator it, end = SrcList.end();
		for (it = SrcList.begin(); src_file && (it != end); ++it)
			src_file << *it << endl;
		if (src_file)
			src_file.close();
		if (!src_file)
			DEB_WARNING() << "Error writing to " << fname;
	} else {
		DEB_WARNING() << "Error creating " << fname;
	}

	ostringstream desc;
	string aux_setter = dir + "/" + setter_name;
	desc << "In order to create " << setter_name << ", compile " << fname
	     << " with the following commands: " << endl
	     << "  gcc -Wall -o " << aux_setter << " " << fname << endl
	     << "  su -c \"cp " << aux_setter << " /usr/local/bin\"" << endl;
	return desc.str();
}

const string NetDevRxQueueMgr::AffinitySetterName = 
					"netdev_set_queue_cpu_affinity";

static const char *NetDevRxQueueMgrAffinitySetterSrcCList[] = {
"#include <stdio.h>",
"#include <stdlib.h>",
"#include <string.h>",
"#include <errno.h>",
"#include <unistd.h>",
"#include <sys/types.h>",
"#include <sys/stat.h>",
"#include <fcntl.h>",
"",
"int main(int argc, char *argv[])",
"{",
"	char *dev, *irq_queue, *p, fname[256], buffer[128];",
"	int irq, rps, fd, len, ret;",
"	long aff;",
"",
"	if (argc != 5)",
"		exit(1);",
"	irq = (strcmp(argv[1], \"-i\") == 0);",
"	rps = (strcmp(argv[1], \"-r\") == 0);",
"	if (!irq && !rps)",
"		exit(2);",
"	if (!strlen(argv[2]) || !strlen(argv[3]) || !strlen(argv[4]))",
"		exit(2);",
"",
"	dev = argv[2];",
"	irq_queue = argv[3];",
"",
"	errno = 0;",
"	aff = strtol(argv[4], &p, 0);",
"	if (errno || *p)",
"		exit(3);",
"",
"	len = sizeof(fname);",
"	if (irq)",
"		ret = snprintf(fname, len, \"/proc/irq/%s/smp_affinity\",", 
"			       irq_queue);",
"	else",
"		ret = snprintf(fname, len, \"/sys/class/net/%s/queues/%s/rps_cpus\",", 
"			       dev, irq_queue);",
"	if ((ret < 0) || (ret == len))",
"		exit(4);",
"",
"	len = sizeof(buffer);",
"	ret = snprintf(buffer, len, \"%016lx\", aff);",
"	if ((ret < 0) || (ret == len))",
"		exit(5);",
"",
"	fd = open(fname, O_WRONLY);",
"	if (fd < 0)",
"		exit(6);",
"",	
"	for (p = buffer; *p; p += ret)",
"		if ((ret = write(fd, p, strlen(p))) < 0)",
"			exit(7);",
"",
"	if (close(fd) < 0)",
"		exit(8);",
"	return 0;",
"}",
};
const StringList NetDevRxQueueMgr::AffinitySetterSrc(
		C_LIST_ITERS(NetDevRxQueueMgrAffinitySetterSrcCList));

IntList NetDevRxQueueMgr::getRxQueueList()
{
	DEB_MEMBER_FUNCT();

	checkDev();

	IntList queue_list;

	SystemCmdPipe ethtool("ethtool");
	ethtool.args() << "-S " << m_dev;
	ethtool.setPipe(SystemCmdPipe::StdOut, SystemCmdPipe::DoPipe);
	ethtool.start();
	Pipe& child_out = ethtool.getPipe(SystemCmdPipe::StdOut);

	RegEx re("^[ \t]*rx_queue_(?P<queue>[0-9]+)_packets:[ \t]+"
		 "(?P<packets>[0-9]+)\n$");
	while (true) {
		string s = child_out.readLine(1024, "\n");
		if (s.empty())
			break;

		RegEx::FullNameMatchType match;
		if (!re.matchName(s, match))
			continue;

		int queue;
		istringstream(match["queue"]) >> queue;
		unsigned long packets;
		istringstream(match["packets"]) >> packets;
		if (packets != 0) {
			DEB_TRACE() << m_dev << " RxQueue " << queue << ": "
				    << packets << " packets";
			queue_list.push_back(queue);
		}
	}
	if (queue_list.empty())
		DEB_WARNING() << "No queue found for " << m_dev;

	DEB_RETURN() << DEB_VAR1(PrettyIntList(queue_list));
	return queue_list;
}

SystemCPUAffinityMgr::WatchDog::WatchDog()
	: m_cmd_pipe(0, true), m_res_pipe(0, true)
{
	DEB_CONSTRUCTOR();

	m_lima_pid = getpid();

	m_child_pid = fork();
	if (m_child_pid == 0) {
		m_child_pid = getpid();
		DEB_TRACE() << DEB_VAR2(m_lima_pid, m_child_pid);

		m_cmd_pipe.close(Pipe::WriteFd);
		m_res_pipe.close(Pipe::ReadFd);

		signal(SIGINT, SIG_IGN);
		signal(SIGTERM, sigTermHandler);

		childFunction();
		_exit(0);
	} else {
		m_cmd_pipe.close(Pipe::ReadFd);
		m_res_pipe.close(Pipe::WriteFd);

		sendChildCmd(Packet(Init));
		DEB_TRACE() << "Child is ready";
	}
}

SystemCPUAffinityMgr::WatchDog::~WatchDog()
{
	DEB_DESTRUCTOR();

	if (!childEnded()) {
		sendChildCmd(Packet(CleanUp));
		waitpid(m_child_pid, NULL, 0);
	}
}

void SystemCPUAffinityMgr::WatchDog::sigTermHandler(int /*signo*/)
{
}

bool SystemCPUAffinityMgr::WatchDog::childEnded()
{
	return (waitpid(m_child_pid, NULL, WNOHANG) != 0);
}

void SystemCPUAffinityMgr::WatchDog::sendChildCmd(const Packet& packet)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR1(packet.cmd);

	if (childEnded())
		THROW_HW_ERROR(Error) << "Watchdog child process killed: " 
				      << m_child_pid;

	void *p = static_cast<void *>(const_cast<Packet *>(&packet));
	string s(static_cast<char *>(p), sizeof(packet));
	m_cmd_pipe.write(s);

	s = m_res_pipe.read(1);
	Cmd res = Cmd(s.data()[0]);
	if (res != Ok)
		THROW_HW_ERROR(Error) << "Invalid watchdog child ack";
	DEB_TRACE() << "Watchdog child acknowledged Ok";
}

SystemCPUAffinityMgr::WatchDog::Packet
SystemCPUAffinityMgr::WatchDog::readParentCmd()
{
	DEB_MEMBER_FUNCT();
	Packet packet;
	string s = m_cmd_pipe.read(sizeof(packet));
	if (s.empty())
		THROW_HW_ERROR(Error) << "Watchdog cmd pipe closed/intr";
	memcpy(&packet, s.data(), s.size());
	DEB_RETURN() << DEB_VAR1(packet.cmd);
	return packet;
}

void SystemCPUAffinityMgr::WatchDog::ackParentCmd()
{
	DEB_MEMBER_FUNCT();
	m_res_pipe.write(string(1, Ok));
}

void SystemCPUAffinityMgr::WatchDog::childFunction()
{
	DEB_MEMBER_FUNCT();

	Packet first = readParentCmd();
	if (first.cmd != Init) {
		DEB_ERROR() << "Invalid watchdog init cmd: " << first.cmd;
		return;
	}
	ackParentCmd();

	bool cleanup_req = false;

	try {
		do {
			Packet packet = readParentCmd();
			if (packet.cmd == SetProcAffinity) {
				procAffinitySetter(packet.u.proc_affinity);
			} else if (packet.cmd == SetNetDevAffinity) {
				NetDevGroupCPUAffinity netdev_affinity =
					netDevAffinityEncode(packet);
				netDevAffinitySetter(netdev_affinity);
			} else if (packet.cmd == CleanUp) {
				cleanup_req = true;
			}
			if (packet.cmd != CleanUp)
				ackParentCmd();
		} while (!cleanup_req);

	} catch (...) {
		DEB_ALWAYS() << "Watchdog parent and/or child killed!";
	}

	DEB_TRACE() << "Clean-up";
	procAffinitySetter(CPUAffinity());
	netDevAffinitySetter(NetDevGroupCPUAffinity());

	if (cleanup_req)
		ackParentCmd(); 
}

void 
SystemCPUAffinityMgr::WatchDog::procAffinitySetter(CPUAffinity cpu_affinity)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR1(cpu_affinity);

	static bool aff_set = false;
	if (aff_set && (cpu_affinity == m_other))
		return;

	ProcList proc_list = getOtherProcList(cpu_affinity);
	if (proc_list.empty())
		return;

	DEB_ALWAYS() << "Setting CPUAffinity for " << PrettyIntList(proc_list)
		     << " to " << cpu_affinity;
	ProcList::const_iterator it, end = proc_list.end();
	for (it = proc_list.begin(); it != end; ++it)
		cpu_affinity.applyToTask(*it);
	DEB_ALWAYS() << "Done!";

	m_other = cpu_affinity;
	aff_set = true;
}

ProcList 
SystemCPUAffinityMgr::WatchDog::getOtherProcList(CPUAffinity cpu_affinity)
{
	DEB_MEMBER_FUNCT();

	ProcList proc_list = getProcList(NoMatchAffinity, cpu_affinity);
	pid_t ignore_proc_list[] = { m_lima_pid, m_child_pid, 0 };
	for (pid_t *p = ignore_proc_list; *p; ++p) {
		ProcList::iterator it, end = proc_list.end();
		it = find(proc_list.begin(), end, *p);
		if (it != end)
			proc_list.erase(it);
	}

	return proc_list;
}

string SystemCPUAffinityMgr::WatchDog::concatStringList(StringList list)
{
	ostringstream os;
	os << list;
	return os.str().substr(1, os.str().size() - 2);
}

StringList SystemCPUAffinityMgr::WatchDog::splitStringList(string str)
{
	StringList list;
	string::size_type i, p, n;
	for (i = 0; (i != string::npos) && (i != str.size()); i = p) {
		p = str.find(",", i);
		n = (p == string::npos) ? p : (p - i);
		list.push_back(string(str, i, n));
		if (p != string::npos)
			++p;
	}
	return list;
}

NetDevGroupCPUAffinity
SystemCPUAffinityMgr::WatchDog::netDevAffinityEncode(const Packet& packet)
{
	DEB_MEMBER_FUNCT();

	NetDevGroupCPUAffinity netdev_affinity;
	const PacketNetDevAffinity& packet_affinity = packet.u.netdev_affinity;
	netdev_affinity.name_list = splitStringList(packet_affinity.name_list);
	NetDevAffinityMap& queue_affinity = netdev_affinity.queue_affinity;
	unsigned int nb_queues = packet_affinity.queue_affinity_len;
	const PacketNetDevQueueAffinity *a = packet_affinity.queue_affinity;
	for (unsigned int i = 0; i < nb_queues; ++i, ++a) {
		NetDevRxQueueCPUAffinity qa;
		qa.irq = a->irq;
		qa.processing = a->processing;
		NetDevRxQueueAffinityMap::value_type v(a->queue, qa);
		queue_affinity.insert(v);
	}
	return netdev_affinity;
}

void SystemCPUAffinityMgr::WatchDog::netDevAffinitySetter(
				const NetDevGroupCPUAffinity& netdev_affinity)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR1(netdev_affinity);

	NetDevMgrMap& netdev_map = m_netdev_mgr_map;

	StringList nl = netdev_affinity.name_list;
	if (nl.empty()) {
		if (netdev_map.empty())
			return;
		NetDevMgrMap::const_iterator it, end = netdev_map.end();
		for (it = netdev_map.begin(); it != end; ++it)
			nl.push_back(it->first);
	}

	DEB_ALWAYS() << "setting " << DEB_VAR1(netdev_affinity);

	StringList::const_iterator dit, dend = nl.end();
	for (dit = nl.begin(); dit != dend; ++dit) {
		const string& dev = *dit;
		NetDevRxQueueMgr& mgr = netdev_map[dev];
		mgr.setDev(dev);
		mgr.apply(netdev_affinity.queue_affinity);
	}

	DEB_ALWAYS() << "Done!";

	bool erase = netdev_affinity.isDefault();
	if (!erase)
		return;

	StringList::const_iterator it, end = nl.end();
	for (it = nl.begin(); it != end; ++it) {
		NetDevMgrMap::iterator sit, send = netdev_map.end();
		sit = netdev_map.find(*it);
		if (sit != send)
			netdev_map.erase(sit);
	}
}

void 
SystemCPUAffinityMgr::WatchDog::setOtherCPUAffinity(CPUAffinity cpu_affinity)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR1(cpu_affinity);
	Packet packet(SetProcAffinity);
	packet.u.proc_affinity = cpu_affinity;
	sendChildCmd(packet);
}

void SystemCPUAffinityMgr::WatchDog::setNetDevCPUAffinity(
				NetDevGroupCPUAffinity netdev_affinity)
{
	DEB_MEMBER_FUNCT();
	string name_list = concatStringList(netdev_affinity.name_list);
	NetDevAffinityMap& queue_affinity = netdev_affinity.queue_affinity;
	DEB_PARAM() << DEB_VAR2(name_list, queue_affinity.size());
	if (name_list.size() > StringLen)
		THROW_HW_ERROR(InvalidValue) << "Child cmd string too long";
	if (queue_affinity.size() > AffinityMapLen)
		THROW_HW_ERROR(InvalidValue) << "Child affinity map too long";

	Packet packet(SetNetDevAffinity);
	PacketNetDevAffinity& ndga = packet.u.netdev_affinity;
	strcpy(ndga.name_list, name_list.c_str());
	ndga.queue_affinity_len = queue_affinity.size();
	NetDevAffinityMap::const_iterator it, end = queue_affinity.end();
	PacketNetDevQueueAffinity *a = ndga.queue_affinity;
	for (it = queue_affinity.begin(); it != end; ++it, ++a) {
		a->queue = it->first;
		a->irq = it->second.irq;
		a->processing = it->second.processing;
	}
	sendChildCmd(packet);
}

SystemCPUAffinityMgr::SystemCPUAffinityMgr()
{
	DEB_CONSTRUCTOR();
}

SystemCPUAffinityMgr::~SystemCPUAffinityMgr()
{
	DEB_DESTRUCTOR();
}

ProcList
SystemCPUAffinityMgr::getProcList(Filter filter, CPUAffinity cpu_affinity)
{
	DEB_STATIC_FUNCT();
	DEB_PARAM() << DEB_VAR2(filter, cpu_affinity);

	ProcList proc_list;
	bool this_proc = filter & ThisProc;
	filter = Filter(filter & ~ThisProc);
	string proc_dir = CPUAffinity::getProcDir(this_proc);
	NumericGlob proc_glob(proc_dir, "/status");
	NumericGlob::IntStringList list = proc_glob.getIntPathList();
	NumericGlob::IntStringList::const_iterator it, end = list.end();
	for (it = list.begin(); it != end; ++it) {
		int pid = it->first;
		const string& fname = it->second;
		bool has_vm = false;
		bool has_good_affinity = (filter == All);
		ifstream status_file(fname.c_str());
		while (status_file) {
			string s;
			status_file >> s;
			RegEx re;
			FullMatch full_match;

			re = "VmSize:?$";
			if (re.match(s, full_match)) {
				has_vm = true;
				continue;
			}

			re = "Cpus_allowed:?$";
			if (re.match(s, full_match) && (filter != All)) {
				uint64_t int_affinity;
				status_file >> hex >> int_affinity >> dec;
				CPUAffinity affinity = int_affinity;
				bool aff_match = (affinity == cpu_affinity);
				bool filt_match = (filter == MatchAffinity);
				has_good_affinity = (aff_match == filt_match);
				continue;
			}
		}
		DEB_TRACE() << DEB_VAR3(pid, has_vm, has_good_affinity);
		if (has_vm && has_good_affinity)
			proc_list.push_back(pid);
	}

	if (DEB_CHECK_ANY(DebTypeReturn))
		DEB_RETURN() << DEB_VAR1(PrettyList<ProcList>(proc_list));
	return proc_list;
}


ProcList
SystemCPUAffinityMgr::getThreadList(Filter filter, 
					  CPUAffinity cpu_affinity)
{
	DEB_STATIC_FUNCT();
	DEB_PARAM() << DEB_VAR2(filter, cpu_affinity);
	return getProcList(Filter(filter | ThisProc), cpu_affinity);
}

void SystemCPUAffinityMgr::checkWatchDogStart()
{
	if (!m_watchdog || m_watchdog->childEnded())
		m_watchdog = new WatchDog();
}

void SystemCPUAffinityMgr::checkWatchDogStop()
{
	if (m_other.isDefault() && m_netdev.empty())
		m_watchdog = NULL;
}

void SystemCPUAffinityMgr::setOtherCPUAffinity(CPUAffinity cpu_affinity)
{
	DEB_MEMBER_FUNCT();

	checkWatchDogStart();
	m_watchdog->setOtherCPUAffinity(cpu_affinity);
	m_other = cpu_affinity;
	checkWatchDogStop();
}

void SystemCPUAffinityMgr::setNetDevCPUAffinity(
			const NetDevGroupCPUAffinityList& netdev_list)
{
	DEB_MEMBER_FUNCT();

	checkWatchDogStart();

	if (!netdev_list.empty()) {
		typedef NetDevGroupCPUAffinityList NetDevList;
		NetDevList::const_iterator it, end = netdev_list.end();
		for (it = netdev_list.begin(); it != end; ++it)
			m_watchdog->setNetDevCPUAffinity(*it);
	} else {
		m_watchdog->setNetDevCPUAffinity(NetDevGroupCPUAffinity());
	}

	m_netdev = netdev_list;
	checkWatchDogStop();
}

RecvCPUAffinity::RecvCPUAffinity()
	: listeners(1), writers(1), recv_threads(1)
{
}

RecvCPUAffinity& RecvCPUAffinity::operator =(CPUAffinity a)
{
	listeners.assign(1, a);
	writers.assign(1, a);
	recv_threads.assign(1, a);
	return *this;
}

CPUAffinity GlobalCPUAffinity::all() const
{
	return (RecvCPUAffinityList_all(recv) |
		NetDevGroupCPUAffinityList_all(netdev) | lima | other);
}

void GlobalCPUAffinity::updateRecvAffinity(CPUAffinity a)
{
	RecvCPUAffinityList::iterator it, end = recv.end();
	for (it = recv.begin(); it != end; ++it)
		*it = a;
}

GlobalCPUAffinityMgr::
ProcessingFinishedEvent::ProcessingFinishedEvent(GlobalCPUAffinityMgr *mgr)
	: m_mgr(mgr), m_cb(this), m_ct(NULL)
{
	DEB_CONSTRUCTOR();

	m_nb_frames = 0;
	m_cnt_act = false;
	m_saving_act = false;
	m_stopped = false;
	m_last_cb_ts = Timestamp::now();
}

GlobalCPUAffinityMgr::
ProcessingFinishedEvent::~ProcessingFinishedEvent()
{
	DEB_DESTRUCTOR();
	if (m_mgr)
		m_mgr->m_proc_finished = NULL;
}

void GlobalCPUAffinityMgr::ProcessingFinishedEvent::prepareAcq()
{
	DEB_MEMBER_FUNCT();
	if (!m_ct)
		return;

	CtAcquisition *acq = m_ct->acquisition();
	acq->getAcqNbFrames(m_nb_frames);

	SoftOpExternalMgr *extOp = m_ct->externalOperation();
	if (DEB_CHECK_ANY(DebTypeTrace)) {
		typedef list<string> NameList;
		typedef map<int, NameList> OpStageNameListMap;
		OpStageNameListMap active_op;
		extOp->getActiveOp(active_op);
		OpStageNameListMap::const_iterator mit, mend = active_op.end();
		ostringstream os;
		os << "{";
		for (mit = active_op.begin(); mit != mend; ++mend) {
			const int& stage = mit->first;
			os << stage << ": [";
			const NameList& name_list = mit->second;
			NameList::const_iterator lit, lend = name_list.end();
			const char *sep = "";
			for (lit = name_list.begin(); lit != lend; ++lit) {
				const string& name = *lit;
				os << sep << name;
				sep = ",";
			}
			os << "]";
		}
		os << "}";
		DEB_TRACE() << "extOp->getActiveOp()=" << os.str();
	}

	bool extOp_link_task_act, extOp_sink_task_act;
	extOp->isTaskActive(extOp_link_task_act, extOp_sink_task_act);
	DEB_TRACE() << DEB_VAR2(extOp_link_task_act, extOp_sink_task_act);
	m_cnt_act = extOp_sink_task_act;

	CtSaving *saving = m_ct->saving();
	CtSaving::SavingMode mode;
	saving->getSavingMode(mode);
	m_saving_act = (mode != CtSaving::Manual);

	DEB_TRACE() << DEB_VAR3(m_nb_frames, m_saving_act, m_cnt_act);

	m_stopped = false;
}

void GlobalCPUAffinityMgr::ProcessingFinishedEvent::stopAcq()
{
	DEB_MEMBER_FUNCT();
	m_stopped = true;
}

void GlobalCPUAffinityMgr::ProcessingFinishedEvent::processingFinished()
{
	DEB_MEMBER_FUNCT();
	if (m_mgr)
		m_mgr->limaFinished();
}

void GlobalCPUAffinityMgr::
ProcessingFinishedEvent::registerStatusCallback(CtControl *ct)
{
	DEB_MEMBER_FUNCT();
	if (m_ct)
		THROW_HW_ERROR(Error) << "StatusCallback already registered";

	ct->registerImageStatusCallback(m_cb);
	m_ct = ct;
}

void GlobalCPUAffinityMgr::
ProcessingFinishedEvent::limitUpdateRate()
{
	Timestamp next_ts = m_last_cb_ts + Timestamp(1.0 / 10);
	double remaining = next_ts - Timestamp::now();
	if (remaining > 0)
		Sleep(remaining);
}

void GlobalCPUAffinityMgr::
ProcessingFinishedEvent::updateLastCallbackTimestamp()
{
	m_last_cb_ts = Timestamp::now();
}

Timestamp GlobalCPUAffinityMgr::
ProcessingFinishedEvent::getLastCallbackTimestamp()
{
	return m_last_cb_ts;
}

void GlobalCPUAffinityMgr::
ProcessingFinishedEvent::imageStatusChanged(
					const CtControl::ImageStatus& status)
{
	DEB_MEMBER_FUNCT();

	limitUpdateRate();
	updateLastCallbackTimestamp();

	int max_frame = m_nb_frames - 1; 
	int last_frame = status.LastImageAcquired;
	bool finished = (m_stopped || (last_frame == max_frame));
	finished &= (status.LastImageReady == last_frame);
	finished &= (!m_cnt_act || (status.LastCounterReady == last_frame));
	finished &= (m_stopped || !m_saving_act || 
		     (status.LastImageSaved == last_frame));
	if (finished)
		processingFinished();
}

GlobalCPUAffinityMgr::GlobalCPUAffinityMgr(Camera *cam)
	: m_cam(cam), m_proc_finished(NULL), 
	  m_lima_finished_timeout(0.5)
{
	DEB_CONSTRUCTOR();

	m_state = Ready;
}

GlobalCPUAffinityMgr::~GlobalCPUAffinityMgr()
{
	DEB_DESTRUCTOR();

	setLimaAffinity(CPUAffinity());

	if (m_proc_finished)
		m_proc_finished->m_mgr = NULL;
}

void GlobalCPUAffinityMgr::applyAndSet(const GlobalCPUAffinity& o)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR1(o);

	if (!m_cam)
		THROW_HW_ERROR(InvalidValue) << "apply without camera";

	CPUAffinity all_system = o.all();
	if (all_system.getNbCPUs() == CPUAffinity::getNbSystemCPUs() / 2)
		DEB_WARNING() << "Hyper-threading seems activated!";

	setLimaAffinity(o.lima);
	setRecvAffinity(o.recv);

	if (!m_system_mgr)
		m_system_mgr = new SystemCPUAffinityMgr();

	m_system_mgr->setOtherCPUAffinity(o.other);
	m_curr.other = o.other;
	m_system_mgr->setNetDevCPUAffinity(o.netdev);
	m_curr.netdev = o.netdev;

	m_set = o;
}

void GlobalCPUAffinityMgr::setLimaAffinity(CPUAffinity lima_affinity)
{
	DEB_MEMBER_FUNCT();

	if (lima_affinity == m_curr.lima)
		return;

	if (m_lima_tids.size()) {
		ProcList::const_iterator it, end = m_lima_tids.end();
		for (it = m_lima_tids.begin(); it != end; ++it)
			// do not use taskset!
			lima_affinity.applyToTask(*it, false, false);
	} else {
		pid_t pid = getpid();
		lima_affinity.applyToTask(pid, true);
		m_curr.updateRecvAffinity(lima_affinity);
	}
	m_curr.lima = lima_affinity;
}

void GlobalCPUAffinityMgr::setRecvAffinity(
			   const RecvCPUAffinityList& recv_affinity_list)
{
	DEB_MEMBER_FUNCT();

	if (recv_affinity_list == m_curr.recv)
		return;

	m_cam->setRecvCPUAffinity(recv_affinity_list);

	const RecvCPUAffinityList& l = recv_affinity_list;
	RecvCPUAffinity::Selector s = &RecvCPUAffinity::RecvThreads;
	CPUAffinity buffer_affinity = RecvCPUAffinityList_all(l, s);
	DEB_ALWAYS() << DEB_VAR1(buffer_affinity);
	m_cam->m_buffer_ctrl_obj->setCPUAffinityMask(buffer_affinity);

	m_curr.recv = recv_affinity_list;
}

void GlobalCPUAffinityMgr::updateRecvRestart()
{
	DEB_MEMBER_FUNCT();
	m_curr.updateRecvAffinity(m_curr.lima);
}

GlobalCPUAffinityMgr::ProcessingFinishedEvent *
GlobalCPUAffinityMgr::getProcessingFinishedEvent()
{
	DEB_MEMBER_FUNCT();
	if (!m_proc_finished)
		m_proc_finished = new ProcessingFinishedEvent(this);
	return m_proc_finished;
}

void GlobalCPUAffinityMgr::prepareAcq()
{
	DEB_MEMBER_FUNCT();
	AutoMutex l = lock();
	if (m_state != Ready)
		THROW_HW_ERROR(Error) << "GlobalCPUAffinityMgr is not Ready: "
				      << "missing ProcessingFinishedEvent";
	if (m_proc_finished)
		m_proc_finished->prepareAcq();
	m_lima_tids.clear();
}

void GlobalCPUAffinityMgr::startAcq()
{
	DEB_MEMBER_FUNCT();
	AutoMutex l = lock();
	m_state = Acquiring;
}

void GlobalCPUAffinityMgr::stopAcq()
{
	DEB_MEMBER_FUNCT();
	if (m_proc_finished)
		m_proc_finished->stopAcq();
}

void GlobalCPUAffinityMgr::recvFinished()
{
	DEB_MEMBER_FUNCT();

	AutoMutex l = lock();
	if (!m_proc_finished)
		m_state = Ready;
	if (m_state == Ready) 
		return;

	CPUAffinity recv_all = RecvCPUAffinityList_all(m_curr.recv);
	DEB_TRACE() << DEB_VAR2(m_curr.lima, recv_all);
	if (m_curr.lima != recv_all) {
		m_state = Changing;
		AutoMutexUnlock u(l);
		SystemCPUAffinityMgr::Filter filter;
		filter = SystemCPUAffinityMgr::MatchAffinity;
		m_lima_tids = SystemCPUAffinityMgr::getThreadList(filter,
								m_curr.lima);
		DEB_ALWAYS() << "Lima TIDs: " << PrettyIntList(m_lima_tids);
		CPUAffinity lima_affinity = m_curr.lima | recv_all;
		DEB_ALWAYS() << "Allowing Lima to run on Recv CPUs: " 
			     << lima_affinity;
		setLimaAffinity(lima_affinity);
	}

	m_state = Processing;
	m_cond.broadcast();
}

void GlobalCPUAffinityMgr::limaFinished()
{
	DEB_MEMBER_FUNCT();

	AutoMutex l = lock();
	if (m_state == Acquiring)
		m_state = Ready;
	while ((m_state != Processing) && (m_state != Ready))
		m_cond.wait();
	if (m_state == Ready)
		return;

	if (m_curr.lima != m_set.lima) {
		m_state = Restoring;
		AutoMutexUnlock u(l);
		DEB_ALWAYS() << "Restoring Lima to dedicated CPUs: " 
			     << m_set.lima;
		setLimaAffinity(m_set.lima);
		setRecvAffinity(m_set.recv);
	}

	m_state = Ready;
	m_cond.broadcast();
}

void GlobalCPUAffinityMgr::waitLimaFinished()
{
	DEB_MEMBER_FUNCT();

	if (!m_proc_finished)
		return;

	m_proc_finished->updateLastCallbackTimestamp();

	AutoMutex l = lock();
	while (m_state != Ready) {
		if (m_cond.wait(0.1))
			continue;

		AutoMutexUnlock u(l);
		Timestamp ts = m_proc_finished->getLastCallbackTimestamp();
		double elapsed = Timestamp::now() - ts;
		if (elapsed < m_lima_finished_timeout)
			continue;

		DEB_ERROR() << "No ImageStatusCallback in " << elapsed << " s";
		limaFinished();
	}
}

void GlobalCPUAffinityMgr::cleanUp()
{
	DEB_MEMBER_FUNCT();
	AutoMutex l = lock();
	m_state = Ready;
}

ostream& lima::SlsDetector::operator <<(ostream& os, const CPUAffinity& a)
{
	return os << hex << "0x" << setw(CPUAffinity::getNbHexDigits())
		  << setfill('0') << a.getMask() << dec << setfill(' ');
}

ostream&
lima::SlsDetector::operator <<(ostream& os, const CPUAffinityList& l)
{
	os << "[";
	bool first = true;
	CPUAffinityList::const_iterator it, end = l.end();
	for (it = l.begin(); it != end; ++it, first = false)
		os << (first ? "" : ", ") << *it;
	return os << "]";
}

ostream&
lima::SlsDetector::operator <<(ostream& os, const NetDevRxQueueCPUAffinity& a)
{
	return os << "<" << "irq=" << a.irq << ", processing=" << a.processing
		  << ">";
}

ostream&
lima::SlsDetector::operator <<(ostream& os, const NetDevGroupCPUAffinity& a)
{
	os << "<" << a.name_list << ", [";
	bool first = true;
	const NetDevRxQueueAffinityMap& m = a.queue_affinity;
	if (a.isDefault()) {
		os << "default";
	} else {
		NetDevRxQueueAffinityMap::const_iterator it, end = m.end();
		for (it = m.begin(); it != end; ++it, first = false)
			os << (first ? "" : ", ") << it->first << ": " 
			   << it->second;
	}
	return os << "]>";
}

ostream& lima::SlsDetector::operator <<(ostream& os, const RecvCPUAffinity& a)
{
	os << "<";
	os << "listeners=" << a.listeners << ", writers=" << a.writers << ", "
	   << "recv_threads=" << a.recv_threads;
	return os << ">";
}

ostream&
lima::SlsDetector::operator <<(ostream& os, const RecvCPUAffinityList& l)
{
	os << "[";
	bool first = true;
	RecvCPUAffinityList::const_iterator it, end = l.end();
	for (it = l.begin(); it != end; ++it, first = false)
		os << (first ? "" : ", ") << *it;
	return os << "]";
}

ostream& lima::SlsDetector::operator <<(ostream& os, const GlobalCPUAffinity& a)
{
	os << "<";
	os << "recv=" << a.recv << ", lima=" << a.lima << ", other=" << a.other;
	return os << ">";
}

ostream& 
lima::SlsDetector::operator <<(ostream& os, const PixelDepthCPUAffinityMap& m)
{
	os << "[";
	bool first = true;
	PixelDepthCPUAffinityMap::const_iterator it, end = m.end();
	for (it = m.begin(); it != end; ++it, first = false)
		os << (!first ? ", " : "") << it->first << ": " << it->second;
	return os << "]";
}

