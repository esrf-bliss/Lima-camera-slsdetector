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

#include "multiSlsDetectorCommand.h"

#include <limits.h>
#include <algorithm>
#include <cmath>
#include <sys/syscall.h>
#include <glob.h>
#include <signal.h>
#include <sys/wait.h>

using namespace std;
using namespace lima;
using namespace lima::SlsDetector;


pid_t gettid() {
	return syscall(SYS_gettid);
}


Camera::TimeRangesChangedCallback::TimeRangesChangedCallback()
	: m_cam(NULL)
{
	DEB_CONSTRUCTOR();
}

Camera::TimeRangesChangedCallback::~TimeRangesChangedCallback()
{
	DEB_DESTRUCTOR();

	if (m_cam)
		m_cam->unregisterTimeRangesChangedCallback(*this);
}

Camera::Glob::Glob(string pattern)
	: m_pattern(pattern)
{
	DEB_CONSTRUCTOR();
	if (!m_pattern.empty())
		m_found_list = find(m_pattern);
}

Camera::Glob& Camera::Glob::operator =(std::string pattern)
{
	if (!pattern.empty())
		m_found_list = find(pattern);
	else
		m_found_list.clear();
	m_pattern = pattern;
	return *this;
}

Camera::StringList Camera::Glob::find(string pattern)
{
	DEB_STATIC_FUNCT();
	DEB_PARAM() << DEB_VAR1(pattern);

	glob_t glob_data;
	int ret = glob(pattern.c_str(), 0, NULL, &glob_data);
	if (ret == GLOB_NOMATCH)
		return StringList();
	else if (ret != 0)
		THROW_HW_ERROR(Error) << "Error in glob: " << ret;

	StringList found;
	try {
		char **begin = glob_data.gl_pathv;
		found.assign(begin, begin + glob_data.gl_pathc);
	} catch (...) {
		globfree(&glob_data);
		throw;
	}
	globfree(&glob_data);
	return found;
}

Camera::StringList Camera::Glob::split(string path)
{
	DEB_STATIC_FUNCT();
	DEB_PARAM() << DEB_VAR1(path);

	StringList split;
	if (path.empty())
		return split;
	size_t i = 0;
	if (path[i] == '/') {
		split.push_back("/");
		++i;
	}
	size_t end = path.size();
	while (i < end) {
		size_t sep = path.find("/", i);
		split.push_back(path.substr(i, sep - i));
		i = (sep == string::npos) ? end : (sep + 1);
	}
	if (DEB_CHECK_ANY(DebTypeReturn)) {
		ostringstream os;
		os << "[";
		for (unsigned int i = 0; i < split.size(); ++i)
			os << (i ? "," : "") << '"' << split[i] << '"';
		os << "]";
		DEB_RETURN() << "split=" << os.str();
	}
	return split;
}

Camera::StringList Camera::Glob::getSubPathList(int idx) const
{
	StringList sub_path_list;
	StringList::const_iterator it, end = m_found_list.end();
	for (it = m_found_list.begin(); it != end; ++it)
		sub_path_list.push_back(split(*it).at(idx));
	return sub_path_list;
}

Camera::NumericGlob::NumericGlob(string pattern_prefix, string pattern_suffix)
	: m_prefix_len(0), m_suffix_len(0)
{
	DEB_CONSTRUCTOR();
	if (pattern_prefix.empty())
		THROW_HW_ERROR(InvalidValue) << "Empty pattern prefix";

	StringList prefix_dir_list = Glob::split(pattern_prefix);
	m_nb_idx = prefix_dir_list.size();
	if (*pattern_prefix.rbegin() != '/') {
		--m_nb_idx;
		m_prefix_len = prefix_dir_list.back().size();
	}
	if (!pattern_suffix.empty() && (pattern_suffix[0] != '/'))
		m_suffix_len = Glob::split(pattern_suffix)[0].size();
	DEB_TRACE() << DEB_VAR3(m_nb_idx, m_prefix_len, m_suffix_len);
	m_glob = pattern_prefix + "[0-9]*" + pattern_suffix;
}

Camera::NumericGlob::IntStringList Camera::NumericGlob::getIntPathList() const
{
	DEB_MEMBER_FUNCT();
	IntStringList list;
	StringList path_list = m_glob.getPathList();
	StringList::const_iterator pit = path_list.begin();
	StringList sub_path_list = m_glob.getSubPathList(m_nb_idx);
	StringList::const_iterator it, end = sub_path_list.end();
	for (it = sub_path_list.begin(); it != end; ++it, ++pit) {
		size_t l = (*it).size() - m_prefix_len - m_suffix_len;
		string s = (*it).substr(m_prefix_len, l);
		int nb;
		istringstream(s) >> nb;
		DEB_TRACE() << DEB_VAR2(nb, *pit);
		list.push_back(IntString(nb, *pit));
	}
	sort(list.begin(), list.end());
	return list;
}

bool Camera::CPUAffinity::UseSudo = true;

int Camera::CPUAffinity::findNbCPUs()
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

int Camera::CPUAffinity::findMaxNbCPUs()
{
	DEB_STATIC_FUNCT();
	NumericGlob proc_glob("/proc/sys/kernel/sched_domain/cpu");
	int max_nb_cpus = proc_glob.getNbEntries();
	DEB_RETURN() << DEB_VAR1(max_nb_cpus);
	return max_nb_cpus;
}

int Camera::CPUAffinity::getNbCPUs(bool max_nb)
{
	static int nb_cpus = 0;
	EXEC_ONCE(nb_cpus = findNbCPUs());
	static int max_nb_cpus = 0;
	EXEC_ONCE(max_nb_cpus = findMaxNbCPUs());
	int cpus = (max_nb && max_nb_cpus) ? max_nb_cpus : nb_cpus;
	return cpus;
}

void Camera::CPUAffinity::initCPUSet(cpu_set_t& cpu_set) const
{
	CPU_ZERO(&cpu_set);
	uint64_t mask = *this;
	for (unsigned int i = 0; i < sizeof(mask) * 8; ++i) {
		if ((mask >> i) & 1)
			CPU_SET(i, &cpu_set);
	}
}

void Camera::CPUAffinity::applyToTask(pid_t task, bool incl_threads,
				      bool use_taskset) const
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR3(task, incl_threads, use_taskset);

	if (kill(task, 0) != 0)
		return;

	if (use_taskset)
		applyWithTaskset(task, incl_threads);
	else
		applyWithSetAffinity(task, incl_threads);
}

void Camera::CPUAffinity::applyWithTaskset(pid_t task, bool incl_threads) const
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR2(task, incl_threads);

	ostringstream os;
	uint64_t mask = *this;
	if (UseSudo)
		os << "sudo ";
	const char *all_tasks_opt = incl_threads ? "-a " : "";
	os << "taskset " << all_tasks_opt
	   << "-p " << hex << showbase << mask << " " 
	   << dec << noshowbase << task << " > /dev/null 2>&1";
	DEB_TRACE() << "executing: '" << os.str() << "'";
	int ret = system(os.str().c_str());
	if (ret != 0) {
		const char *th = incl_threads ? "and threads " : "";
		THROW_HW_ERROR(Error) << "Error setting task " << task 
				      << " " << th << "CPU affinity";
	}
}

void Camera::CPUAffinity::applyWithSetAffinity(pid_t task, bool incl_threads)
									const
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR2(task, incl_threads);

	IntList task_list;
	if (incl_threads) {
		ostringstream os;
		os << "/proc/" << task << "/task/";
		NumericGlob proc_glob(os.str());
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

Camera::ProcCPUAffinityMgr::WatchDog::WatchDog()
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

		sendChildCmd(Init);
		DEB_TRACE() << "Child is ready";
	}
}

Camera::ProcCPUAffinityMgr::WatchDog::~WatchDog()
{
	DEB_DESTRUCTOR();

	if (!childEnded()) {
		sendChildCmd(CleanUp);
		waitpid(m_child_pid, NULL, 0);
	}
}

void Camera::ProcCPUAffinityMgr::WatchDog::sigTermHandler(int /*signo*/)
{
}

bool Camera::ProcCPUAffinityMgr::WatchDog::childEnded()
{
	return (waitpid(m_child_pid, NULL, WNOHANG) != 0);
}

void Camera::ProcCPUAffinityMgr::WatchDog::sendChildCmd(Cmd cmd, Arg arg)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR2(cmd, DebHex(arg));

	if (childEnded())
		THROW_HW_ERROR(Error) << "Watchdog child process killed: " 
				      << m_child_pid;
	Packet packet = {cmd, arg};
	void *p = static_cast<void *>(&packet);
	string s(static_cast<char *>(p), sizeof(packet));
	m_cmd_pipe.write(s);

	s = m_res_pipe.read(1);
	Cmd res = Cmd(s.data()[0]);
	if (res != Ok)
		THROW_HW_ERROR(Error) << "Invalid watchdog child ack";
	DEB_TRACE() << "Watchdog child acknowledged Ok";
}

Camera::ProcCPUAffinityMgr::WatchDog::Packet
Camera::ProcCPUAffinityMgr::WatchDog::readParentCmd()
{
	DEB_MEMBER_FUNCT();
	Packet packet;
	string s = m_cmd_pipe.read(sizeof(packet));
	if (s.empty())
		THROW_HW_ERROR(Error) << "Watchdog cmd pipe closed/intr";
	memcpy(&packet, s.data(), s.size());
	DEB_RETURN() << DEB_VAR2(packet.cmd, DebHex(packet.arg));
	return packet;
}

void Camera::ProcCPUAffinityMgr::WatchDog::ackParentCmd()
{
	DEB_MEMBER_FUNCT();
	m_res_pipe.write(string(1, Ok));
}

void Camera::ProcCPUAffinityMgr::WatchDog::childFunction()
{
	DEB_MEMBER_FUNCT();

	Packet first = readParentCmd();
	if (first.cmd != Init) {
		DEB_ERROR() << "Invalid watchdog init cmd: " << first.cmd;
		return;
	}
	ackParentCmd();

	bool last_was_clean = false;
	bool cleanup_req = false;

	try {
		do {
			Packet packet = readParentCmd();
			if (packet.cmd == SetAffinity) {
				CPUAffinity cpu_affinity = packet.arg;
				affinitySetter(cpu_affinity);
				ackParentCmd();
				last_was_clean = cpu_affinity.isDefault();
			} else if (packet.cmd == CleanUp) {
				cleanup_req = true;
			}
		} while (!cleanup_req);

	} catch (...) {
		DEB_ALWAYS() << "Watchdog parent and/or child killed!";
	}

	DEB_TRACE() << "Clean-up";
	if (!last_was_clean)
		affinitySetter(CPUAffinity());

	if (cleanup_req)
		ackParentCmd(); 
}

void 
Camera::ProcCPUAffinityMgr::WatchDog::affinitySetter(CPUAffinity cpu_affinity)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR1(cpu_affinity);

	ProcList proc_list = getOtherProcList(cpu_affinity);
	if (proc_list.empty())
		return;

	DEB_ALWAYS() << "Setting CPUAffinity for " << PrettyIntList(proc_list)
		     << " to " << cpu_affinity;
	ProcList::const_iterator it, end = proc_list.end();
	for (it = proc_list.begin(); it != end; ++it)
		cpu_affinity.applyToTask(*it);
	DEB_ALWAYS() << "Done!";
}

Camera::ProcList 
Camera::ProcCPUAffinityMgr::WatchDog::getOtherProcList(CPUAffinity cpu_affinity)
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

void 
Camera::ProcCPUAffinityMgr::WatchDog::setOtherCPUAffinity(CPUAffinity cpu_affinity)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR1(cpu_affinity);
	sendChildCmd(SetAffinity, cpu_affinity);
}

Camera::ProcCPUAffinityMgr::ProcCPUAffinityMgr()
{
	DEB_CONSTRUCTOR();
}

Camera::ProcCPUAffinityMgr::~ProcCPUAffinityMgr()
{
	DEB_DESTRUCTOR();
}

Camera::ProcList
Camera::ProcCPUAffinityMgr::getProcList(Filter filter, CPUAffinity cpu_affinity)
{
	DEB_STATIC_FUNCT();
	DEB_PARAM() << DEB_VAR2(filter, cpu_affinity);

	ProcList proc_list;
	bool this_proc = filter & ThisProc;
	filter = Filter(filter & ~ThisProc);
	ostringstream os;
	os << "/proc/";
	if (this_proc)
		os << getpid() << "/task/";
	NumericGlob proc_glob(os.str(), "/status");
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


Camera::ProcList
Camera::ProcCPUAffinityMgr::getThreadList(Filter filter, 
					  CPUAffinity cpu_affinity)
{
	DEB_STATIC_FUNCT();
	DEB_PARAM() << DEB_VAR2(filter, cpu_affinity);
	return getProcList(Filter(filter | ThisProc), cpu_affinity);
}

void Camera::ProcCPUAffinityMgr::setOtherCPUAffinity(CPUAffinity cpu_affinity)
{
	DEB_MEMBER_FUNCT();

	if (!m_watchdog || m_watchdog->childEnded())
		m_watchdog = new WatchDog();

	m_watchdog->setOtherCPUAffinity(cpu_affinity);

	if (cpu_affinity.isDefault())
		m_watchdog = NULL;
}

Camera::SystemCPUAffinityMgr::
ProcessingFinishedEvent::ProcessingFinishedEvent(SystemCPUAffinityMgr *mgr)
	: m_mgr(mgr), m_cb(this), m_ct(NULL)
{
	DEB_CONSTRUCTOR();

	m_nb_frames = 0;
}

Camera::SystemCPUAffinityMgr::
ProcessingFinishedEvent::~ProcessingFinishedEvent()
{
	DEB_DESTRUCTOR();
	if (m_mgr)
		m_mgr->m_proc_finished = NULL;
}


void Camera::SystemCPUAffinityMgr::ProcessingFinishedEvent::prepareAcq()
{
	DEB_MEMBER_FUNCT();
	if (m_ct) {
		CtAcquisition *acq = m_ct->acquisition();
		acq->getAcqNbFrames(m_nb_frames);
	}
}

void Camera::SystemCPUAffinityMgr::ProcessingFinishedEvent::processingFinished()
{
	DEB_MEMBER_FUNCT();
	if (m_mgr)
		m_mgr->limaFinished();
}

void Camera::SystemCPUAffinityMgr::
ProcessingFinishedEvent::registerStatusCallback(CtControl *ct)
{
	DEB_MEMBER_FUNCT();
	if (m_ct)
		THROW_HW_ERROR(Error) << "StatusCallback already registered";

	ct->registerImageStatusCallback(m_cb);
	m_ct = ct;
}

void Camera::SystemCPUAffinityMgr::
ProcessingFinishedEvent::imageStatusChanged(
					const CtControl::ImageStatus& status)
{
	DEB_MEMBER_FUNCT();

	int last_frame = status.LastImageAcquired;
	if ((last_frame == m_nb_frames - 1) && 
	    (status.LastImageReady == last_frame))
		processingFinished();
}

Camera::SystemCPUAffinityMgr::SystemCPUAffinityMgr(Camera *cam)
	: m_cam(cam), m_proc_finished(NULL)
{
	DEB_CONSTRUCTOR();

	m_state = Ready;
}

Camera::SystemCPUAffinityMgr::~SystemCPUAffinityMgr()
{
	DEB_DESTRUCTOR();

	setLimaAffinity(CPUAffinity());

	if (m_proc_finished)
		m_proc_finished->m_mgr = NULL;
}

void Camera::SystemCPUAffinityMgr::applyAndSet(const SystemCPUAffinity& o)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR1(o);

	if (!m_cam)
		THROW_HW_ERROR(InvalidValue) << "apply without camera";

	setLimaAffinity(o.lima);
	setRecvAffinity(o.recv);

	if (!m_proc_mgr)
		m_proc_mgr = new ProcCPUAffinityMgr();

	m_proc_mgr->setOtherCPUAffinity(o.other);
	m_curr.other = o.other;

	m_set = o;
}

void Camera::SystemCPUAffinityMgr::setLimaAffinity(CPUAffinity lima_affinity)
{
	DEB_MEMBER_FUNCT();

	if (lima_affinity == m_curr.lima)
		return;

	if (m_lima_tids.size()) {
		ProcList::const_iterator it, end = m_lima_tids.end();
		for (it = m_lima_tids.begin(); it != end; ++it)
			lima_affinity.applyToTask(*it, false, false);
	} else {
		pid_t pid = getpid();
		lima_affinity.applyToTask(pid, true);
		m_curr.recv = lima_affinity;
	}
	m_curr.lima = lima_affinity;
}

void Camera::SystemCPUAffinityMgr::setRecvAffinity(CPUAffinity recv_affinity)
{
	DEB_MEMBER_FUNCT();

	if (recv_affinity == m_curr.recv)
		return;

	cpu_set_t cpu_set;
	recv_affinity.initCPUSet(cpu_set);
	Camera::RecvList& recv_list = m_cam->m_recv_list;
	for (unsigned int i = 0; i < recv_list.size(); ++i) {
		DEB_TRACE() << "setting recv " << i << " "
			     << "CPU mask to " << recv_affinity;
		slsReceiverUsers *recv = recv_list[i]->m_recv;
		recv->setThreadCPUAffinity(sizeof(cpu_set),
					   &cpu_set, &cpu_set);
	}
	for (int i = 0; i < m_cam->getTotNbPorts(); ++i) {
		pid_t tid = m_cam->m_buffer_thread[i].getTID();
		recv_affinity.applyToTask(tid, false);
	}
	m_curr.recv = recv_affinity;
}

void Camera::SystemCPUAffinityMgr::updateRecvRestart()
{
	DEB_MEMBER_FUNCT();
	m_curr.recv = m_curr.lima;
}

Camera::SystemCPUAffinityMgr::ProcessingFinishedEvent *
Camera::SystemCPUAffinityMgr::getProcessingFinishedEvent()
{
	DEB_MEMBER_FUNCT();
	if (!m_proc_finished)
		m_proc_finished = new ProcessingFinishedEvent(this);
	return m_proc_finished;
}

void Camera::SystemCPUAffinityMgr::prepareAcq()
{
	DEB_MEMBER_FUNCT();
	AutoMutex l = lock();
	if (m_state == Changing)
		THROW_HW_ERROR(Error) << "SystemCPUAffinityMgr Changing Lima";
	if (m_state != Ready)
		m_cond.wait(1);
	if (m_state != Ready)
		THROW_HW_ERROR(Error) << "SystemCPUAffinityMgr is not Ready: "
				      << "missing ProcessingFinishedEvent";

	if (m_proc_finished)
		m_proc_finished->prepareAcq();
	m_lima_tids.clear();
}

void Camera::SystemCPUAffinityMgr::startAcq()
{
	DEB_MEMBER_FUNCT();
	AutoMutex l = lock();
	m_state = Acquiring;
}

void Camera::SystemCPUAffinityMgr::stopAcq()
{
	DEB_MEMBER_FUNCT();
	if (m_proc_finished)
		limaFinished();
}

void Camera::SystemCPUAffinityMgr::recvFinished()
{
	DEB_MEMBER_FUNCT();

	AutoMutex l = lock();
	if (!m_proc_finished)
		m_state = Ready;
	if (m_state == Ready) 
		return;

	if (m_curr.lima != m_curr.recv) {
		m_state = Changing;
		AutoMutexUnlock u(l);
		ProcCPUAffinityMgr::Filter filter;
		filter = ProcCPUAffinityMgr::MatchAffinity;
		m_lima_tids = ProcCPUAffinityMgr::getThreadList(filter,
								m_curr.lima);
		DEB_ALWAYS() << "Lima TIDs: " << PrettyIntList(m_lima_tids);
		CPUAffinity lima_affinity = (uint64_t(m_curr.lima) | 
					     uint64_t(m_curr.recv));
		DEB_ALWAYS() << "Allowing Lima to run on Recv CPUs: " 
			     << lima_affinity;
		setLimaAffinity(lima_affinity);
	}

	m_state = Processing;
	m_cond.broadcast();
}

void Camera::SystemCPUAffinityMgr::limaFinished()
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

Camera::FrameType Camera::getLatestFrame(const FrameArray& l)
{
	DEB_STATIC_FUNCT();

	FrameType last_frame = !l.empty() ? l[0] : -1;
	for (unsigned int i = 1; i < l.size(); ++i)
		updateLatestFrame(last_frame, l[i]);

	DEB_RETURN() << DEB_VAR1(last_frame);
	return last_frame;
}

Camera::FrameType Camera::getOldestFrame(const FrameArray& l)
{
	DEB_STATIC_FUNCT();

	FrameType first_frame = !l.empty() ? l[0] : -1;
	for (unsigned int i = 1; i < l.size(); ++i)
		updateOldestFrame(first_frame, l[i]);

	DEB_RETURN() << DEB_VAR1(first_frame);
	return first_frame;
}

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


Camera::FrameMap::FrameMap()
	: m_nb_items(0), m_buffer_size(0)
{
	DEB_CONSTRUCTOR();
}

Camera::FrameMap::~FrameMap()
{
	DEB_DESTRUCTOR();
}

void Camera::FrameMap::setNbItems(int nb_items)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR1(nb_items);
	if (nb_items == m_nb_items)
		return;

	m_last_item_frame.resize(nb_items);
	m_nb_items = nb_items;
}

void Camera::FrameMap::setBufferSize(int buffer_size)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR1(buffer_size);
	if (buffer_size == m_buffer_size)
		return;

	m_frame_item_count.resize(buffer_size);
	m_buffer_size = buffer_size;
}

void Camera::FrameMap::clear()
{
	DEB_MEMBER_FUNCT();
	for (int i = 0; i < m_nb_items; ++i)
		m_last_item_frame[i] = -1;
	for (int i = 0; i < m_buffer_size; ++i)
		m_frame_item_count[i].set(m_nb_items);
}

void Camera::FrameMap::checkFinishedFrameItem(FrameType frame, int item)
{
	DEB_MEMBER_FUNCT();

	if (m_nb_items == 0)		
		THROW_HW_ERROR(InvalidValue) << "No items defined";
	else if (m_buffer_size == 0)
		THROW_HW_ERROR(InvalidValue) << "No buffer size defined";
	else if ((item < 0) || (item >= m_nb_items))
		THROW_HW_ERROR(InvalidValue) << DEB_VAR2(item, m_nb_items);

	FrameType &last = m_last_item_frame[item];
	if (isValidFrame(last) && (frame <= last))
		THROW_HW_ERROR(Error) << DEB_VAR1(frame) << " finished already";
}

Camera::FrameMap::FinishInfo 
Camera::FrameMap::frameItemFinished(FrameType frame, int item, bool no_check,
				    bool valid)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR4(frame, item, no_check, m_nb_items);

	if (!no_check)
		checkFinishedFrameItem(frame, item);

	FinishInfo finfo;
	FrameType &last = m_last_item_frame[item];
	finfo.first_lost = last + 1;
	finfo.nb_lost = frame - finfo.first_lost + (!valid ? 1 : 0);
	for (FrameType f = last + 1; f != (frame + 1); ++f) {
		int idx = f % m_buffer_size;
		AtomicCounter& count = m_frame_item_count[idx];
		bool frame_finished = count.dec_test_and_reset(m_nb_items);
		if (!frame_finished)
			continue;
		finfo.finished.insert(f);
	}
	last = frame;

	if (DEB_CHECK_ANY(DebTypeReturn))
		DEB_RETURN() << DEB_VAR3(finfo.first_lost, finfo.nb_lost,
					 PrettySortedList(finfo.finished));
	return finfo;
}

Camera::SimpleStat::SimpleStat(double f)
	: factor(f)
{
	reset();
}

void Camera::SimpleStat::reset()
{
	AutoMutex l(lock);
	xmin = xmax = xacc = xacc2 = 0;
	xn = 0;
}

void Camera::SimpleStat::add(double x) {
	AutoMutex l(lock);
	x *= factor;
	xmin = xn ? std::min(xmin, x) : x;
	xmax = xn ? std::max(xmax, x) : x;
	xacc += x;
	xacc2 += pow(x, 2);
	++xn;
}

Camera::SimpleStat& Camera::SimpleStat::operator =(const SimpleStat& o)
{
	if (&o == this)
		return *this;

	AutoMutex l(o.lock);
	xmin = o.xmin;
	xmax = o.xmax;
	xacc = o.xacc;
	xacc2 = o.xacc2;
	xn = o.xn;
	factor = o.factor;
	return *this;
}

int Camera::SimpleStat::n() const
{ 
	AutoMutex l(lock);
	return xn; 
}

double Camera::SimpleStat::min() const
{ 
	AutoMutex l(lock);
	return xmin;
}

double Camera::SimpleStat::max() const
{
	AutoMutex l(lock);
	return xmax; 
}

double Camera::SimpleStat::ave() const
{ 
	AutoMutex l(lock);
	return xn ? (xacc / xn) : 0; 
}

double Camera::SimpleStat::std() const
{ 
	AutoMutex l(lock);
	return xn ? sqrt(xacc2 / xn - pow(ave(), 2)) : 0; 
}

Camera::Stats::Stats()
	: cb_period(1e6), new_finish(1e6), cb_exec(1e6), recv_exec(1e6)
{}

void Camera::Stats::reset()
{
	cb_period.reset();
	new_finish.reset();
	cb_exec.reset();
	recv_exec.reset();
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
	os << "LastFinishedFrame=" << m.getLastFinishedFrame() << ", "
	   << "LastItemFrame=" << m.getLastItemFrame() << ", "
	   << "ItemFrameArray=" << m.getItemFrameArray();
	return os << ">";
}

ostream& lima::SlsDetector::operator <<(ostream& os, 
					const Camera::SortedIntList& l)
{
	return os << PrettySortedList(l);
}

ostream& lima::SlsDetector::operator <<(ostream& os, 
					const Camera::FrameArray& a)
{
	os << "[";
	for (unsigned int i = 0; i < a.size(); ++i)
		os << (i ? ", " : "") << a[i];
	return os << "]";
}

ostream& lima::SlsDetector::operator <<(ostream& os, 
					const Camera::SimpleStat& s)
{
	os << "<";
	os << "min=" << int(s.min()) << ", max=" << int(s.max()) << ", "
	   << "ave=" << int(s.ave()) << ", std=" << int(s.std()) << ", "
	   << "n=" << s.n();
	return os << ">";
}

ostream& lima::SlsDetector::operator <<(ostream& os, 
					const Camera::Stats& s)
{
	os << "<";
	os << "cb_period=" << s.cb_period << ", "
	   << "new_finish=" << s.new_finish << ", "
	   << "cb_exec=" << s.cb_exec << ", "
	   << "recv_exec=" << s.recv_exec;
	return os << ">";
}

ostream& lima::SlsDetector::operator <<(ostream& os, 
					const Camera::CPUAffinity& a)
{
	return os << hex << showbase << uint64_t(a) << dec << noshowbase;
}

ostream& lima::SlsDetector::operator <<(ostream& os, 
					const Camera::SystemCPUAffinity& a)
{
	os << "<";
	os << "recv=" << a.recv << ", lima=" << a.lima << ", other=" << a.other;
	return os << ">";
}

ostream& 
lima::SlsDetector::operator <<(ostream& os, 
			       const Camera::PixelDepthCPUAffinityMap& m)
{
	os << "[";
	bool first = true;
	Camera::PixelDepthCPUAffinityMap::const_iterator it, end = m.end();
	for (it = m.begin(); it != end; ++it, first=false)
		os << (!first ? ", " : "") << it->first << ": " << it->second;
	return os << "]";
}

Camera::Receiver::Receiver(Camera *cam, int idx, int rx_port)
	: m_cam(cam), m_idx(idx), m_rx_port(rx_port)
{
	DEB_CONSTRUCTOR();

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
	if (m_cam->m_model)
		m_cam->processRecvFileStart(m_idx, dsize);
	return 0;
}

void Camera::Receiver::portCallback(FrameType frame, int port, char *dptr, 
				    uint32_t dsize)
{
	DEB_MEMBER_FUNCT();

	if (!m_cam->m_model || (m_cam->getState() == Stopping))
		return;

	Timestamp t0 = Timestamp::now();

	int port_idx = m_cam->getPortIndex(m_idx, port);
	Timestamp& last_t0 = m_cam->m_stat_last_t0[port_idx];
	if (last_t0.isSet())
		m_cam->m_stats.cb_period.add(t0 - last_t0);
	Timestamp& last_t1 = m_cam->m_stat_last_t1[port_idx];
	if (last_t1.isSet())
		m_cam->m_stats.recv_exec.add(t0 - last_t1);
	last_t0 = t0;

	try {
		if (frame >= m_cam->m_nb_frames)
			THROW_HW_ERROR(Error) << "Invalid " 
					      << DEB_VAR2(frame, DebHex(frame));
		m_cam->processRecvPort(port_idx, frame, dptr, dsize);
	} catch (Exception& e) {
		ostringstream err_msg;
		err_msg << "Receiver::frameCallback: " << e << ": "
			<< DEB_VAR3(m_idx, frame, port);
		Event::Code err_code = Event::CamOverrun;
		Event *event = new Event(Hardware, Event::Error, Event::Camera, 
					 err_code, err_msg.str());
		DEB_EVENT(*event) << DEB_VAR1(*event);
		m_cam->reportEvent(event);
	}

	Timestamp t1 = Timestamp::now();
	m_cam->m_stats.cb_exec.add(t1 - t0);
	last_t1 = t1;
}

Camera::BufferThread::BufferThread()
	: m_cam(NULL)
{
	DEB_CONSTRUCTOR();
}

void Camera::BufferThread::init(Camera *cam, int port_idx, int size)
{
	DEB_MEMBER_FUNCT();

	m_cam = cam;
	m_port_idx = port_idx;
	m_size = size;
	m_finfo_array.resize(size);

	m_free_idx = getIndex(0);
	m_ready_idx = getIndex(-1);
	m_finish_idx = getIndex(-1);

	start();
}

Camera::BufferThread::~BufferThread()
{
	DEB_DESTRUCTOR();

	if (!hasStarted())
		return;

	AutoMutex l = lock();
	m_end = true;
	m_cond.broadcast();
}

void Camera::BufferThread::start()
{
	DEB_MEMBER_FUNCT();

	m_end = true;
	Thread::start();

	struct sched_param param;
	param.sched_priority = 50;
	int ret = pthread_setschedparam(m_thread, SCHED_RR, &param);
	if (ret != 0)
		DEB_ERROR() << "Could not set real-time priority!!";

	AutoMutex l = lock();
	while (m_end)
		m_cond.wait();
}

void Camera::BufferThread::threadFunction()
{
	DEB_MEMBER_FUNCT();

	m_tid = gettid();

	AutoMutex l = lock();

	m_end = false;
	m_cond.broadcast();
	while (!m_end) {
		while (!m_end && (m_finish_idx == m_ready_idx))
			m_cond.wait();
		if (m_finish_idx != m_ready_idx) {
			int new_idx = getIndex(m_finish_idx + 1);
			{
				AutoMutexUnlock u(l);
				FinishInfo& finfo = m_finfo_array[new_idx];
				processFinishInfo(finfo);
			}
			m_finish_idx = new_idx;
		}
	}
}

void Camera::BufferThread::processFinishInfo(FinishInfo& finfo)
{
	DEB_MEMBER_FUNCT();

	try {
		if ((finfo.nb_lost > 0) && !m_cam->m_tol_lost_packets)
			THROW_HW_ERROR(Error) << "lost frames: "
					      << "port_idx=" << m_port_idx
					      << ", first=" << finfo.first_lost
					      << ", nb=" << finfo.nb_lost;
		FrameType f = finfo.first_lost;
		for (int i = 0; i < finfo.nb_lost; ++i, ++f) {
			char *bptr = m_cam->getFrameBufferPtr(f);
			Model *model = m_cam->m_model;
			model->processRecvPort(m_port_idx, f, NULL, 0, bptr);
			AutoMutex l = m_cam->lock();
			m_cam->m_bad_frame_list.push_back(f);
		}
		SortedIntList::const_iterator it, end = finfo.finished.end();
		for (it = finfo.finished.begin(); it != end; ++it)
			m_cam->frameFinished(*it);
	} catch (Exception& e) {
		ostringstream err_msg;
		err_msg << "BufferThread::processRecvPort: " << e;
		Event::Code err_code = Event::CamOverrun;
		Event *event = new Event(Hardware, Event::Error, Event::Camera, 
					 err_code, err_msg.str());
		DEB_EVENT(*event) << DEB_VAR1(*event);
		m_cam->reportEvent(event);
	}
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
	m_cam->m_system_cpu_affinity_mgr.startAcq();
	{
		AutoMutexUnlock u(l);
		DEB_TRACE() << "calling startReceiver";
		det->startReceiver();
		DEB_TRACE() << "calling startAcquisition";
		det->startAcquisition();
	}
	m_state = Running;
	m_cond.broadcast();

	SeqFilter seq_filter;

	do {
		while ((m_state != StopReq) && m_frame_queue.empty()) {
			if (!m_cond.wait(m_cam->m_new_frame_timeout)) {
				AutoMutexUnlock u(l);
				m_cam->checkLostPackets();
			}
		}
		if (!m_frame_queue.empty()) {
			FrameType frame = m_frame_queue.front();
			m_frame_queue.pop();
			DEB_TRACE() << DEB_VAR1(frame);
			seq_filter.addVal(frame);
			SeqFilter::Range frames = seq_filter.getSeqRange();
			bool cont_acq = true;
			if (frames.nb > 0) {
				AutoMutexUnlock u(l);
				int f = frames.first;
				do {
					DEB_TRACE() << DEB_VAR1(f);
					cont_acq = newFrameReady(f);
				} while ((++f != frames.end()) && cont_acq);
			}
			if (!cont_acq)
				m_state = StopReq;
		}
	} while (m_state != StopReq);

	m_state = Stopping;
	{
		AutoMutexUnlock u(l);
		if (m_cam->getDetStatus() == Defs::Running) {
			DEB_TRACE() << "calling stopAcquisition";
			det->stopAcquisition();
			Timestamp t0 = Timestamp::now();
			while (m_cam->getDetStatus() != Defs::Idle)
				Sleep(m_cam->m_abort_sleep_time);
			double milli_sec = (Timestamp::now() - t0) * 1e3;
			DEB_TRACE() << "Abort -> Idle: " << DEB_VAR1(milli_sec);
		}
		DEB_TRACE() << "calling stopReceiver";
		det->stopReceiver();
	}

	IntList bfl = m_cam->getSortedBadFrameList();
	DEB_ALWAYS() << "bad_frames=" << bfl.size() << ": "
		     << PrettyIntList(bfl);
	DEB_ALWAYS() << DEB_VAR1(m_cam->m_stats);

	m_cam->m_system_cpu_affinity_mgr.recvFinished();

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
	  m_recv_fifo_depth(1000),
	  m_nb_frames(1),
	  m_lat_time(0),
	  m_recv_ports(0),
	  m_pixel_depth(PixelDepth16), 
	  m_image_type(Bpp16), 
	  m_raw_mode(false),
	  m_state(Idle),
	  m_new_frame_timeout(1),
	  m_abort_sleep_time(0.1),
	  m_tol_lost_packets(true),
	  m_time_ranges_cb(NULL),
	  m_system_cpu_affinity_mgr(this)
{
	DEB_CONSTRUCTOR();

	CPUAffinity::getNbCPUs();

	m_input_data = new AppInputData(config_fname);

	removeSharedMem();
	createReceivers();

	DEB_TRACE() << "Creating the multiSlsDetector object";
	m_det = new multiSlsDetector(0);
	DEB_TRACE() << "Reading configuration file";
	const char *fname = m_input_data->config_file_name.c_str();
	m_det->readConfigurationFile(fname);

	m_det->setReceiverFifoDepth(m_recv_fifo_depth);

	m_pixel_depth = PixelDepth(m_det->setDynamicRange(-1));

	setSettings(Defs::Standard);
	setTrigMode(Defs::Auto);
	setNbFrames(1);
	setExpTime(0.99);
	setFramePeriod(1.0);
}

Camera::~Camera()
{
	DEB_DESTRUCTOR();

	if (m_time_ranges_cb)
		unregisterTimeRangesChangedCallback(*m_time_ranges_cb);

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

	m_recv_ports = m_model->getRecvPorts();
	int nb_ports = getTotNbPorts();
	if (!m_buffer_thread) {
		int buffer_size = 1000;
		m_buffer_thread = new BufferThread[nb_ports];
		for (int i = 0; i < nb_ports; ++i)
			m_buffer_thread[i].init(this, i, buffer_size);
	}

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
	waitState(Idle);
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

	// Lima frame numbers are (signed) int
	const FrameType MaxFrames = INT_MAX;
	if (nb_frames >= MaxFrames)
		THROW_HW_ERROR(InvalidValue) << "too high " 
					     <<	DEB_VAR2(nb_frames, MaxFrames);

	waitState(Idle);
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
	waitState(Idle);
	m_det->setExposureTime(NSec(exp_time));
	m_exp_time = exp_time;
}

void Camera::getExpTime(double& exp_time)
{ 
	DEB_MEMBER_FUNCT();
	exp_time = m_exp_time;
	DEB_RETURN() << DEB_VAR1(exp_time);
}

void Camera::setLatTime(double lat_time)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR1(lat_time);
	m_lat_time = lat_time;
}

void Camera::getLatTime(double& lat_time)
{ 
	DEB_MEMBER_FUNCT();
	lat_time = m_lat_time;
	DEB_RETURN() << DEB_VAR1(lat_time);
}

void Camera::setFramePeriod(double frame_period)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR1(frame_period);

	if (m_model) {
		TimeRanges time_ranges;
		double e = 1e-6;
		m_model->getTimeRanges(time_ranges);
		if ((frame_period < time_ranges.min_frame_period - e) ||
		    (frame_period > time_ranges.max_frame_period + e))
			THROW_HW_ERROR(InvalidValue) 
				<< DEB_VAR3(frame_period,
					    time_ranges.min_frame_period, 
					    time_ranges.max_frame_period);
	}

	waitState(Idle);
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

void Camera::updateTimeRanges()
{
	DEB_MEMBER_FUNCT();
	TimeRanges time_ranges;
	m_model->getTimeRanges(time_ranges);
	m_exp_time = max(m_exp_time, time_ranges.min_exp_time);
	m_frame_period = max(m_frame_period, time_ranges.min_frame_period);
	DEB_TRACE() << "TimeRangesChanged: " 
		    << DEB_VAR6(time_ranges.min_exp_time, 
				time_ranges.max_exp_time,
				time_ranges.min_lat_time,
				time_ranges.max_lat_time,
				time_ranges.min_frame_period,
				time_ranges.max_frame_period);
	if (m_time_ranges_cb)
		m_time_ranges_cb->timeRangesChanged(time_ranges);
}

void Camera::updateCPUAffinity(bool recv_restarted)
{
	DEB_MEMBER_FUNCT();

	// receiver threads are restarted after DR change
	if (recv_restarted)
		m_system_cpu_affinity_mgr.updateRecvRestart();

	// apply the corresponding SystemCPUAffinity
	SystemCPUAffinity system_affinity = m_cpu_affinity_map[m_pixel_depth];
	m_system_cpu_affinity_mgr.applyAndSet(system_affinity);
}

void Camera::setPixelDepth(PixelDepth pixel_depth)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR1(pixel_depth);

	if (getState() != Idle)
		THROW_HW_FATAL(Error) << "Camera is not idle";

	waitState(Idle);
	switch (pixel_depth) {
	case PixelDepth4:
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

	if (m_model) {
		updateImageSize();
		updateTimeRanges();
		updateCPUAffinity(true);
	}
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

	bool need_period = !m_nb_frames || (m_nb_frames > 1);
	need_period &= ((m_trig_mode == Defs::Auto) || 
			(m_trig_mode == Defs::BurstTrigger));
	if (need_period && (m_lat_time > 0))
		setFramePeriod(m_exp_time + m_lat_time);

	int nb_buffers;
	m_buffer_cb_mgr->getNbBuffers(nb_buffers);
	int nb_ports = getTotNbPorts();

	{
		AutoMutex l = lock();
		m_frame_map.setNbItems(nb_ports);
		m_frame_map.setBufferSize(nb_buffers);
		m_frame_map.clear();
		DEB_TRACE() << DEB_VAR1(m_frame_queue.size());
		while (!m_frame_queue.empty())
			m_frame_queue.pop();
		m_bad_frame_list.clear();
		m_bad_frame_list.reserve(16 * 1024);
		m_stat_last_t0.assign(nb_ports, Timestamp());
		m_stat_last_t1.assign(nb_ports, Timestamp());
		m_stats.reset();
	}

	m_model->prepareAcq();
	m_system_cpu_affinity_mgr.prepareAcq();

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

	m_system_cpu_affinity_mgr.stopAcq();

	AutoMutex l = lock();
	if (getEffectiveState() != Running)
		return;

	m_acq_thread->stop(true);
	if (getEffectiveState() != Idle)
		THROW_HW_ERROR(Error) << "Camera not Idle";
}

void Camera::processRecvFileStart(int recv_idx, uint32_t dsize)
{
	DEB_MEMBER_FUNCT();
	for (int i = 0; i < m_model->getRecvPorts(); ++i) {
		int port_idx = getPortIndex(recv_idx, i);
		m_model->processRecvFileStart(port_idx, dsize);
	}
}

void Camera::processRecvPort(int port_idx, FrameType frame, char *dptr, 
			     uint32_t dsize)
{
	DEB_MEMBER_FUNCT();

	m_frame_map.checkFinishedFrameItem(frame, port_idx);
	bool valid = (dptr != NULL);
	if (valid) {
		char *bptr = getFrameBufferPtr(frame);
		m_model->processRecvPort(port_idx, frame, dptr, dsize, bptr);
	}
	FrameMap::FinishInfo finfo;
	finfo = m_frame_map.frameItemFinished(frame, port_idx, true, valid);
	if ((finfo.nb_lost == 0) && finfo.finished.empty())
		return;

	Timestamp t0 = Timestamp::now();
	BufferThread& buffer = m_buffer_thread[port_idx];
	int idx;
	BufferThread::FinishInfo *buffer_finfo;
	buffer.getNewFrameEntry(idx, buffer_finfo);
	*buffer_finfo = finfo;
	buffer.putNewFrameEntry(idx, buffer_finfo);
	Timestamp t1 = Timestamp::now();
	m_stats.new_finish.add(t1 - t0);
}

void Camera::frameFinished(FrameType frame)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR1(frame);
	AutoMutex l = lock();
	m_frame_queue.push(frame);
	m_cond.broadcast();
}

bool Camera::checkLostPackets()
{
	DEB_MEMBER_FUNCT();

	FrameArray ifa = m_frame_map.getItemFrameArray();
	FrameType last_frame = getLatestFrame(ifa);
	if (getOldestFrame(ifa) == last_frame) {
		DEB_RETURN() << DEB_VAR1(false);
		return false;
	}

	if (!m_tol_lost_packets) {
		ostringstream err_msg;
		err_msg << "checkLostPackets: frame_map=" << m_frame_map;
		Event::Code err_code = Event::CamOverrun;
		Event *event = new Event(Hardware, Event::Error, Event::Camera, 
					 err_code, err_msg.str());
		DEB_EVENT(*event) << DEB_VAR1(*event);
		reportEvent(event);
		DEB_RETURN() << DEB_VAR1(true);
		return true;
	}

	int first_bad = 0;
	if (DEB_CHECK_ANY(DebTypeWarning)) {
		AutoMutex l = lock();
		first_bad = m_bad_frame_list.size();
	}
	for (int port_idx = 0; port_idx < int(ifa.size()); ++port_idx) {
		if (ifa[port_idx] != last_frame)
			processRecvPort(port_idx, last_frame, NULL, 0);
	}
	if (DEB_CHECK_ANY(DebTypeWarning)) {
		IntList bfl;
		{
			AutoMutex l = lock();
			int last_bad = m_bad_frame_list.size();
			bfl = getSortedBadFrameList(first_bad, last_bad);
		}
		DEB_WARNING() << "bad_frames=" << bfl.size() << ": " 
			      << PrettyIntList(bfl);
	}

	DEB_RETURN() << DEB_VAR1(false);
	return false;
}

Camera::FrameType Camera::getLastReceivedFrame()
{
	DEB_MEMBER_FUNCT();
	FrameType last_frame = m_frame_map.getLastItemFrame();
	DEB_RETURN() << DEB_VAR1(last_frame);
	return last_frame;
}

int Camera::getFramesCaught()
{
	DEB_MEMBER_FUNCT();
	// recv->getTotalFramesCaught()
	int frames_caught = m_det->getFramesCaughtByReceiver();
	DEB_RETURN() << DEB_VAR1(frames_caught);
	return frames_caught;
}

Camera::DetStatus Camera::getDetStatus()
{
	DEB_MEMBER_FUNCT();
	DetStatus status = DetStatus(m_det->getRunStatus());
	DEB_RETURN() << DEB_VAR1(status);
	return status;
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
	if (m_model)
		updateTimeRanges();
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

	updateTimeRanges();
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

	if (!m_model)
		return;

	IntList aux_list;
	ReadoutFlags flags = Defs::Normal;
	if (m_model->checkReadoutFlags(flags, aux_list, true))
		addValidReadoutFlags(DEB_PTR(), flags, flag_list, 
				     flag_name_list);

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
			addValidReadoutFlags(DEB_PTR(), flags, flag_list, 
					     flag_name_list);
	}
}

void Camera::setTolerateLostPackets(bool tol_lost_packets)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR1(tol_lost_packets);
	m_tol_lost_packets = tol_lost_packets;
}

void Camera::getTolerateLostPackets(bool& tol_lost_packets)
{
	DEB_MEMBER_FUNCT();
	tol_lost_packets = m_tol_lost_packets;
	DEB_RETURN() << DEB_VAR1(tol_lost_packets);
}

Camera::IntList Camera::getSortedBadFrameList(int first_idx, int last_idx)
{
	IntList::iterator bfl_begin = m_bad_frame_list.begin();
	IntList::iterator first = bfl_begin + first_idx;
	IntList::iterator last = bfl_begin + last_idx;
	sort(first, last);
	IntList aux(last - first);
	IntList::iterator aux_end, aux_begin = aux.begin();
	aux_end = unique_copy(first, last, aux_begin);
	aux.resize(aux_end - aux_begin);
	return aux;
}

void Camera::getBadFrameList(IntList& bad_frame_list)
{
	DEB_MEMBER_FUNCT();
	{
		AutoMutex l = lock();
		bad_frame_list = getSortedBadFrameList();
	}
	DEB_RETURN() << DEB_VAR1(PrettyIntList(bad_frame_list));
}

void Camera::registerTimeRangesChangedCallback(TimeRangesChangedCallback& cb)
{
	DEB_MEMBER_FUNCT();

	if (m_time_ranges_cb)
		THROW_HW_ERROR(InvalidValue) << "a cb is already registered";

	cb.m_cam = this;
	m_time_ranges_cb = &cb;
}

void Camera::unregisterTimeRangesChangedCallback(TimeRangesChangedCallback& cb)
{
	DEB_MEMBER_FUNCT();

	if (&cb != m_time_ranges_cb)
		THROW_HW_ERROR(InvalidValue) << "the cb is not registered";

	m_time_ranges_cb = NULL;
	cb.m_cam = NULL;
}

void Camera::getStats(Stats& stats)
{
	DEB_MEMBER_FUNCT();
	stats = m_stats;
	DEB_RETURN() << DEB_VAR1(stats);
}

void Camera::setPixelDepthCPUAffinityMap(PixelDepthCPUAffinityMap aff_map)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR1(aff_map);
	m_cpu_affinity_map = aff_map;
	updateCPUAffinity(false);
}

void Camera::getPixelDepthCPUAffinityMap(PixelDepthCPUAffinityMap& aff_map)
{
	DEB_MEMBER_FUNCT();
	aff_map = m_cpu_affinity_map;
	DEB_RETURN() << DEB_VAR1(aff_map);
}

Camera::SystemCPUAffinityMgr::
ProcessingFinishedEvent *Camera::getProcessingFinishedEvent()
{
	DEB_MEMBER_FUNCT();
	return m_system_cpu_affinity_mgr.getProcessingFinishedEvent();
}
