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

#ifndef __SLS_DETECTOR_CPU_AFFINITY_H
#define __SLS_DETECTOR_CPU_AFFINITY_H

#include "SlsDetectorDefs.h"

#include "lima/CtControl.h"
#include "lima/SimplePipe.h"

namespace lima 
{

namespace SlsDetector
{

class CPUAffinity 
{
	DEB_CLASS_NAMESPC(DebModCamera, "CPUAffinity", "SlsDetector");
 public:
 	CPUAffinity(uint64_t m = 0) : m_mask(m) 
	{}

	static void setUseSudo(bool use_sudo);
	static bool getUseSudo();

	static void checkSudo(std::string cmd, std::string desc = "");
	static int getNbCPUs(bool max_nb = false);

	static uint64_t allCPUs(bool max_nb = false)
	{ return (uint64_t(1) << getNbCPUs(max_nb)) - 1; }

	void initCPUSet(cpu_set_t& cpu_set) const;
	void applyToTask(pid_t task, bool incl_threads = true,
			 bool use_taskset = true) const;
	void applyToNetDev(std::string dev) const;
	void applyToNetDevGroup(StringList dev_list) const;

	operator uint64_t() const
	{ return m_mask ? m_mask : allCPUs(); }

	CPUAffinity& operator =(uint64_t m)
		{ m_mask = m; return *this; }

	bool isDefault() const
	{ return !m_mask || (m_mask == allCPUs()); }

	static std::string getProcDir(bool local_threads);
	static std::string getTaskProcDir(pid_t task, bool is_thread);

 private:
	void applyWithTaskset(pid_t task, bool incl_threads) const;
	void applyWithSetAffinity(pid_t task, bool incl_threads) const;
	bool applyWithNetDevFile(const std::string& fname) const;
	bool applyWithNetDevSetter(const std::string& dev, 
				   const std::string& queue) const;

	static bool UseSudo;
	static int findNbCPUs();
	static int findMaxNbCPUs();
	static std::string getNetDevSetterSudoDesc();

	static const std::string NetDevSetQueueRpsName;
	static const StringList NetDevSetQueueRpsSrc;

	uint64_t m_mask;
};

inline
bool operator ==(const CPUAffinity& a, const CPUAffinity& b)
{
	uint64_t mask = CPUAffinity::allCPUs();
	return (uint64_t(a) & mask) == (uint64_t(b) & mask);
}

inline
bool operator !=(const CPUAffinity& a, const CPUAffinity& b)
{
	return !(a == b);
}


class ProcCPUAffinityMgr
{
	DEB_CLASS_NAMESPC(DebModCamera, "ProcCPUAffinityMgr", "SlsDetector");
 public:
	enum Filter {
		All, MatchAffinity, NoMatchAffinity, ThisProc=0x10,
	};

	ProcCPUAffinityMgr();
	~ProcCPUAffinityMgr();

	static ProcList getProcList(Filter filter = All, 
				    CPUAffinity cpu_affinity = 0);
	static ProcList getThreadList(Filter filter = All, 
				      CPUAffinity cpu_affinity = 0);

	void setOtherCPUAffinity(CPUAffinity cpu_affinity);

 private:
	class WatchDog
	{
		DEB_CLASS_NAMESPC(DebModCamera, "WatchDog", 
				  "SlsDetector::ProcCPUAffinityMgr");
	public:
		WatchDog();
		~WatchDog();

		bool childEnded();
		void setOtherCPUAffinity(CPUAffinity cpu_affinity);

	private:
		enum Cmd {
			Init, SetAffinity, CleanUp, Ok,
		};

		typedef uint64_t Arg;

		struct Packet {
			Cmd cmd;
			Arg arg;
		};
			
		static void sigTermHandler(int signo);

		void childFunction();
		void affinitySetter(CPUAffinity cpu_affinity);

		ProcList getOtherProcList(CPUAffinity cpu_affinity);

		void sendChildCmd(Cmd cmd, Arg arg = 0);
		Packet readParentCmd();
		void ackParentCmd();

		Pipe m_cmd_pipe;
		Pipe m_res_pipe;
		pid_t m_lima_pid;
		pid_t m_child_pid;
	};

	AutoPtr<WatchDog> m_watchdog;
};

struct RecvCPUAffinity {
	CPUAffinity listeners;
	CPUAffinity writers;

	CPUAffinity all() const
	{
		return listeners | writers;
	}

	RecvCPUAffinity& operator =(CPUAffinity a)
	{
		listeners = writers = a;
		return *this;
	}
};

inline 
bool operator ==(const RecvCPUAffinity& a, const RecvCPUAffinity& b)
{
	return ((a.listeners == b.listeners) && (a.writers == b.writers));
}

inline 
bool operator !=(const RecvCPUAffinity& a, const RecvCPUAffinity& b)
{
	return !(a == b);
}


struct NetDevGroupCPUAffinity {
	StringList name_list;
	CPUAffinity processing;
};

inline 
bool operator ==(const NetDevGroupCPUAffinity& a, 
		 const NetDevGroupCPUAffinity& b)
{
	return ((a.name_list == b.name_list) && (a.processing == b.processing));
}

inline 
bool operator !=(const NetDevGroupCPUAffinity& a, 
		 const NetDevGroupCPUAffinity& b)
{
	return !(a == b);
}

typedef std::vector<NetDevGroupCPUAffinity> NetDevGroupCPUAffinityList;


struct SystemCPUAffinity {
	RecvCPUAffinity recv;
	CPUAffinity lima;
	CPUAffinity other;
	NetDevGroupCPUAffinityList netdev;
};

typedef std::map<PixelDepth, SystemCPUAffinity> PixelDepthCPUAffinityMap;

class SystemCPUAffinityMgr 
{
	DEB_CLASS_NAMESPC(DebModCamera, "SystemCPUAffinityMgr", 
			  "SlsDetector");
 public:
	class ProcessingFinishedEvent
	{
		DEB_CLASS_NAMESPC(DebModCamera, "ProcessingFinishedEvent", 
				  "SlsDetector::SystemCPUAffinityMgr");
	public:
		ProcessingFinishedEvent(SystemCPUAffinityMgr *mgr);
		~ProcessingFinishedEvent();

		void processingFinished();

		void registerStatusCallback(CtControl *ct_control);

	private:
		friend class SystemCPUAffinityMgr;

		class ImageStatusCallback : 
		public CtControl::ImageStatusCallback
			{
			public:
			ImageStatusCallback(
				ProcessingFinishedEvent *proc_finished)
				: m_proc_finished(proc_finished) 
				{}
			protected:
				virtual void imageStatusChanged(
					const CtControl::ImageStatus& status)
				{ m_proc_finished->imageStatusChanged(status); }
			private:
				ProcessingFinishedEvent *m_proc_finished;
			};

		void prepareAcq();
		void stopAcq();

		void limitUpdateRate();
		void updateLastCallbackTimestamp();
		Timestamp getLastCallbackTimestamp();

		void imageStatusChanged(const CtControl::ImageStatus& status);

		SystemCPUAffinityMgr *m_mgr;
		ImageStatusCallback m_cb;
		CtControl *m_ct;
		int m_nb_frames;
		bool m_cnt_act;
		bool m_saving_act;
		bool m_stopped;
		Timestamp m_last_cb_ts;
	};

	SystemCPUAffinityMgr(Camera *cam = NULL);
	~SystemCPUAffinityMgr();

	void applyAndSet(const SystemCPUAffinity& o);
	void updateRecvRestart();

	ProcessingFinishedEvent *getProcessingFinishedEvent();

	void prepareAcq();
	void startAcq();
	void stopAcq();
	void recvFinished();
	void limaFinished();
	void waitLimaFinished();

 private:
	friend class ProcessingFinishedEvent;

	enum State {
		Ready, Acquiring, Changing, Processing, Restoring,
	};

	void setLimaAffinity(CPUAffinity lima_affinity);
	void setRecvAffinity(const RecvCPUAffinity& recv_affinity);
	void setNetDevAffinity(const NetDevGroupCPUAffinityList& netdev_list);

	AutoMutex lock()
	{ return AutoMutex(m_cond.mutex()); }

	Camera *m_cam;
	ProcList m_lima_tids;
	SystemCPUAffinity m_curr;
	SystemCPUAffinity m_set;
	AutoPtr<ProcCPUAffinityMgr> m_proc_mgr;
	Cond m_cond;
	State m_state;
	ProcessingFinishedEvent *m_proc_finished;
	double m_lima_finished_timeout;
};

std::ostream& operator <<(std::ostream& os, const CPUAffinity& a);
std::ostream& operator <<(std::ostream& os, const RecvCPUAffinity& a);
std::ostream& operator <<(std::ostream& os, const SystemCPUAffinity& a);
std::ostream& operator <<(std::ostream& os, const PixelDepthCPUAffinityMap& m);

} // namespace SlsDetector

} // namespace lima



#endif // __SLS_DETECTOR_CPU_AFFINITY_H
