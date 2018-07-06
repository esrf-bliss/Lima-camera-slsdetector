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

class SystemCmd
{
	DEB_CLASS_NAMESPC(DebModCamera, "SystemCmd", "SlsDetector");
 public:
	SystemCmd(std::string cmd, std::string desc = "",
		  bool try_sudo = true, bool can_hide_out = true);
	SystemCmd(const SystemCmd& o);

	static void setUseSudo(bool use_sudo);
	static bool getUseSudo();

	std::ostream& args()
	{ return m_args; }

	int execute();

 private:
	void checkSudo();

	static bool UseSudo;

	std::string m_cmd;
	std::string m_desc;
	bool m_try_sudo;
	bool m_can_hide_out;
	std::ostringstream m_args;
};

class CPUAffinity 
{
	DEB_CLASS_NAMESPC(DebModCamera, "CPUAffinity", "SlsDetector");
 public:
	CPUAffinity(uint64_t m = 0) : m_mask(internalMask(m))
	{}

	static int getNbSystemCPUs(bool max_nb = false);

	static int getNbHexDigits(bool max_nb = false)
	{ return getNbSystemCPUs(max_nb) / 4; }

	static uint64_t allCPUs(bool max_nb = false)
	{ return (uint64_t(1) << getNbSystemCPUs(max_nb)) - 1; }

	int getNbCPUs() const
	{ return m_mask.any() ? m_mask.count() : getNbSystemCPUs(); }

	void initCPUSet(cpu_set_t& cpu_set) const;
	void applyToTask(pid_t task, bool incl_threads = true,
			 bool use_taskset = true) const;
	void applyToNetDev(std::string dev) const;
	void applyToNetDevGroup(StringList dev_list) const;

	uint64_t getMask() const
	{ return m_mask.any() ? m_mask.to_ulong() : allCPUs(); }

	uint64_t getZeroDefaultMask() const
	{ return m_mask.to_ulong(); }

	operator uint64_t() const
	{ return getMask(); }

	CPUAffinity& operator |=(const CPUAffinity& o);

	bool isDefault() const
	{ return m_mask.none() || (m_mask.to_ulong() == allCPUs()); }

	void getNUMANodeMask(std::vector<unsigned long>& node_mask,
			     int& max_node);

	static std::string getProcDir(bool local_threads);
	static std::string getTaskProcDir(pid_t task, bool is_thread);

 private:
	static uint64_t internalMask(uint64_t m)
	{ return (m != allCPUs()) ? m : 0; }

	void applyWithTaskset(pid_t task, bool incl_threads) const;
	void applyWithSetAffinity(pid_t task, bool incl_threads) const;
	bool applyWithNetDevFile(const std::string& fname) const;
	bool applyWithNetDevSetter(const std::string& dev, 
				   const std::string& queue) const;

	static int findNbSystemCPUs();
	static int findMaxNbSystemCPUs();
	static std::string getNetDevSetterSudoDesc();

	static const std::string NetDevSetQueueRpsName;
	static const StringList NetDevSetQueueRpsSrc;

	std::bitset<64> m_mask;
};

inline
bool operator ==(const CPUAffinity& a, const CPUAffinity& b)
{
	uint64_t mask = CPUAffinity::allCPUs();
	return (a.getMask() & mask) == (b.getMask() & mask);
}

inline
bool operator !=(const CPUAffinity& a, const CPUAffinity& b)
{
	return !(a == b);
}

inline
CPUAffinity operator |(const CPUAffinity& a, const CPUAffinity& b)
{
	if (a.isDefault() || b.isDefault())
		return CPUAffinity();
	return CPUAffinity(a.getMask() | b.getMask());
}

inline
CPUAffinity& CPUAffinity::operator |=(const CPUAffinity& o)
{
	return *this = *this | o;
}

typedef std::vector<CPUAffinity> CPUAffinityList;


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


class SystemCPUAffinityMgr
{
	DEB_CLASS_NAMESPC(DebModCamera, "SystemCPUAffinityMgr", "SlsDetector");
 public:
	enum Filter {
		All, MatchAffinity, NoMatchAffinity, ThisProc=0x10,
	};

	SystemCPUAffinityMgr();
	~SystemCPUAffinityMgr();

	static ProcList getProcList(Filter filter = All, 
				    CPUAffinity cpu_affinity = 0);
	static ProcList getThreadList(Filter filter = All, 
				      CPUAffinity cpu_affinity = 0);

	void setOtherCPUAffinity(CPUAffinity cpu_affinity);
	void setNetDevCPUAffinity(
			const NetDevGroupCPUAffinityList& netdev_list);

 private:
	class WatchDog
	{
		DEB_CLASS_NAMESPC(DebModCamera, "WatchDog", 
				  "SlsDetector::SystemCPUAffinityMgr");
	public:
		WatchDog();
		~WatchDog();

		bool childEnded();
		void setOtherCPUAffinity(CPUAffinity cpu_affinity);
		void setNetDevCPUAffinity(NetDevGroupCPUAffinity netdev_affinity);

	private:
		enum Cmd {
			Init, SetProcAffinity, SetNetDevAffinity, CleanUp, Ok,
		};

		typedef uint64_t Arg;

		struct Packet {
			typedef char String[128];

			Cmd cmd;
			Arg arg;
			String str;
		};
			
		static void sigTermHandler(int signo);
		static std::string concatStringList(StringList list);
		static StringList splitStringList(std::string str);

		void childFunction();
		void procAffinitySetter(CPUAffinity cpu_affinity);
		void netDevAffinitySetter(
				  NetDevGroupCPUAffinity netdev_affinity);

		ProcList getOtherProcList(CPUAffinity cpu_affinity);

		void sendChildCmd(Cmd cmd, Arg arg = 0, std::string str = "");
		Packet readParentCmd();
		void ackParentCmd();

		Pipe m_cmd_pipe;
		Pipe m_res_pipe;
		pid_t m_lima_pid;
		pid_t m_child_pid;
		CPUAffinity m_other;
		StringList m_netdev_list;
	};

	void checkWatchDogStart();
	void checkWatchDogStop();

	AutoPtr<WatchDog> m_watchdog;
	CPUAffinity m_other;
	NetDevGroupCPUAffinityList m_netdev;
};

struct RecvCPUAffinity {
	CPUAffinityList listeners;
	CPUAffinityList writers;
	CPUAffinityList port_threads;

	RecvCPUAffinity();
	CPUAffinity all() const;
	RecvCPUAffinity& operator =(CPUAffinity a);
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


struct GlobalCPUAffinity {
	RecvCPUAffinity recv;
	CPUAffinity lima;
	CPUAffinity other;
	NetDevGroupCPUAffinityList netdev;
	CPUAffinity all() const;
};

typedef std::map<PixelDepth, GlobalCPUAffinity> PixelDepthCPUAffinityMap;

class GlobalCPUAffinityMgr 
{
	DEB_CLASS_NAMESPC(DebModCamera, "GlobalCPUAffinityMgr", 
			  "SlsDetector");
 public:
	class ProcessingFinishedEvent
	{
		DEB_CLASS_NAMESPC(DebModCamera, "ProcessingFinishedEvent", 
				  "SlsDetector::GlobalCPUAffinityMgr");
	public:
		ProcessingFinishedEvent(GlobalCPUAffinityMgr *mgr);
		~ProcessingFinishedEvent();

		void processingFinished();

		void registerStatusCallback(CtControl *ct_control);

	private:
		friend class GlobalCPUAffinityMgr;

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

		GlobalCPUAffinityMgr *m_mgr;
		ImageStatusCallback m_cb;
		CtControl *m_ct;
		int m_nb_frames;
		bool m_cnt_act;
		bool m_saving_act;
		bool m_stopped;
		Timestamp m_last_cb_ts;
	};

	GlobalCPUAffinityMgr(Camera *cam = NULL);
	~GlobalCPUAffinityMgr();

	void applyAndSet(const GlobalCPUAffinity& o);
	void updateRecvRestart();

	ProcessingFinishedEvent *getProcessingFinishedEvent();

	void prepareAcq();
	void startAcq();
	void stopAcq();
	void recvFinished();
	void limaFinished();
	void waitLimaFinished();
	void cleanUp();

 private:
	friend class ProcessingFinishedEvent;

	enum State {
		Ready, Acquiring, Changing, Processing, Restoring,
	};

	void setLimaAffinity(CPUAffinity lima_affinity);
	void setRecvAffinity(const RecvCPUAffinity& recv_affinity);

	AutoMutex lock()
	{ return AutoMutex(m_cond.mutex()); }

	Camera *m_cam;
	ProcList m_lima_tids;
	GlobalCPUAffinity m_curr;
	GlobalCPUAffinity m_set;
	AutoPtr<SystemCPUAffinityMgr> m_system_mgr;
	Cond m_cond;
	State m_state;
	ProcessingFinishedEvent *m_proc_finished;
	double m_lima_finished_timeout;
};

std::ostream& operator <<(std::ostream& os, const CPUAffinity& a);
std::ostream& operator <<(std::ostream& os, const CPUAffinityList& a);
std::ostream& operator <<(std::ostream& os, const RecvCPUAffinity& a);
std::ostream& operator <<(std::ostream& os, const GlobalCPUAffinity& a);
std::ostream& operator <<(std::ostream& os, const PixelDepthCPUAffinityMap& m);

} // namespace SlsDetector

} // namespace lima



#endif // __SLS_DETECTOR_CPU_AFFINITY_H
