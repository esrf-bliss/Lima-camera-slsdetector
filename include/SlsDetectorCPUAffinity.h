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

#include <numeric>

namespace lima 
{

namespace SlsDetector
{

class SystemCmd
{
	DEB_CLASS_NAMESPC(DebModCamera, "SystemCmd", "SlsDetector");
 public:
	SystemCmd(std::string cmd, std::string desc = "", bool try_sudo = true);
	SystemCmd(const SystemCmd& o);

	static void setUseSudo(bool use_sudo);
	static bool getUseSudo();

	std::ostream& args()
	{ return m_args; }

	void setPipes(Pipe *stdin, Pipe *stdout, Pipe *stderr);

	int execute();

 private:
	void checkSudo();

	void preparePipes();
	void restorePipes();
	bool sameOutErr()
	{ return (m_stderr == m_stdout); }

	static bool UseSudo;

	std::string m_cmd;
	std::string m_desc;
	bool m_try_sudo;
	std::ostringstream m_args;
	Pipe *m_stdin;
	Pipe *m_stdout;
	Pipe *m_stderr;
};

class SystemCmdPipe
{
	DEB_CLASS_NAMESPC(DebModCamera, "SystemCmdPipe", "SlsDetector");
 public:
	enum PipeIdx {
		StdIn, StdOut, StdErr,
	};

	enum PipeType {
		None, DoPipe,
	};

	SystemCmdPipe(std::string cmd, std::string desc = "",
		      bool try_sudo = true);
	~SystemCmdPipe();

	std::ostream& args()
	{ return m_cmd.args(); }

	void start();
	void wait();

	void setPipe(PipeIdx idx, PipeType type);
	Pipe& getPipe(PipeIdx idx);
 
 private:
	enum {
		NbPipes = StdErr + 1,
	};

	struct PipeData {
		PipeType type;
		AutoPtr<Pipe> ptr;

		PipeData(PipeType t = None) : type(t)
		{
			if (*this)
				ptr = new Pipe();
		}

		operator bool() const
		{ return (type != None); }

		void close(Pipe::EndFd end)
		{
			if (*this)
				ptr->close(end);
		}
	};
			
	typedef std::vector<PipeData> PipeList;

	PipeList m_pipe_list;
	pid_t m_child_pid;
	SystemCmd m_cmd;
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

	static int findNbSystemCPUs();
	static int findMaxNbSystemCPUs();

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

inline CPUAffinity CPUAffinityList_all(const CPUAffinityList& l)
{
	CPUAffinity all;
	if (!l.empty())
		all = std::accumulate(std::next(l.begin()), l.end(), l.front(),
				      std::bit_or<CPUAffinity>());
	return all;
}

class IrqMgr
{
	DEB_CLASS_NAMESPC(DebModCamera, "IrqMgr", "SlsDetector");
 public:
	IrqMgr(std::string net_dev = "");
	~IrqMgr();

	void setDev(std::string net_dev);

	IntList getIrqList();
	void updateRxQueueIrqAffinity(bool default_affinity);

 private:
	static bool isManaged(std::string net_dev);

	static void stopIrqBalance();
	static void restoreIrqBalance();

	static bool getIrqBalanceActive();
	static void setIrqBalanceActive(bool act);

	std::string m_net_dev;

	static bool m_irqbalance_stopped;
	static StringList m_dev_list;
};

struct NetDevRxQueueCPUAffinity {
	CPUAffinity irq;
	CPUAffinity processing;

	bool isDefault() const
	{ return irq.isDefault() && processing.isDefault(); }
	CPUAffinity all() const
	{ return irq | processing; }
};

inline
bool operator ==(const NetDevRxQueueCPUAffinity& a,
		 const NetDevRxQueueCPUAffinity& b)
{
	return ((a.irq == b.irq) && (a.processing == b.processing));
}

typedef std::map<int, NetDevRxQueueCPUAffinity> NetDevRxQueueAffinityMap;

inline
bool NetDevRxQueueAffinityMap_isDefault(const NetDevRxQueueAffinityMap& m)
{
	if (m.empty())
		return true;
	else if (m.size() != 1)
		return false;
	NetDevRxQueueAffinityMap::const_iterator it = m.begin();
	return (it->first == -1) && (it->second.isDefault());
}

class NetDevRxQueueMgr
{
	DEB_CLASS_NAMESPC(DebModCamera, "NetDevRxQueueMgr", "SlsDetector");
 public:
	typedef NetDevRxQueueCPUAffinity Affinity;
	typedef NetDevRxQueueAffinityMap AffinityMap;

	NetDevRxQueueMgr(std::string dev = "");
	~NetDevRxQueueMgr();

	void setDev(std::string dev);

	void apply(int queue, const Affinity& queue_affinity);
	void apply(const AffinityMap& affinity_map);

	IntList getRxQueueList();

 private:
	void checkDev();

	enum Task {
		Irq, Processing, NbTasks,
	};

	struct FileSetterData {
		StringList file;
		StringList setter;
	};
			
	void apply(Task task, int queue, CPUAffinity a);
	void getIrqFileSetterData(int queue,
				  FileSetterData& file_setter);
	void getProcessingFileSetterData(int queue,
					 FileSetterData& file_setter);
	bool applyWithFile(const std::string& fname, CPUAffinity a);
	bool applyWithSetter(Task task, const std::string& irq_queue,
			     CPUAffinity a);

	static std::string getSetterSudoDesc();

	std::string m_dev;
	IrqMgr m_irq_mgr;
	AffinityMap m_aff_map;

	static const std::string AffinitySetterName;
	static const StringList AffinitySetterSrc;
};

struct NetDevGroupCPUAffinity {
	StringList name_list;
	NetDevRxQueueAffinityMap queue_affinity;

	bool isDefault() const;
	CPUAffinity all() const;
};

inline bool NetDevGroupCPUAffinity::isDefault() const
{
	return NetDevRxQueueAffinityMap_isDefault(queue_affinity);
}

inline CPUAffinity NetDevGroupCPUAffinity::all() const
{
	if (isDefault())
		return CPUAffinity();
	CPUAffinityList all_queues;
	NetDevRxQueueAffinityMap::const_iterator it, end = queue_affinity.end();
	for (it = queue_affinity.begin(); it != end; ++it)
		all_queues.push_back(it->second.all());
	return CPUAffinityList_all(all_queues);
}

inline 
bool operator ==(const NetDevGroupCPUAffinity& a,
		 const NetDevGroupCPUAffinity& b)
{
	return ((a.name_list == b.name_list) &&
		(a.queue_affinity == b.queue_affinity));
}

inline 
bool operator !=(const NetDevGroupCPUAffinity& a,
		 const NetDevGroupCPUAffinity& b)
{
	return !(a == b);
}

typedef std::vector<NetDevGroupCPUAffinity> NetDevGroupCPUAffinityList;

inline CPUAffinity NetDevGroupCPUAffinityList_all(
					const NetDevGroupCPUAffinityList& l)
{
	CPUAffinityList netdev_aff_list;
	NetDevGroupCPUAffinityList::const_iterator it, end = l.end();
	for (it = l.begin(); it != end; ++it)
		netdev_aff_list.push_back(it->all());
	return CPUAffinityList_all(netdev_aff_list);
}

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

		enum {
			StringLen=128,
			AffinityMapLen=128,
		};
		typedef uint64_t Arg;
		typedef char String[StringLen];

		struct Packet {
			Cmd cmd;
			union Union {
				uint64_t proc_affinity;
				struct NetDevAffinity {
					String name_list;
					unsigned int queue_affinity_len;
					struct QueueAffinity {
						int queue;
						uint64_t irq;
						uint64_t processing;
					} queue_affinity[AffinityMapLen];
				} netdev_affinity;
			} u;

			Packet(Cmd c=Init) : cmd(c)
			{ memset(&u, 0, sizeof(u)); }
		};
		typedef NetDevRxQueueAffinityMap NetDevAffinityMap;
		typedef Packet::Union::NetDevAffinity PacketNetDevAffinity;
		typedef PacketNetDevAffinity::QueueAffinity
						PacketNetDevQueueAffinity;

		typedef std::map<std::string, NetDevRxQueueMgr> NetDevMgrMap;

		static void sigTermHandler(int signo);
		static std::string concatStringList(StringList list);
		static StringList splitStringList(std::string str);

		void childFunction();
		void procAffinitySetter(CPUAffinity cpu_affinity);

		NetDevGroupCPUAffinity netDevAffinityEncode(
							const Packet& packet);
		void netDevAffinitySetter(
				const NetDevGroupCPUAffinity& netdev_affinity);

		ProcList getOtherProcList(CPUAffinity cpu_affinity);

		void sendChildCmd(const Packet& packet);
		Packet readParentCmd();
		void ackParentCmd();

		Pipe m_cmd_pipe;
		Pipe m_res_pipe;
		pid_t m_lima_pid;
		pid_t m_child_pid;
		CPUAffinity m_other;
		NetDevMgrMap m_netdev_mgr_map;
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

inline CPUAffinity RecvCPUAffinity::all() const
{
	return (CPUAffinityList_all(listeners) | CPUAffinityList_all(writers) |
		CPUAffinityList_all(port_threads));
}


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

typedef std::vector<RecvCPUAffinity> RecvCPUAffinityList;

inline CPUAffinity RecvCPUAffinityList_all(const RecvCPUAffinityList& l)
{
	CPUAffinityList recv_aff_list;
	RecvCPUAffinityList::const_iterator it, end = l.end();
	for (it = l.begin(); it != end; ++it)
		recv_aff_list.push_back(it->all());
	return CPUAffinityList_all(recv_aff_list);
}

struct GlobalCPUAffinity {
	RecvCPUAffinityList recv;
	CPUAffinity lima;
	CPUAffinity other;
	NetDevGroupCPUAffinityList netdev;

	CPUAffinity all() const;
	void updateRecvAffinity(CPUAffinity a);
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
	void setRecvAffinity(const RecvCPUAffinityList& recv_affinity_list);

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
std::ostream& operator <<(std::ostream& os, const CPUAffinityList& l);
std::ostream& operator <<(std::ostream& os, const NetDevRxQueueCPUAffinity& a);
std::ostream& operator <<(std::ostream& os, const NetDevGroupCPUAffinity& a);
std::ostream& operator <<(std::ostream& os, const RecvCPUAffinity& a);
std::ostream& operator <<(std::ostream& os, const RecvCPUAffinityList& l);
std::ostream& operator <<(std::ostream& os, const GlobalCPUAffinity& a);
std::ostream& operator <<(std::ostream& os, const PixelDepthCPUAffinityMap& m);

} // namespace SlsDetector

} // namespace lima



#endif // __SLS_DETECTOR_CPU_AFFINITY_H
