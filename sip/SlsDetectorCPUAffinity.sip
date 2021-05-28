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


// namespace SlsDetector
// {
// typedef std::vector<CPUAffinity> CPUAffinityList;
// };

%MappedType SlsDetector::CPUAffinityList
{
%TypeHeaderCode
#include "SlsDetectorCPUAffinity.h"
#include "sipAPIlimaslsdetector.h"
#include "SlsDetectorSip.h"

using namespace lima::SlsDetector;
%End

%ConvertToTypeCode
	typedef SipSequence<CPUAffinityList> Seq;
	Seq seq(sipType_SlsDetector_CPUAffinity);
	return seq.convertToTypeCode(sipPy, sipCppPtr, sipIsErr,
				     sipTransferObj);
%End

%ConvertFromTypeCode
	typedef SipSequence<CPUAffinityList> Seq;
	Seq seq(sipType_SlsDetector_CPUAffinity);
	return seq.convertFromTypeCode(sipCpp);
%End
};


// namespace SlsDetector
// {
// typedef std::vector<RecvCPUAffinity> RecvCPUAffinityList;
// };

%MappedType SlsDetector::RecvCPUAffinityList
{
%TypeHeaderCode
#include "SlsDetectorCPUAffinity.h"
#include "sipAPIlimaslsdetector.h"
#include "SlsDetectorSip.h"

using namespace lima::SlsDetector;
%End

%ConvertToTypeCode
	typedef SipSequence<RecvCPUAffinityList> Seq;
	Seq seq(sipType_SlsDetector_RecvCPUAffinity);
	return seq.convertToTypeCode(sipPy, sipCppPtr, sipIsErr,
				     sipTransferObj);
%End

%ConvertFromTypeCode
	typedef SipSequence<RecvCPUAffinityList> Seq;
	Seq seq(sipType_SlsDetector_RecvCPUAffinity);
	return seq.convertFromTypeCode(sipCpp);
%End
};


// namespace SlsDetector
// {
// typedef std::vector<SlsDetector::NetDevGroupCPUAffinity>
//					 NetDevGroupCPUAffinityList;
// };

%MappedType SlsDetector::NetDevGroupCPUAffinityList
{
%TypeHeaderCode
#include "SlsDetectorCPUAffinity.h"
#include "sipAPIlimaslsdetector.h"
#include "SlsDetectorSip.h"

using namespace lima::SlsDetector;
%End

%ConvertToTypeCode
	typedef SipSequence<NetDevGroupCPUAffinityList> Seq;
	Seq seq(sipType_SlsDetector_NetDevGroupCPUAffinity);
	return seq.convertToTypeCode(sipPy, sipCppPtr, sipIsErr,
				     sipTransferObj);
%End

%ConvertFromTypeCode
	typedef SipSequence<NetDevGroupCPUAffinityList> Seq;
	Seq seq(sipType_SlsDetector_NetDevGroupCPUAffinity);
	return seq.convertFromTypeCode(sipCpp);
%End
};


// namespace SlsDetector
// {
// typedef std::map<int, NetDevRxQueueCPUAffinity> NetDevRxQueueAffinityMap;
// };

%MappedType SlsDetector::NetDevRxQueueAffinityMap
{
%TypeHeaderCode
#include "SlsDetectorCPUAffinity.h"
#include "sipAPIlimaslsdetector.h"
#include "SlsDetectorSip.h"

using namespace lima::SlsDetector;
%End

%ConvertToTypeCode
	typedef SipMap<NetDevRxQueueAffinityMap> Map;
	Map map(NULL, sipType_SlsDetector_NetDevRxQueueCPUAffinity);
	return map.convertToTypeCode(sipPy, sipCppPtr, sipIsErr,
				     sipTransferObj);
%End

%ConvertFromTypeCode
	typedef SipMap<NetDevRxQueueAffinityMap> Map;
	Map map(NULL, sipType_SlsDetector_NetDevRxQueueCPUAffinity);
	return map.convertFromTypeCode(sipCpp);
%End
};


// namespace SlsDetector
// {
// typedef std::map<PixelDepth, GlobalCPUAffinity> PixelDepthCPUAffinityMap;
// };

%MappedType SlsDetector::PixelDepthCPUAffinityMap
{
%TypeHeaderCode
#include "SlsDetectorCPUAffinity.h"
#include "sipAPIlimaslsdetector.h"
#include "SlsDetectorSip.h"

using namespace lima::SlsDetector;

#ifndef SIP_TYPE_INT_IMPL_PixelDepth
SipTypeIntImpl(PixelDepth);
#define SIP_TYPE_INT_IMPL_PixelDepth
#endif
%End

%ConvertToTypeCode
	typedef SipMap<PixelDepthCPUAffinityMap> Map;
	Map map(sipType_SlsDetector_PixelDepth,
	        sipType_SlsDetector_GlobalCPUAffinity);
	return map.convertToTypeCode(sipPy, sipCppPtr, sipIsErr,
				     sipTransferObj);
%End

%ConvertFromTypeCode
	typedef SipMap<PixelDepthCPUAffinityMap> Map;
	Map map(sipType_SlsDetector_PixelDepth,
	        sipType_SlsDetector_GlobalCPUAffinity);
	return map.convertFromTypeCode(sipCpp);
%End
};


namespace SlsDetector
{

%TypeHeaderCode
#include "SlsDetectorCamera.h"
%End

class SystemCmd
{
public:
	SystemCmd(std::string cmd, std::string desc = "", bool try_sudo = true);

	static void setUseSudo(bool use_sudo);
	static bool getUseSudo();

	void addArgs(std::string args);
%MethodCode
	Py_BEGIN_ALLOW_THREADS
	sipCpp->args() << *a0;
	Py_END_ALLOW_THREADS
%End

	int execute();

 private:
	SystemCmd(const SlsDetector::SystemCmd& o);
};

class CPUAffinity
{
%TypeCode
	typedef ::SlsDetector::CPUAffinity::Mask CPUMask;

	PyObject *maskToPyLong(const CPUMask& mask)
	{
		std::string r = mask.to_string();
		return PyLong_FromString(r.c_str(), NULL, 2);
	}

	CPUMask pyBytesToCPUMask(PyObject *py_bytes)
	{
		char *r = PyBytes_AsString(py_bytes);
		if (!r)
			throw LIMA_HW_EXC(InvalidValue, "Invalid PyBytes");
		int base = 2;
		Py_ssize_t len = PyBytes_Size(py_bytes);
		if ((len > 2) && (strncmp(r, "0x", 2) == 0))
			base = 16;
		return ::SlsDetector::CPUAffinity::maskFromString(r, base);
	}
%End

public:
	CPUAffinity();
	CPUAffinity(SIP_PYOBJECT mask);
%MethodCode
	Py_BEGIN_ALLOW_THREADS
	try
	{
		if (!PyBytes_Check(a0))
			throw LIMA_HW_EXC(InvalidValue, "CPUAffinity accepts "
							"PyBytes objects only");
		CPUMask mask = pyBytesToCPUMask(a0);
		sipCpp = new ::SlsDetector::CPUAffinity(mask);
	}
	catch (...)
	{
		Py_BLOCK_THREADS
		sipRaiseUnknownException();
		return NULL;
	}
	Py_END_ALLOW_THREADS
%End

	static int getNbSystemCPUs(bool max_nb = false);
	static int getNbHexDigits(bool max_nb = false);
	static SIP_PYOBJECT allCPUs(bool max_nb = false);
%MethodCode
	CPUMask mask;
	Py_BEGIN_ALLOW_THREADS
	mask = ::SlsDetector::CPUAffinity::allCPUs(a0);
	Py_END_ALLOW_THREADS
	sipRes = maskToPyLong(mask);
%End

	int getNbCPUs() const;

	//void initCPUSet(cpu_set_t& cpu_set) const;
	void applyToTask(int task, bool incl_threads = true,
			 bool use_taskset = true) const;

	SIP_PYOBJECT getMask() const;
%MethodCode
	CPUMask mask;
	Py_BEGIN_ALLOW_THREADS
	mask = sipCpp->getMask();
	Py_END_ALLOW_THREADS
	sipRes = maskToPyLong(mask);
%End

	SIP_PYOBJECT getZeroDefaultMask() const;
%MethodCode
	CPUMask mask;
	Py_BEGIN_ALLOW_THREADS
	mask = sipCpp->getZeroDefaultMask();
	Py_END_ALLOW_THREADS
	sipRes = maskToPyLong(mask);
%End

	SlsDetector::CPUAffinity& operator |=(const SlsDetector::CPUAffinity& o);

	bool isDefault() const;

	void getNUMANodeMask(std::vector<unsigned long>& node_mask /Out/,
			     int& max_node /Out/);

	static std::string getProcDir(bool local_threads);
	static std::string getTaskProcDir(int task, bool is_thread);
};

// typedef std::vector<CPUAffinity> CPUAffinityList;

SlsDetector::CPUAffinity CPUAffinityList_all(
				const SlsDetector::CPUAffinityList& l);

class IrqMgr
{
 public:
	IrqMgr(std::string net_dev = "");
	~IrqMgr();

	void setDev(std::string net_dev);

	std::vector<int> getIrqList();
	void updateRxQueueIrqAffinity(bool default_affinity);
};

struct NetDevRxQueueCPUAffinity {
	SlsDetector::CPUAffinity irq;
	SlsDetector::CPUAffinity processing;

	bool isDefault() const;
	SlsDetector::CPUAffinity all() const;
};

// typedef std::map<int, NetDevRxQueueCPUAffinity> NetDevRxQueueAffinityMap;

bool NetDevRxQueueAffinityMap_isDefault(
			const SlsDetector::NetDevRxQueueAffinityMap& m);

class NetDevRxQueueMgr
{
 public:
	NetDevRxQueueMgr(std::string dev = "");
	~NetDevRxQueueMgr();

	void setDev(std::string dev);

	void apply(int queue,
		   const SlsDetector::NetDevRxQueueCPUAffinity& queue_affinity);
	void apply(const SlsDetector::NetDevRxQueueAffinityMap& affinity_map);

	std::vector<int> getRxQueueList();
};

struct NetDevGroupCPUAffinity {
	std::vector<std::string> name_list;
	SlsDetector::NetDevRxQueueAffinityMap queue_affinity;

	bool isDefault() const;
	SlsDetector::CPUAffinity all() const;
};

// typedef std::vector<SlsDetector::NetDevGroupCPUAffinity> 
//					NetDevGroupCPUAffinityList;

SlsDetector::CPUAffinity NetDevGroupCPUAffinityList_all(
			const SlsDetector::NetDevGroupCPUAffinityList& l);

class SystemCPUAffinityMgr
{
public:
	SystemCPUAffinityMgr();
	~SystemCPUAffinityMgr();

	enum Filter {
		All, MatchAffinity, NoMatchAffinity, ThisProc=0x10
	};

	static std::vector<int> getProcList(
		SlsDetector::SystemCPUAffinityMgr::Filter filter
			= SlsDetector::SystemCPUAffinityMgr::All,
		SlsDetector::CPUAffinity cpu_affinity = CPUAffinity());
	static std::vector<int> getThreadList(
		SlsDetector::SystemCPUAffinityMgr::Filter filter
			= SlsDetector::SystemCPUAffinityMgr::All,
		SlsDetector::CPUAffinity cpu_affinity = CPUAffinity());

	void setOtherCPUAffinity(
		SlsDetector::CPUAffinity affinity);
	void setNetDevCPUAffinity(
		const SlsDetector::NetDevGroupCPUAffinityList& netdev_list);
};

struct RecvCPUAffinity {
	SlsDetector::CPUAffinityList listeners;

	RecvCPUAffinity();
	SlsDetector::CPUAffinity all() const;
	// RecvCPUAffinity& operator =(CPUAffinity a);
};

// typedef std::vector<RecvCPUAffinity> RecvCPUAffinityList;

SlsDetector::CPUAffinity RecvCPUAffinityList_all(
				const SlsDetector::RecvCPUAffinityList& l);

struct GlobalCPUAffinity {
	SlsDetector::RecvCPUAffinityList recv;
	SlsDetector::CPUAffinity acq;
	SlsDetector::CPUAffinity lima;
	SlsDetector::CPUAffinity other;
	SlsDetector::NetDevGroupCPUAffinityList netdev;

	SlsDetector::CPUAffinity all() const;
};

// typedef std::map<SlsDetector::PixelDepth, SlsDetector::GlobalCPUAffinity> 
//						PixelDepthCPUAffinityMap;

class GlobalCPUAffinityMgr
{
public:
	class ProcessingFinishedEvent
	{
	public:
		ProcessingFinishedEvent(SlsDetector::GlobalCPUAffinityMgr *mgr);
		~ProcessingFinishedEvent();

		void processingFinished();

		void registerStatusCallback(CtControl *ct_control);
	};

	GlobalCPUAffinityMgr(SlsDetector::Camera *cam = NULL);
	~GlobalCPUAffinityMgr();
	
	void applyAndSet(const SlsDetector::GlobalCPUAffinity& o);
	void updateRecvRestart();

	SlsDetector::GlobalCPUAffinityMgr::
	ProcessingFinishedEvent *getProcessingFinishedEvent();

	void prepareAcq();
	void startAcq();
	void stopAcq();
	void recvFinished();
	void limaFinished();
	void waitLimaFinished();
	void cleanUp();
};


}; // namespace SlsDetector
