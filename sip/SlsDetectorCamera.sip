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
// class Camera
// {
// public:
//   typedef std::map<PixelDepth, SystemCPUAffinity> PixelDepthCPUAffinityMap;
// };
// };

%MappedType SlsDetector::Camera::PixelDepthCPUAffinityMap
{
%TypeHeaderCode
#include <map>
#include "SlsDetectorCamera.h"
#include "sipAPIlimaslsdetector.h"

using namespace std;
using namespace lima;
using namespace lima::SlsDetector;

typedef Camera::PixelDepth CPP_KeyType;
typedef Camera::SystemCPUAffinity CPP_ValueType;
typedef Camera::PixelDepthCPUAffinityMap CPP_MapType;
#define SIP_ValueType sipType_SlsDetector_Camera_SystemCPUAffinity
%End

%ConvertToTypeCode
	PyObject *key, *value;
	SIP_SSIZE_T pos;

	if (sipIsErr == NULL) {
		bool ok = PyDict_Check(sipPy);
		pos = 0;
		while (ok && PyDict_Next(sipPy, &pos, &key, &value))
			ok = (PyInt_Check(key) && 
			      sipCanConvertToType(value, SIP_ValueType, 
						  SIP_NOT_NONE));
		return ok;
	}

	*sipCppPtr = new CPP_MapType();
	pos = 0;
	while (PyDict_Next(sipPy, &pos, &key, &value)) {
		CPP_KeyType cpp_key = CPP_KeyType(PyInt_AS_LONG(key));
		void *p = sipConvertToType(value, SIP_ValueType, NULL, 
					   SIP_NOT_NONE, NULL, sipIsErr);
		CPP_ValueType *cpp_value = static_cast<CPP_ValueType *>(p);
		CPP_MapType::value_type map_value(cpp_key, *cpp_value);
		(*sipCppPtr)->insert(map_value);
	}
	return sipGetState(sipTransferObj);
%End

%ConvertFromTypeCode
	PyObject* sip_map = PyDict_New();

	CPP_MapType::iterator it, end = sipCpp->end();
	for (it = sipCpp->begin();  it != end; ++it) {
		PyObject *key = PyInt_FromLong(it->first);
		if (key == NULL)
			goto error;
		PyObject *value = sipConvertFromType(&it->second, SIP_ValueType,
						     sipTransferObj);
		if (value == NULL) {
			Py_DECREF(key);
			goto error;
		}
		PyDict_SetItem(sip_map, key, value);
		Py_DECREF(value);
		Py_DECREF(key);
	}
	return sip_map;
error:
	Py_DECREF(sip_map);
	return NULL;
%End
};


namespace SlsDetector
{

%TypeHeaderCode
#include "SlsDetectorCamera.h"
%End

class Camera : public HwMaxImageSizeCallbackGen, public EventCallbackGen
{

%TypeHeaderCode
#include "SlsDetectorCamera.h"
%End

public:
	enum State {
		Idle, Init, Starting, Running, StopReq, Stopping, Stopped,
	};

	enum Type {
		GenericDet, EigerDet, JungfrauDet,
	};

	enum PixelDepth {
		PixelDepth4 = 4, 
		PixelDepth8 = 8, 
		PixelDepth16 = 16, 
		PixelDepth32 = 32,
	};

	struct TimeRanges {
		TimeRanges();
		double min_exp_time;
		double max_exp_time;
		double min_lat_time;
		double max_lat_time;
		double min_frame_period;
		double max_frame_period;
	};

	class CPUAffinity 
	{
	public:
		CPUAffinity(unsigned long m = 0);

		static bool UseSudo;
		static int getNbCPUs(bool max_nb = false);
		static unsigned long allCPUs(bool max_nb = false);

		//void initCPUSet(cpu_set_t& cpu_set) const;
		void applyToTask(int task, bool incl_threads = true,
				 bool use_taskset = true) const;

		operator unsigned long() const;

		//CPUAffinity& operator =(unsigned long m);

		bool isDefault() const;
	};

	class ProcCPUAffinityMgr
	{
	public:
		ProcCPUAffinityMgr();
		~ProcCPUAffinityMgr();

		enum Filter {
			All, MatchAffinity, NoMatchAffinity,
		};

		static std::vector<int> getProcList(
			SlsDetector::Camera::ProcCPUAffinityMgr::Filter filter
				= SlsDetector::Camera::ProcCPUAffinityMgr::All,
			SlsDetector::Camera::CPUAffinity cpu_affinity = 0);

		void setOtherCPUAffinity(
			SlsDetector::Camera::CPUAffinity affinity);
	};

	struct SystemCPUAffinity {
		SlsDetector::Camera::CPUAffinity recv;
		SlsDetector::Camera::CPUAffinity lima;
		SlsDetector::Camera::CPUAffinity other;

		SystemCPUAffinity(SlsDetector::Camera *cam = NULL);
		
		void applyAndSet(const SlsDetector::Camera::SystemCPUAffinity& o);
	};

	//typedef std::map<SlsDetector::Camera::PixelDepth, 
	//		 SlsDetector::Camera::SystemCPUAffinity> 
	//					PixelDepthCPUAffinityMap;

	static bool isValidFrame(unsigned long frame);
	static unsigned long latestFrame(unsigned long a, unsigned long b);

	class Model
	{
	public:
		Model(SlsDetector::Camera *cam, SlsDetector::Camera::Type type);
		virtual ~Model();

		virtual void getFrameDim(FrameDim& frame_dim /Out/,
					 bool raw = false) = 0;

		SlsDetector::Camera *getCamera();
		SlsDetector::Camera::Type getType();

		virtual std::string getName() = 0;
		virtual void getPixelSize(double& x_size /Out/,
					  double& y_size /Out/) = 0;

		virtual 
		void getDACInfo(std::vector<std::string>& name_list /Out/, 
				std::vector<int>& idx_list /Out/, 
				std::vector<int>& milli_volt_list /Out/) = 0;
		virtual 
		void getADCInfo(std::vector<std::string>& name_list /Out/, 
				std::vector<int>& idx_list /Out/,
				std::vector<double>& factor_list /Out/,
				std::vector<double>& min_val_list /Out/) = 0;

		virtual void getTimeRanges(SlsDetector::Camera::TimeRanges& 
			     			time_ranges /Out/) = 0;

	protected:
		void updateCameraModel();

		virtual void updateImageSize() = 0;

		void putCmd(const std::string& s, int idx = -1);
		std::string getCmd(const std::string& s, int idx = -1);

		virtual 
		bool checkSettings(SlsDetector::Defs::Settings settings) = 0;

		virtual
		SlsDetector::Defs::ReadoutFlags getReadoutFlagsMask() = 0;
		virtual 
		bool checkReadoutFlags(
				SlsDetector::Defs::ReadoutFlags flags,
				std::vector<int>& flag_list /Out/,
				bool silent = false) = 0;

		virtual int getRecvPorts() = 0;

		virtual void prepareAcq() = 0;
		virtual void processRecvFileStart(int port_idx, 
						  unsigned int dsize) = 0;
		virtual void processRecvPort(int port_idx, unsigned long frame,
					     char *dptr, unsigned int dsize,
					     char *bptr) = 0;
	};

	class FrameMap
	{
	public:
		typedef std::set<int> List;
		typedef std::map<unsigned long, std::set<int> > Map;

		~FrameMap();

		void setNbItems(int nb_items);
		void setBufferSize(int buffer_size);
		void clear();

		unsigned long getLastItemFrame() const;
		unsigned long getLastFinishedFrame() const;

	private:
		FrameMap();
	};


	struct SimpleStat {
		double xmin;
		double xmax;
		double xacc;
		double xacc2;
		int xn;
		double factor;
		Mutex lock;

		SimpleStat(double f = 1);
		void reset();
		void add(double x);
		//SimpleStat& operator =(const SimpleStat& o);

		int n() const;
		double min() const;
		double max() const;
		double ave() const;
		double std() const;
	};

	struct Stats {
		SlsDetector::Camera::SimpleStat cb_period;
		SlsDetector::Camera::SimpleStat new_finish;
		SlsDetector::Camera::SimpleStat cb_exec;
		SlsDetector::Camera::SimpleStat recv_exec;
		Stats();
		void reset();
	};

	Camera(std::string config_fname);
	virtual ~Camera();

	SlsDetector::Camera::Type getType();
	SlsDetector::Camera::Model *getModel();
	std::vector<std::string> getHostnameList();
	int getNbDetModules();
	int getNbDetSubModules();
	int getPortIndex(int recv_idx, int port);

	void setBufferCbMgr(StdBufferCbMgr *buffer_cb_mgr);

	void setPixelDepth(SlsDetector::Camera::PixelDepth  pixel_depth);
	void getPixelDepth(SlsDetector::Camera::PixelDepth& pixel_depth /Out/);

	void setRawMode(bool  raw_mode);
	void getRawMode(bool& raw_mode /Out/);

	SlsDetector::Camera::State getState();
	void waitState(SlsDetector::Camera::State state);
	SlsDetector::Camera::State waitNotState(
				   SlsDetector::Camera::State state);

	void getFrameDim(FrameDim& frame_dim /Out/, bool raw = false);

	const SlsDetector::Camera::FrameMap& getFrameMap();

	void putCmd(const std::string& s, int idx = -1);
	std::string getCmd(const std::string& s, int idx = -1);

	int getFramesCaught();
	SlsDetector::Defs::DetStatus getDetStatus();

	void setTrigMode(SlsDetector::Defs::TrigMode  trig_mode);
	void getTrigMode(SlsDetector::Defs::TrigMode& trig_mode /Out/);
	void setNbFrames(unsigned long  nb_frames);
	void getNbFrames(unsigned long& nb_frames /Out/);
	void setExpTime(double  exp_time);
	void getExpTime(double& exp_time /Out/);
	void setLatTime(double  lat_time);
	void getLatTime(double& lat_time /Out/);
	void setFramePeriod(double  frame_period);
	void getFramePeriod(double& frame_period /Out/);

	// setDAC: sub_mod_idx: 0-N=module, -1=all
	void setDAC(int sub_mod_idx, SlsDetector::Defs::DACIndex dac_idx,
		    int  val,       bool milli_volt = false);
	void getDAC(int sub_mod_idx, SlsDetector::Defs::DACIndex dac_idx,
	     	    int& val /Out/, bool milli_volt = false);
	void getDACList(SlsDetector::Defs::DACIndex dac_idx,
			std::vector<int>& val_list /Out/,
			bool milli_volt = false);

	void getADC(int sub_mod_idx, SlsDetector::Defs::ADCIndex adc_idx,
		    int& val /Out/);
	void getADCList(SlsDetector::Defs::ADCIndex adc_idx,
		        std::vector<int>& val_list /Out/);

	void setAllTrimBits(int sub_mod_idx, int  val);
	void getAllTrimBits(int sub_mod_idx, int& val /Out/);
	void getAllTrimBitsList(std::vector<int>& val_list /Out/);

	void setSettings(SlsDetector::Defs::Settings  settings);
	void getSettings(SlsDetector::Defs::Settings& settings /Out/);
	void setThresholdEnergy(int  thres);
	void getThresholdEnergy(int& thres /Out/);

	void setClockDiv(SlsDetector::Defs::ClockDiv  clock_div);
	void getClockDiv(SlsDetector::Defs::ClockDiv& clock_div /Out/);
	void setReadoutFlags(SlsDetector::Defs::ReadoutFlags  flags);
	void getReadoutFlags(SlsDetector::Defs::ReadoutFlags& flags /Out/);
	void getValidReadoutFlags(std::vector<int>& flag_list /Out/,
	     			std::vector<std::string>& flag_name_list /Out/);

	void setTolerateLostPackets(bool  tol_lost_packets);
	void getTolerateLostPackets(bool& tol_lost_packets /Out/);
	void getBadFrameList(std::vector<int>& bad_frame_list /Out/);

	void prepareAcq();
	void startAcq();
	void stopAcq();

	void getStats(SlsDetector::Camera::Stats& stats /Out/);

	void setPixelDepthCPUAffinityMap(
		SlsDetector::Camera::PixelDepthCPUAffinityMap aff_map);
	void getPixelDepthCPUAffinityMap(
		SlsDetector::Camera::PixelDepthCPUAffinityMap& aff_map /Out/);
};


}; // namespace SlsDetector
