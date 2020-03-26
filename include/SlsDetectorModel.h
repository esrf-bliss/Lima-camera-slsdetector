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

#ifndef __SLS_DETECTOR_MODEL_H
#define __SLS_DETECTOR_MODEL_H

#include "SlsDetectorDefs.h"
#include "SlsDetectorCPUAffinity.h"

#include "lima/SizeUtils.h"

#include "slsDetectorUsers.h"

namespace lima 
{

namespace SlsDetector
{

class Camera;
class Receiver;

class Model
{
	DEB_CLASS_NAMESPC(DebModCamera, "Model", "SlsDetector");
 public:
	typedef Defs::Settings Settings;

	typedef FrameMap::FinishInfo FinishInfo;

	class Recv
	{
		DEB_CLASS_NAMESPC(DebModCamera, "Model::Recv", "SlsDetector");
	public:
		virtual ~Recv();

		virtual int getNbProcessingThreads() = 0;
		virtual void setNbProcessingThreads(int nb_proc_threads) = 0;

		virtual void setCPUAffinity(const RecvCPUAffinity&
							recv_affinity) = 0;
	};

	Model(Camera *cam, Type type);
	virtual ~Model();
	
	virtual void getFrameDim(FrameDim& frame_dim, bool raw = false) = 0;
		
	Camera *getCamera()
	{ return m_cam; }

	Type getType()
	{ return m_type; }

	int getNbDetModules()
	{ return m_nb_det_modules; }

	int getNbDetSubModules()
	{ return m_nb_det_submodules; }

	virtual std::string getName() = 0;
	virtual void getPixelSize(double& x_size, double& y_size) = 0;

	virtual void getDACInfo(NameList& name_list, IntList& idx_list,
				IntList& milli_volt_list) = 0;
	virtual void getADCInfo(NameList& name_list, IntList& idx_list,
				FloatList& factor_list, 
				FloatList& min_val_list) = 0;

	virtual void getTimeRanges(TimeRanges& time_ranges) = 0;

	virtual int getNbFrameMapItems() = 0;
	virtual void updateFrameMapItems(FrameMap *map) = 0;
	virtual void processBadItemFrame(FrameType frame, int item,
					 char *bptr) = 0;

 protected:
	void updateCameraModel();
	void updateTimeRanges();

	virtual void updateImageSize() = 0;

	void putCmd(const std::string& s, int idx = -1);
	std::string getCmd(const std::string& s, int idx = -1);

	char *getFrameBufferPtr(FrameType frame_nb);

	virtual bool checkSettings(Settings settings) = 0;

	virtual int getNbRecvs() = 0;
	virtual Recv *getRecv(int recv_idx) = 0;

	virtual void prepareAcq() = 0;
	virtual void startAcq() = 0;
	virtual void stopAcq() = 0;

	void processFinishInfo(const FinishInfo& finfo);

 private:
	friend class Camera;
	friend class Receiver;

	Camera *m_cam;
	Type m_type;
	int m_nb_det_modules;
	int m_nb_det_submodules;

 protected:
	AutoPtr<slsDetectorUsers> m_det;
};


} // namespace SlsDetector

} // namespace lima



#endif // __SLS_DETECTOR_MODEL_H
