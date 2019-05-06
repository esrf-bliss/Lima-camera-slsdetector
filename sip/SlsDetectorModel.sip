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

namespace SlsDetector
{

%TypeHeaderCode
#include "SlsDetectorModel.h"
%End

class Model
{

public:
	class Recv
	{
	public:
		class Port
		{
		public:
			virtual ~Port();
	
			virtual void processFrame(unsigned long frame, char *dptr,
						  unsigned int dsize,
						  char *bptr) = 0;
		};

		virtual ~Recv();

		virtual int getNbPorts() = 0;
		virtual SlsDetector::Model::Recv::Port *getPort(int port_idx) = 0;

		virtual void processFileStart(unsigned int dsize) = 0;

		virtual int getNbProcessingThreads() = 0;
		virtual void setNbProcessingThreads(int nb_proc_threads) = 0;
		virtual int getThreadID(int thread_idx) = 0;
	};

	Model(SlsDetector::Camera *cam, SlsDetector::Type type);
	virtual ~Model();

	virtual void getFrameDim(FrameDim& frame_dim /Out/,
				 bool raw = false) = 0;

	SlsDetector::Camera *getCamera();
	SlsDetector::Type getType();

	virtual std::string getName() = 0;
	virtual void getPixelSize(double& x_size /Out/,
				  double& y_size /Out/) = 0;

	virtual void getDACInfo(std::vector<std::string>& name_list /Out/, 
				std::vector<int>& idx_list /Out/, 
				std::vector<int>& milli_volt_list /Out/) = 0;
	virtual void getADCInfo(std::vector<std::string>& name_list /Out/, 
				std::vector<int>& idx_list /Out/,
				std::vector<double>& factor_list /Out/,
				std::vector<double>& min_val_list /Out/) = 0;

	virtual void getTimeRanges(SlsDetector::TimeRanges& 
		     		   time_ranges /Out/) = 0;

	virtual int getNbFrameMapItems() = 0;
	virtual void updateFrameMapItems(FrameMap *map) = 0;
	virtual void processBadItemFrame(unsigned long frame, int item,
					 char *bptr) = 0;

protected:
	void updateCameraModel();

	virtual void updateImageSize() = 0;

	void putCmd(const std::string& s, int idx = -1);
	std::string getCmd(const std::string& s, int idx = -1);

	virtual 
	bool checkSettings(SlsDetector::Defs::Settings settings) = 0;

	virtual int getNbRecvs() = 0;
	virtual SlsDetector::Model::Recv *getRecv(int recv_idx) = 0;

	int getNbRecvPorts();

	virtual void prepareAcq() = 0;
	virtual void startAcq() = 0;
	virtual void stopAcq() = 0;
};



}; // namespace SlsDetector
