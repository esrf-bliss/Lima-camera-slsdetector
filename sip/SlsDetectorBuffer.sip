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
#include "SlsDetectorBuffer.h"
%End

class BufferMgr
{
public:
	enum ResizePolicy {
		Auto, Manual, Max,
	};

	void setMaxMemory(short  max_memory);
	void getMaxMemory(short& max_memory /Out/);

	void getMaxNbBuffers(long& nb_buffers /Out/);

	void setResizePolicy(
		SlsDetector::BufferMgr::ResizePolicy  resize_policy);
	void getResizePolicy(
		SlsDetector::BufferMgr::ResizePolicy& resize_policy /Out/);

	void setPacketFifoDepth(int  fifo_depth);
	void getPacketFifoDepth(int& fifo_depth);

	void clearAllBuffers();
	void releaseBuffers();

private:
	BufferMgr(Camera *cam);
	BufferMgr(const BufferMgr& o);
};


}; // namespace SlsDetector
