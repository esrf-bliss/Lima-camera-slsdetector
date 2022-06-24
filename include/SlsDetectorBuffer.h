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

#ifndef __SLS_DETECTOR_BUFFER_H
#define __SLS_DETECTOR_BUFFER_H

#include "SlsDetectorCPUAffinity.h"

#include "lima/HwBufferMgr.h"

namespace lima 
{

namespace SlsDetector
{

inline Data GetMappedData(void *buffer, const FrameDim& frame_dim)
{
	Data d;
	switch (frame_dim.getImageType()) {
	case Bpp8:   d.type = Data::UINT8;  break;
	case Bpp8S:  d.type = Data::INT8;   break;
	case Bpp16:  d.type = Data::UINT16; break;
	case Bpp16S: d.type = Data::INT16;  break;
	case Bpp32:  d.type = Data::UINT32; break;
	case Bpp32S: d.type = Data::INT32;  break;
	default: throw LIMA_HW_EXC(Error, "Invalid image type");
	}
	const Size& size = frame_dim.getSize();
	d.dimensions = {size.getWidth(), size.getHeight()};
	Buffer *b = new Buffer;
	b->owner = Buffer::MAPPED;
	b->data = buffer;
	d.setBuffer(b);
	b->unref();
	return d;
}

class BufferCtrlObj : public NumaSoftBufferCtrlObj {

 public:
	void releaseBuffers() { getBuffer().releaseBuffers(); }

	Data getFrameData(FrameType frame)
	{
		StdBufferCbMgr& buffer = getBuffer();
		void *buffer_ptr = buffer.getFrameBufferPtr(frame);
		const FrameDim& frame_dim = buffer.getFrameDim();
		return GetMappedData(buffer_ptr, frame_dim);
	}
};

class BufferMgr
{
	DEB_CLASS_NAMESPC(DebModCamera, "BufferMgr", "SlsDetector");

public:
	enum ResizePolicy {
		Auto, Manual
	};

	void setBufferCtrlObj(BufferCtrlObj *buffer_ctrl_obj);

	void setBufferCPUAffinity(CPUAffinity buffer_affinity);

	bool waitFrame(FrameType frame_nb, AutoMutex& l);

	BufferCtrlObj *getBufferCtrlObj()
	{ return m_buffer_ctrl_obj; }

	StdBufferCbMgr *getBufferCbMgr()
	{
		BufferCtrlObj *buffer = getBufferCtrlObj();
		return buffer ? &buffer->getBuffer() : NULL;
	}

	void setMaxMemory(short  max_memory);
	void getMaxMemory(short& max_memory);

	void getMaxNbBuffers(long& nb_buffers);

	void setResizePolicy(ResizePolicy  resize_policy);
	void getResizePolicy(ResizePolicy& resize_policy);

	void setPacketFifoDepth(int  fifo_depth);
	void getPacketFifoDepth(int& fifo_depth);

	void prepareAcq();

	void clearAllBuffers();
	void releaseBuffers();

private:
	friend class Camera;
	typedef BufferCtrlObj::Sync BufferSync;

	BufferMgr(Camera *cam);

	Camera *m_cam;
	Cond& m_cond;
	ResizePolicy m_resize_policy;
	CPUAffinity m_buffer_affinity;
	BufferCtrlObj *m_buffer_ctrl_obj;
	BufferSync *m_buffer_sync;
	int m_max_memory;
};

} // namespace SlsDetector

} // namespace lima


#endif // __SLS_DETECTOR_BUFFER_H
