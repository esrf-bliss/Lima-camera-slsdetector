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

enum BufferType {
	AcqBuffer, LimaBuffer
};

class BufferCtrlObj : public NumaSoftBufferCtrlObj {

 public:
	BufferCtrlObj() : m_type(AcqBuffer) {}

	void releaseBuffers() { getBuffer().releaseBuffers(); }

	void setType(BufferType type)
	{
		if (type == m_type)
			return;
		releaseBuffers();
		m_type = type;
	}

	virtual void getMaxNbBuffers(int& max_nb_buffers)
	{
		if (m_type == LimaBuffer)
			max_nb_buffers = 1024;
		else
			NumaSoftBufferCtrlObj::getMaxNbBuffers(max_nb_buffers);
	}

	Data getFrameData(FrameType frame)
	{
		Data d;
		StdBufferCbMgr& buffer = getBuffer();
		const FrameDim& frame_dim = buffer.getFrameDim();
		switch (frame_dim.getImageType()) {
		case Bpp8:  d.type = Data::UINT8;  break;
		case Bpp16: d.type = Data::UINT16; break;
		case Bpp32: d.type = Data::UINT32; break;
		default: throw LIMA_HW_EXC(Error, "Invalid image type");
		}
		const Size& size = frame_dim.getSize();
		d.dimensions = {size.getWidth(), size.getHeight()};
		Buffer *b = new Buffer;
		b->owner = Buffer::MAPPED;
		b->data = buffer.getFrameBufferPtr(frame);;
		d.setBuffer(b);
		b->unref();
		return d;
	}

 private:
	BufferType m_type;
};

class BufferMgr
{
	DEB_CLASS_NAMESPC(DebModCamera, "BufferMgr", "SlsDetector");

public:
	enum Mode { Single, Dual };

	void setLimaBufferCtrlObj(BufferCtrlObj *buffer_ctrl_obj);

	void setAcqBufferCPUAffinity(CPUAffinity buffer_affinity);

	void waitLimaFrame(FrameType frame_nb, AutoMutex& l);
	char *getAcqFrameBufferPtr(FrameType frame_nb);

	BufferCtrlObj *getBufferCtrlObj(BufferType type)
	{
		bool lima = ((type == LimaBuffer) || (m_mode == Single));
		return lima ? m_lima_buffer_ctrl_obj :
			      m_acq_buffer_ctrl_obj.getPtr();
	}

	StdBufferCbMgr *getBufferCbMgr(BufferType type)
	{
		BufferCtrlObj *buffer = getBufferCtrlObj(type);
		return buffer ? &buffer->getBuffer() : NULL;
	}

	void setMaxMemory(short  max_memory);
	void getMaxMemory(short& max_memory);

	void getMaxNbBuffers(long& nb_buffers);

	void setMode(Mode  mode);
	void getMode(Mode& mode);

	void prepareAcq();

	void clearAllBuffers();
	void releaseBuffers();

private:
	friend class Camera;
	typedef BufferCtrlObj::Sync BufferSync;

	BufferMgr(Camera *cam);

	Camera *m_cam;
	Cond& m_cond;
	Mode m_mode;
	CPUAffinity m_buffer_affinity;
	BufferCtrlObj *m_lima_buffer_ctrl_obj;
	BufferSync *m_lima_buffer_sync;
	AutoPtr<BufferCtrlObj> m_acq_buffer_ctrl_obj;
	int m_max_memory;
};

} // namespace SlsDetector

} // namespace lima


#endif // __SLS_DETECTOR_BUFFER_H
