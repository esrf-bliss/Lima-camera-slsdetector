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
#ifndef __SLS_DETECTOR_RECONSTRUCTION_H
#define __SLS_DETECTOR_RECONSTRUCTION_H

#include "processlib/LinkTask.h"
#include "lima/Debug.h"
#include "lima/Exceptions.h"

#include "SlsDetectorReceiver.h"

namespace lima
{

namespace SlsDetector
{

/*******************************************************************
 * Reconstruction
 *******************************************************************/

class Reconstruction : public LinkTask
{
	DEB_CLASS_NAMESPC(DebModCamera, "Reconstruction", "SlsDetector");

public:
	class CtrlObjProxy {
		DEB_CLASS_NAMESPC(DebModCamera, "Reconstruction::CtrlObjProxy",
				  "SlsDetector");
	public:
		CtrlObjProxy(Reconstruction *r);
		virtual ~CtrlObjProxy();

		LinkTask *getReconstructionTask();

	protected:
		virtual void reconstructionChange(LinkTask *task) = 0;

	private:
		friend class Reconstruction;
		Reconstruction *m_r;
	};

	enum LimaBufferMode { RawData, CorrData };
			     
	Reconstruction(Camera *cam);
	virtual ~Reconstruction();

	void setActive(bool  active);
	void getActive(bool& active);

	void setLimaBufferMode(LimaBufferMode  lima_buffer_mode);
	void getLimaBufferMode(LimaBufferMode& lima_buffer_mode);
	
	virtual void prepare();

	Data getRawData(Data& data);

	void assemblePackets(Data& data, DetFrameImagePackets& packets);

	virtual Data process(Data& data);
	virtual Data processModel(Data& data) = 0;

	virtual void stop();

private:
	struct ThreadData {
		void *ptr;
		long size;
	};
	
	static void releaseThreadData(void *thread_data);

	friend class CtrlObjProxy;
	Camera *m_cam;
	CtrlObjProxy *m_proxy;
	bool m_active;
	LimaBufferMode m_lima_buffer_mode;
	FrameDim m_raw_frame_dim;
};

} // namespace SlsDetector

} // namespace lima

#endif // __SLS_DETECTOR_RECONSTRUCTION_H
