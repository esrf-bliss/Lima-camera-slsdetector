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

#ifndef __SLS_DETECTOR_RECEIVER_H
#define __SLS_DETECTOR_RECEIVER_H

#include "SlsDetectorFrameMap.h"
#include "SlsDetectorModel.h"

#include "sls/sls_detector_defs.h"

namespace sls
{
class Receiver;
}

namespace lima 
{

namespace SlsDetector
{

class Camera;

class Receiver 
{
	DEB_CLASS_NAMESPC(DebModCamera, "Receiver", "SlsDetector");

public:
	typedef slsDetectorDefs::sls_detector_header sls_detector_header;
	typedef slsDetectorDefs::sls_receiver_header sls_receiver_header;

	struct ImageData {
		FrameType frame;
		sls_receiver_header header;
		int numberOfPorts;
		std::bitset<MAX_NUM_PORTS> validPortData;

		ImageData() : frame(-1), numberOfPorts(0)
		{ header.detHeader.frameNumber = -1; }

		virtual ~ImageData() {}

		uint64_t detFrame() { return header.detHeader.frameNumber; }
	};

	Receiver(Camera *cam, int idx, int rx_port);
	~Receiver();

	void start();

	void setGapPixelsEnable(bool enable)
	{ m_gap_pixels_enable = enable; }

	void prepareAcq();

	void setCPUAffinity(const RecvCPUAffinity& recv_affinity);

	ImageData *readImagePackets();
	bool asmImagePackets(ImageData *image_data, char *buffer);
	
	SlsDetector::Stats& getStats()
	{ return m_stats.stats; }

	void clearAllBuffers();

private:
	friend class Camera;

	struct Stats {
		SlsDetector::Stats stats;
		Timestamp last_t0;
		Timestamp last_t1;
		void reset()
		{
			stats.reset();
			last_t0 = last_t1 = Timestamp();
		}
	};

	struct AssemblerImpl;

	AutoPtr<ImageData> readSkippableImagePackets();

	Camera *m_cam;
	int m_idx;
	int m_rx_port;
	Args m_args;
	bool m_gap_pixels_enable;
	AutoPtr<sls::Receiver> m_recv;
	AssemblerImpl *m_asm_impl;
	Stats m_stats;
	bool m_last_skipped;
}; 


} // namespace SlsDetector

} // namespace lima


#endif // __SLS_DETECTOR_RECEIVER_H
