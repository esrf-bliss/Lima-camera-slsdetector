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

#include "SlsDetectorCPUAffinity.h"

#include "sls/sls_detector_defs.h"

#include "lima/AppPars.h"

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
	class ImagePackets {
	public:
		FrameType frame;
		int numberOfPorts;
		std::bitset<MAX_NUM_PORTS> validPortData;

		virtual ~ImagePackets() {}

		bool assemble(char *buf)
		{ return recv->asmImagePackets(this, buf); }

	protected:
		friend class Receiver;

		ImagePackets(Receiver *r) : frame(-1), numberOfPorts(0), recv(r)
		{}

		Receiver *recv;
	};

	Receiver(Camera *cam, int idx, int rx_port);
	~Receiver();

	void start();

	void setGapPixelsEnable(bool enable)
	{ m_gap_pixels_enable = enable; }

	void prepareAcq();

	void setCPUAffinity(const RecvCPUAffinity& recv_affinity);

	AutoPtr<ImagePackets> readImagePackets();

	void fillBadFrame(char *buf);

	SlsDetector::Stats& getStats()
	{ return m_stats.stats; }

	void clearAllBuffers();

private:
	friend class Camera;
	friend class ImagePackets;

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

	bool asmImagePackets(ImagePackets *image_data, char *buffer);
	AutoPtr<ImagePackets> readSkippableImagePackets();

	Camera *m_cam;
	int m_idx;
	int m_rx_port;
	AppArgs m_args;
	bool m_gap_pixels_enable;
	AutoPtr<sls::Receiver> m_recv;
	AssemblerImpl *m_asm_impl;
	Stats m_stats;
	bool m_last_skipped;
}; 

typedef std::map<int, AutoPtr<Receiver::ImagePackets>> DetImagePackets;
typedef std::pair<FrameType, DetImagePackets> DetFrameImagePackets;

struct PacketData : public sideband::Data {
	DetFrameImagePackets packets;
	PacketData(DetFrameImagePackets&& p) : packets(std::move(p)) {}
};


} // namespace SlsDetector

} // namespace lima


#endif // __SLS_DETECTOR_RECEIVER_H
