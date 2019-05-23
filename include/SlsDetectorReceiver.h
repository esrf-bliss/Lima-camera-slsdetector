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
#include "slsReceiverUsers.h"

namespace lima 
{

namespace SlsDetector
{

class Camera;

class Receiver 
{
	DEB_CLASS_NAMESPC(DebModCamera, "Receiver", "SlsDetector");

public:
	typedef slsReceiverDefs::sls_detector_header sls_detector_header;
	typedef slsReceiverDefs::sls_receiver_header sls_receiver_header;
	typedef slsReceiverDefs::thread_image_data thread_image_data;
	typedef slsReceiverDefs::receiver_image_data receiver_image_data;
	
	struct ImageData {
		receiver_image_data recv_data;
		FrameType frame;
	};

	Receiver(Camera *cam, int idx, int rx_port);
	~Receiver();

	void start();

	void prepareAcq();

	bool getImage(ImageData& image_data);
	
	SlsDetector::Stats& getStats()
	{ return m_stats.stats; }

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
	
	Camera *m_cam;
	int m_idx;
	int m_rx_port;
	Args m_args;
	AutoPtr<slsReceiverUsers> m_recv;
	Stats m_stats;
}; 


} // namespace SlsDetector

} // namespace lima


#endif // __SLS_DETECTOR_RECEIVER_H
