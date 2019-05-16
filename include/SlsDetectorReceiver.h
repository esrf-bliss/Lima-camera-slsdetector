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
#include "SlsDetectorCPUAffinity.h"
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

	Receiver(Camera *cam, int idx, int rx_port);
	~Receiver();

	void start();

	void setModelRecv(Model::Recv *model_recv);

	void prepareAcq();

	void setCPUAffinity(const RecvCPUAffinity& recv_affinity);

private:
	friend class Camera;

	class Port 
	{
		DEB_CLASS_NAMESPC(DebModCamera, "Receiver::Port", 
				  "SlsDetector");
	public:
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
	
		Port(Receiver& recv, int port);

		void prepareAcq();

		Stats& getStats()
		{ return m_stats; }

	private:
		friend class Receiver;

		void portCallback(FrameType det_frame, char *dptr, 
				  uint32_t dsize);

		Camera *m_cam;
		int m_port_idx;
		Model::Recv::Port *m_model_port;
		Stats m_stats;
	};
	typedef std::vector<AutoPtr<Port> > PortList;

	static int fileStartCallback(char *fpath, char *fname, 
				     FrameType fidx, uint32_t dsize, 
				     void *priv);
	static void portCallback(char *header, char *dptr, uint32_t dsize, 
				 void *priv);

	int fileStartCallback(char *fpath, char *fname, uint64_t fidx, 
			      uint32_t dsize);

	void getNodeMaskList(const CPUAffinityList& listener,
			     const CPUAffinityList& writer,
			     slsReceiverUsers::NodeMaskList& fifo_node_mask,
			     int& max_node);

	Camera *m_cam;
	int m_idx;
	int m_rx_port;
	Args m_args;
	AutoPtr<slsReceiverUsers> m_recv;
	Model::Recv *m_model_recv;
	PortList m_port_list;
}; 


} // namespace SlsDetector

} // namespace lima


#endif // __SLS_DETECTOR_RECEIVER_H
