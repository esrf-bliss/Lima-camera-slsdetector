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
	Receiver(Camera *cam, int idx, int rx_port);
	~Receiver();

	void start();
	void setNbPorts(int nb_ports);

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

		pid_t getThreadID()
		{ return m_thread.getThreadID(); }
		
		void prepareAcq();

		void processFileStart(uint32_t dsize);
		void processFrame(FrameType frame, char *dptr, uint32_t dsize);

		bool isBadFrame(FrameType frame);
	
		int getNbBadFrames()
		{
			AutoMutex l = lock();
			return m_bad_frame_list.size();
		}
	
		void getBadFrameList(int first_idx, int last_idx, IntList& bfl)
		{
			AutoMutex l = lock();
			IntList::const_iterator b = m_bad_frame_list.begin();
			bfl.assign(b + first_idx, b + last_idx);
		}
		
		Stats& getStats()
		{ return m_stats; }

	private:
		friend class Receiver;

		typedef FrameMap::Item::FinishInfo FinishInfo;
		typedef FrameMap::Item::FinishInfoList FinishInfoList;
		typedef std::vector<FinishInfo> FinishInfoArray;
		
		class Thread : public lima::Thread
		{
			DEB_CLASS_NAMESPC(DebModCamera, 
					  "Receiver::Port::Thread", 
					  "SlsDetector");
		public:
			Thread(Port& port);
			virtual ~Thread();

			virtual void start();

		protected:
			virtual void threadFunction();

		private:
			Port& m_port;
			pid_t m_tid;
			volatile bool m_end;
		};

		AutoMutex lock()
		{ return m_mutex; }

		void pollFrameFinished();
		void stopPollFrameFinished();
		void processFinishInfo(const FinishInfo& finfo);
		
		Camera *m_cam;
		Model *m_model;
		int m_port_idx;
		Mutex m_mutex;
		FrameMap::Item *m_frame_map_item;
		IntList m_bad_frame_list;
		Stats m_stats;
		Thread m_thread;
	};
	typedef std::vector<AutoPtr<Port> > PortList;

	static int fileStartCallback(char *fpath, char *fname, 
				     FrameType fidx, uint32_t dsize, 
				     void *priv);
	static void portCallback(FrameType frame, 
				 uint32_t exp_len,
				 uint32_t recv_packets,
				 uint64_t bunch_id,
				 uint64_t timestamp,
				 uint16_t mod_id,
				 uint16_t x, uint16_t y, uint16_t z,
				 uint32_t debug,
				 uint16_t rr_nb,
				 uint8_t det_type,
				 uint8_t cb_version,
				 char *dptr, 
				 uint32_t dsize, 
				 void *priv);

	int fileStartCallback(char *fpath, char *fname, uint64_t fidx, 
			      uint32_t dsize);
	void portCallback(FrameType det_frame, int port, char *dptr, 
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
	PortList m_port_list;
}; 


} // namespace SlsDetector

} // namespace lima


#endif // __SLS_DETECTOR_RECEIVER_H
