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

#ifndef __SLS_DETECTOR_FRAME_MAP_H
#define __SLS_DETECTOR_FRAME_MAP_H

#include "SlsDetectorDefs.h"

namespace lima 
{

namespace SlsDetector
{

class FrameMap
{
	DEB_CLASS_NAMESPC(DebModCamera, "FrameMap", "SlsDetector");

 public:
	class Item
	{
		DEB_CLASS_NAMESPC(DebModCamera, "Item", "SlsDetector");
	
	public:
		typedef std::pair<int, bool> FrameData;
		typedef std::vector<FrameData> FrameDataList;

		struct FinishInfo {
			FrameType first_lost;
			int nb_lost;
			SortedIntList finished;
		};
		typedef std::vector<FinishInfo> FinishInfoList;
	
		Item();
		~Item();
	
		void checkFinishedFrame(FrameType frame);
		void frameFinished(FrameType frame, bool no_check, bool valid);
		FrameDataList pollFrameFinished();
		FinishInfoList getFrameFinishInfo(const FrameDataList&
								  data_list);
		void stopPollFrameFinished();
	
	private:
		friend class FrameMap;
	
		class FrameQueue 
		{
		public:
			FrameQueue(int size = 1000);
			void clear();
			void push(FrameData data);
			FrameDataList pop_all();
			void stop();
	
		private:
			int index(int i)
			{ return i % m_size; }
	
			FrameDataList m_array;
			int m_size;
			volatile int m_write_idx;
			volatile int m_read_idx;
			volatile bool m_stopped;
		};
	
		friend bool SlsDetector::operator <(FrameData a, FrameData b);
	
		void setFrameMap(FrameMap *map);
		void clear();
	
		FrameMap *m_map;
		FrameQueue m_frame_queue;
		FrameType m_last_pushed_frame;
		FrameType m_last_frame;
	};
	typedef std::vector<Item> ItemList;

	
	FrameMap();
		
	void setNbItems(int nb_items);
	void setBufferSize(int buffer_size);
	void clear();

	Item& getItem(int item)
	{ return m_item_list[item]; }

	FrameArray getItemFrameArray() const;

	FrameType getLastItemFrame() const
	{ return getLatestFrame(getItemFrameArray()); }

	FrameType getLastFinishedFrame() const
	{ return getOldestFrame(getItemFrameArray()); }

 private:
	friend class Item;

	struct AtomicCounter {
		int count;
		Mutex mutex;
	
		void set(int reset)
		{ count = reset; }
	
		bool dec_test_and_reset(int reset)
		{
			mutex.lock();
			bool zero = (--count == 0);
			if (zero)
				set(reset);
			mutex.unlock();
			return zero;
		}
	};
	typedef std::vector<AtomicCounter> CounterList;
	
	int m_nb_items;
	int m_buffer_size;
	CounterList m_frame_item_count_list;
	ItemList m_item_list;
};

std::ostream& operator <<(std::ostream& os, const FrameMap& m);

} // namespace SlsDetector

} // namespace lima

#endif // __SLS_DETECTOR_FRAME_MAP_H
