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

#include <algorithm>

namespace lima
{

namespace SlsDetector
{

class FrameMap
{
	DEB_CLASS_NAMESPC(DebModCamera, "FrameMap", "SlsDetector");

 public:
	struct FinishInfo {
		FrameType first_lost;
		int nb_lost;
		SortedIntList finished;
	};

	class Item
	{
		DEB_CLASS_NAMESPC(DebModCamera, "Item", "SlsDetector");

	public:
		Item(FrameMap *map, int idx);

		FrameMap *getFrameMap()
		{ return m_map; }

		int getIndex()
		{ return m_idx; }

		void checkFinishedFrame(FrameType frame);
		FinishInfo frameFinished(FrameType frame, bool no_check,
					 bool valid);

		bool isBadFrame(FrameType frame);
		int getNbBadFrames();
		void getBadFrameList(int first_idx, int last_idx, IntList& bfl);

	private:
		friend class FrameMap;

		void clear();

		AutoMutex lock()
		{ return m_mutex; }

		Mutex m_mutex;
		FrameMap *m_map;
		int m_idx;
		FrameType m_last_frame;
		IntList m_bad_frame_list;
	};
	typedef std::vector<AutoPtr<Item> > ItemList;

	FrameMap(Camera *cam);

	void setNbItems(int nb_items);
	int getNbItems()
	{ return m_nb_items; }

	void setBufferSize(int buffer_size);
	void clear();

	Item *getItem(int item)
	{ return m_item_list[item]; }

	FrameArray getItemFrameArray() const;

	FrameType getLastItemFrame() const
	{ return getLatestFrame(getItemFrameArray()); }

	FrameType getLastFinishedFrame() const
	{ return getOldestFrame(getItemFrameArray()); }

	XYStat::LinRegress calcDelayStat()
	{ return m_delay_stat.calcLinRegress(); }

 private:
	friend class Item;

	struct AtomicCounter {
		int count;
		Mutex mutex;
		Timestamp t0;

		void set(int reset)
		{
			mutex.lock();
			count = reset;
			mutex.unlock();
		}

		bool dec_test_and_reset(int reset, double& delay)
		{
			mutex.lock();
			Timestamp t = Timestamp::now();
			if (count == reset)
				t0 = t;
			bool zero = (--count == 0);
			if (zero) {
				count = reset;
				delay = t - t0;
			}
			mutex.unlock();
			return zero;
		}
	};
	typedef std::vector<AtomicCounter> CounterList;

	Camera *m_cam;
	int m_nb_items;
	int m_buffer_size;
	CounterList m_frame_item_count_list;
	ItemList m_item_list;
	XYStat m_delay_stat;
};

inline bool FrameMap::Item::isBadFrame(FrameType frame)
{ 
	AutoMutex l = lock();
	IntList::iterator end = m_bad_frame_list.end();
	return (std::find(m_bad_frame_list.begin(), end, frame) != end); 
}

inline int FrameMap::Item::getNbBadFrames()
{
	AutoMutex l = lock();
	return m_bad_frame_list.size();
}
	
inline void FrameMap::Item::getBadFrameList(int first_idx, int last_idx,
					    IntList& bfl)
{
	AutoMutex l = lock();
	IntList::const_iterator b = m_bad_frame_list.begin();
	bfl.assign(b + first_idx, b + last_idx);
}


std::ostream& operator <<(std::ostream& os, const FrameMap& m);


} // namespace SlsDetector

} // namespace lima

#endif // __SLS_DETECTOR_FRAME_MAP_H
