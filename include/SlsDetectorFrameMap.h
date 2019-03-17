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

#include <queue>

namespace lima
{

namespace SlsDetector
{

#define MaxFrameMapItemGroupSize	32

class FrameMap
{
	DEB_CLASS_NAMESPC(DebModCamera, "FrameMap", "SlsDetector");

 public:
	typedef std::bitset<MaxFrameMapItemGroupSize> Mask;
	struct FrameData {
		int frame;
		Mask valid;
		FrameData()
		{}
		FrameData(int f, Mask v) : frame(f), valid(v)
		{}
	};
	typedef std::vector<FrameData> FrameDataList;

	struct FinishInfo {
		FrameType first_lost;
		int nb_lost;
		SortedIntList finished;
	};
	typedef std::vector<FinishInfo> FinishInfoList;

	class Item
	{
		DEB_CLASS_NAMESPC(DebModCamera, "Item", "SlsDetector");

	private:
		class Follower;

	public:
		class Group
		{
			DEB_CLASS_NAMESPC(DebModCamera, "Item::Group",
					  "SlsDetector");
		public:
			typedef std::vector<Item *> ItemList;

			Group();
			Group(Group&& o);
			~Group();

			void setItemList(const ItemList& item_list);

			void clear();

			FrameDataList pollFrameFinished();
			FinishInfoList
			getFrameFinishInfo(const FrameDataList& data_list);
			void stopPollFrameFinished();

			FrameType getLastFrame() const
			{ return m_last_frame; }

		private:
			typedef std::vector<AutoPtr<Follower> > FollowerList;

			void setFrameMap(FrameMap *map);

			FrameMap *m_map;
			volatile bool m_stopped;
			FollowerList m_follower_list;
			FrameType m_last_frame;
		};

		Item();
		Item(Item&& o);
		~Item();

		void checkFinishedFrame(FrameType frame);
		void frameFinished(FrameType frame, bool no_check, bool valid);

		FrameMap *getFrameMap()
		{ return m_map; }

		int getIndex()
		{ return m_idx; }

	private:
		friend class Group;
		friend class FrameMap;

		class FrameQueue;

		class Follower
		{
			DEB_CLASS_NAMESPC(DebModCamera, "Item::Follower",
					  "SlsDetector");
		public:
			Follower();
			Follower(Follower&& o);
			~Follower();

			void setGroup(Group *group, int idx);
			void setItem(Item *item);

			int getIndex()
			{ return m_idx; }

			bool empty()
			{ return m_data_queue.empty(); }

			void update();
			void pop();
			void clear();

			FrameData& front()
			{ return *m_next_data; }

		private:
			friend class FrameQueue;

			typedef std::queue<FrameDataList> FrameDataQueue;

			Group *m_group;
			int m_idx;
			Mask m_mask;
			Item *m_item;
			volatile int m_read_idx;
			FrameDataQueue m_data_queue;
			FrameDataList::iterator m_next_data;
		};
		typedef std::vector<Follower *> FollowerList;

		class FrameQueue
		{
		public:
			FrameQueue(Item *item, int size = 1000);
			void clear();
			void push(FrameData data);
			FrameDataList popAll(Follower *f);

		private:
			int index(int i)
			{ return i % m_size; }

			FrameDataList m_array;
			int m_size;
			volatile int m_write_idx;
			FollowerList *m_follower_list;
		};

		friend bool SlsDetector::operator <(FrameData a, FrameData b);

		void init(FrameMap *map, int idx);
		void clear();

		FrameMap *m_map;
		int m_idx;
		FollowerList m_follower_list;
		FrameQueue m_frame_queue;
		FrameType m_last_pushed_frame;
	};
	typedef std::vector<Item> ItemList;

	FrameMap();

	void setNbItems(int nb_items);
	void setBufferSize(int buffer_size);
	void clear();

	Item& getItem(int item)
	{ return m_item_list[item]; }

	FrameArray getGroupFrameArray() const;

	FrameType getLastGroupFrame() const
	{ return getLatestFrame(getGroupFrameArray()); }

	FrameType getLastFinishedFrame() const
	{ return getOldestFrame(getGroupFrameArray()); }

	int getNbItemGroups()
	{ return m_group_list.size(); }

 private:
	friend class Item;

	typedef std::vector<Item::Group *> GroupList;

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
	GroupList m_group_list;
};

std::ostream& operator <<(std::ostream& os, const FrameMap& m);


} // namespace SlsDetector

} // namespace lima

#endif // __SLS_DETECTOR_FRAME_MAP_H
