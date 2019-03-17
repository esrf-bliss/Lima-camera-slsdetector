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

#include "SlsDetectorFrameMap.h"

#include <algorithm>

using namespace std;
using namespace lima;
using namespace lima::SlsDetector;

FrameMap::Item::Follower::Follower()
	: m_group(NULL), m_item(NULL), m_read_idx(0)
{
	DEB_CONSTRUCTOR();
}

FrameMap::Item::Follower::Follower(Follower&& o)
	: m_group(move(o.m_group)), m_idx(move(o.m_idx)), m_item(NULL),
	  m_read_idx(move(o.m_read_idx))
{
	DEB_CONSTRUCTOR();
	setItem(o.m_item);
}

FrameMap::Item::Follower::~Follower()
{
	DEB_DESTRUCTOR();
	setItem(NULL);
}

void FrameMap::Item::Follower::setGroup(Group *group, int idx)
{
	DEB_MEMBER_FUNCT();
	m_group = group;
	m_idx = idx;
	m_mask.set(idx);
}

void FrameMap::Item::Follower::setItem(Item *item)
{
	DEB_MEMBER_FUNCT();

	if (item == m_item)
		return;

	if (m_item) {
		typedef FrameMap::Item::FollowerList List;
		List& l = m_item->m_follower_list;
		List::iterator it, end = l.end();
		if ((it = find(l.begin(), end, this)) == end)
			THROW_HW_ERROR(Error) << "Could not find Follower "
					      << "in Item's FollowerList";
		l.erase(it);
	}

	if ((m_item = item))
		m_item->m_follower_list.push_back(this);
}

void FrameMap::Item::Follower::update()
{
	DEB_MEMBER_FUNCT();

	FrameDataList data_list = m_item->m_frame_queue.popAll(this);
	if (data_list.empty())
		return;
	bool first_data = empty();
	m_data_queue.push(data_list);
	if (first_data)
		m_next_data = m_data_queue.front().begin();
}

void FrameMap::Item::Follower::pop()
{
	DEB_MEMBER_FUNCT();

	if (++m_next_data != m_data_queue.front().end())
		return;
	m_data_queue.pop();
	if (!empty())
		m_next_data = m_data_queue.front().begin();
}

void FrameMap::Item::Follower::clear()
{
	DEB_MEMBER_FUNCT();
	while (!empty())
		m_data_queue.pop();
}

FrameMap::Item::Group::Group()
	: m_map(NULL), m_stopped(false)
{
	DEB_CONSTRUCTOR();
}

FrameMap::Item::Group::Group(Group&& o)
	: m_map(NULL), m_stopped(move(o.m_stopped)),
	  m_follower_list(move(o.m_follower_list))
{
	DEB_CONSTRUCTOR();

	setFrameMap(o.m_map);

	FollowerList::iterator it, end = m_follower_list.end();
	int i = 0;
	for (it = m_follower_list.begin(); it != end; ++it, ++i)
		(*it)->setGroup(this, i);
}

FrameMap::Item::Group::~Group()
{
	DEB_DESTRUCTOR();
	stopPollFrameFinished();
	setFrameMap(NULL);
}

void FrameMap::Item::Group::setItemList(const ItemList& item_list)
{
	DEB_MEMBER_FUNCT();

	int nb_items = item_list.size();
	if (nb_items > MaxFrameMapItemGroupSize)
		THROW_HW_ERROR(InvalidValue)
			<< DEB_VAR2(nb_items, MaxFrameMapItemGroupSize);

	m_follower_list.clear();
	if (nb_items == 0) {
		setFrameMap(NULL);
		return;
	} 

	setFrameMap(item_list[0]->getFrameMap());

	for (int i = 0; i < nb_items; ++i) {
		Follower *f = new Follower();
		f->setGroup(this, i);
		f->setItem(item_list[i]);
		m_follower_list.push_back(f);
	}

	clear();
}

void FrameMap::Item::Group::setFrameMap(FrameMap *map)
{
	DEB_MEMBER_FUNCT();

	if (map == m_map)
		return;

	FrameMap *m;
	if (!m_map)
		m = map;
	else if (!map)
		m = m_map;
	else
		THROW_HW_ERROR(Error) << "Changed FrameMap in Group";

	GroupList& l = m->m_group_list;
	GroupList::iterator it, end = l.end();
	bool found = ((it = find(l.begin(), end, this)) != end);
	if (found)
		l.erase(it);
	else if (map)
		l.push_back(this);
	else
		THROW_HW_ERROR(Error) << "Could not find Group in FrameMap";
	m_map = map;
}

void FrameMap::Item::Group::clear()
{
	DEB_MEMBER_FUNCT();
	FollowerList::iterator it, end = m_follower_list.end();
	for (it = m_follower_list.begin(); it != end; ++it)
		(*it)->clear();
	m_last_frame = -1;
}

FrameMap::FrameDataList FrameMap::Item::Group::pollFrameFinished()
{
	DEB_MEMBER_FUNCT();

	FollowerList& l = m_follower_list;
	if (l.empty())
		THROW_HW_ERROR(Error) << "Empty Group Follower list";

	FollowerList::iterator it, end = l.end();

	bool has_data = false;
	while (!has_data && !m_stopped) {
		has_data = true;
		for (it = l.begin(); it != end; ++it) {
			(*it)->update();
			has_data &= !(*it)->empty();
		}
		if (!has_data)
			Sleep(1e-3);
	}

	FrameDataList data_list;
	while (has_data) {
		it = l.begin();
		int frame = (*it)->front().frame;
		while (++it != end)
			frame = min(frame, (*it)->front().frame);
		Mask valid;
		for (it = l.begin(); it != end; ++it) {
			FrameData& fdata = (*it)->front();
			if (fdata.frame != frame)
				continue;
			valid.set((*it)->getIndex(), fdata.valid.any());
			(*it)->pop();
			has_data &= !(*it)->empty();
		}
		data_list.push_back(FrameData(frame, valid));
	}
	return data_list;
}

FrameMap::FinishInfoList
FrameMap::Item::Group::getFrameFinishInfo(const FrameDataList& data_list)
{
	DEB_MEMBER_FUNCT();

	FinishInfoList finfo_list;
	FrameMap& m = *m_map;
	FrameDataList::const_iterator it, end = data_list.end();
	int nb_groups = m.getNbItemGroups();
	for (it = data_list.begin(); it != end; ++it) {
		FrameType frame = it->frame;
		bool valid = it->valid.any();
		FinishInfo finfo;
		finfo.first_lost = m_last_frame + 1;
		finfo.nb_lost = frame - finfo.first_lost + (!valid ? 1 : 0);
		for (FrameType f = m_last_frame + 1; f != (frame + 1); ++f) {
			int idx = f % m.m_buffer_size;
			AtomicCounter& count = m.m_frame_item_count_list[idx];
			bool finished = count.dec_test_and_reset(nb_groups);
			if (finished)
				finfo.finished.insert(f);
		}
		m_last_frame = frame;

		if (DEB_CHECK_ANY(DebTypeReturn)) {
			PrettySortedList finished_list(finfo.finished);
			DEB_RETURN() << DEB_VAR3(finfo.first_lost,
						 finfo.nb_lost, finished_list);
		}

		finfo_list.push_back(finfo);
	}

	return finfo_list;
}

void FrameMap::Item::Group::stopPollFrameFinished()
{
	DEB_MEMBER_FUNCT();
	m_stopped = true;
}

FrameMap::Item::FrameQueue::FrameQueue(Item *item, int size)
	: m_size(size + 1), m_write_idx(0),
	  m_follower_list(&item->m_follower_list)
{
	m_array.resize(m_size);
}

void FrameMap::Item::FrameQueue::clear()
{
	m_write_idx = 0;
	FollowerList::iterator it, end = m_follower_list->end();
	for (it = m_follower_list->begin(); it != end; ++it)
		(*it)->m_read_idx = 0;
}

void FrameMap::Item::FrameQueue::push(FrameData data)
{
	FollowerList::iterator it, end = m_follower_list->end();
	for (it = m_follower_list->begin(); it != end; ++it)
		if (index(m_write_idx + 1) == (*it)->m_read_idx)
			throw LIMA_EXC(Hardware, Error,
				       "FrameMap::Item::FrameQueue full");
	m_array[m_write_idx] = data;
	m_write_idx = index(m_write_idx + 1);
}

FrameMap::FrameDataList
FrameMap::Item::FrameQueue::popAll(Follower *f)
{
	volatile int& read_idx = f->m_read_idx;
	int write_idx = m_write_idx;
	if (write_idx == read_idx)
		return FrameDataList();
	bool two_steps = (read_idx > write_idx);
	int end_idx = two_steps ? m_size : write_idx;
	FrameDataList::const_iterator b = m_array.begin();
	FrameDataList ret(b + read_idx, b + end_idx);
	read_idx = index(end_idx);
	if (two_steps) {
		ret.insert(ret.end(), b, b + write_idx);
		read_idx = write_idx;
	}
	return ret;
}

FrameMap::Item::Item()
	: m_map(NULL), m_frame_queue(this), m_last_pushed_frame(-1)
{
	DEB_CONSTRUCTOR();
}

FrameMap::Item::Item(Item&& o)
	: m_map(move(o.m_map)), m_idx(move(o.m_idx)),
	  m_follower_list(move(o.m_follower_list)),
	  m_frame_queue(move(o.m_frame_queue)),
	  m_last_pushed_frame(move(o.m_last_pushed_frame))
{
	DEB_CONSTRUCTOR();
	FollowerList::iterator it, end = m_follower_list.end();
	for (it = m_follower_list.begin(); it != end; ++it)
		(*it)->setItem(this);
}

FrameMap::Item::~Item()
{
	DEB_DESTRUCTOR();

	FollowerList::reverse_iterator it, end = m_follower_list.rend();
	for (it = m_follower_list.rbegin(); it != end; ++it)
		(*it)->setItem(NULL);
}

void FrameMap::Item::init(FrameMap *map, int idx)
{
	DEB_MEMBER_FUNCT();
	m_map = map;
	m_idx = idx;
}

void FrameMap::Item::clear()
{
	DEB_MEMBER_FUNCT();
	m_frame_queue.clear();
	m_last_pushed_frame = -1;
}

void FrameMap::Item::checkFinishedFrame(FrameType frame)
{
	DEB_MEMBER_FUNCT();

	if (m_map->m_buffer_size == 0)
		THROW_HW_ERROR(InvalidValue) << "No buffer size defined";

	FrameType& last = m_last_pushed_frame;
	if (isValidFrame(last) && (frame <= last))
		THROW_HW_ERROR(Error) << DEB_VAR1(frame) << " finished already";
}

void FrameMap::Item::frameFinished(FrameType frame, bool no_check, bool valid)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR3(frame, no_check, valid);

	if (!no_check)
		checkFinishedFrame(frame);

	m_frame_queue.push(FrameData(frame, valid));
	m_last_pushed_frame = frame;
}

FrameMap::FrameMap()
	: m_nb_items(0), m_buffer_size(0)
{
	DEB_CONSTRUCTOR();
}

void FrameMap::setNbItems(int nb_items)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR2(nb_items, m_nb_items);
	if (nb_items == m_nb_items)
		return;

	m_item_list.resize(nb_items);
	for (int i = m_nb_items; i < nb_items; ++i)
		m_item_list[i].init(this, i);
	m_nb_items = nb_items;
}

void FrameMap::setBufferSize(int buffer_size)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR1(buffer_size);
	if (buffer_size == m_buffer_size)
		return;

	m_frame_item_count_list.resize(buffer_size);
	m_buffer_size = buffer_size;
}

void FrameMap::clear()
{
	DEB_MEMBER_FUNCT();

	ItemList::iterator iit, iend = m_item_list.end();
	for (iit = m_item_list.begin(); iit != iend; ++iit)
		iit->clear();

	GroupList::iterator git, gend = m_group_list.end();
	for (git = m_group_list.begin(); git != gend; ++git)
		(*git)->clear();

	int nb_groups = getNbItemGroups();
	CounterList& count_list = m_frame_item_count_list;
	CounterList::iterator cit, cend = count_list.end();
	for (cit = count_list.begin(); cit != cend; ++cit)
		cit->set(nb_groups);
}

FrameArray FrameMap::getGroupFrameArray() const
{
	FrameArray frame_array;
	GroupList::const_iterator it, end = m_group_list.end();
	for (it = m_group_list.begin(); it != end; ++it)
		frame_array.push_back((*it)->getLastFrame());
	return frame_array;
}

ostream& lima::SlsDetector::operator <<(ostream& os, const FrameMap& m)
{
	os << "<";
	os << "LastFinishedFrame=" << m.getLastFinishedFrame() << ", "
	   << "LastGroupFrame=" << m.getLastGroupFrame() << ", "
	   << "GroupFrameArray=" << m.getGroupFrameArray();
	return os << ">";
}

