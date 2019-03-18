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

using namespace std;
using namespace lima;
using namespace lima::SlsDetector;

FrameMap::Item::FrameQueue::FrameQueue(int size) 
	: m_size(size + 1), m_write_idx(0), m_read_idx(0), m_stopped(false)
{
	m_array.resize(m_size);
}

void FrameMap::Item::FrameQueue::clear()
{
	m_write_idx = m_read_idx = 0;
}

void FrameMap::Item::FrameQueue::push(FrameData data)
{
	if (index(m_write_idx + 1) == m_read_idx)
		throw LIMA_EXC(Hardware, Error, 
			       "FrameMap::Item::FrameQueue full");
	m_array[m_write_idx] = data;
	m_write_idx = index(m_write_idx + 1);
}

FrameMap::Item::FrameDataList FrameMap::Item::FrameQueue::pop_all()
{
	while ((m_read_idx == m_write_idx) && !m_stopped)
		Sleep(1e-3);
	int write_idx = m_write_idx;
	bool two_steps = (m_read_idx > write_idx);
	int end_idx = two_steps ? m_size : write_idx;
	FrameDataList::const_iterator b = m_array.begin();
	FrameDataList ret(b + m_read_idx, b + end_idx);
	m_read_idx = index(end_idx);
	if (two_steps) {
		ret.insert(ret.end(), b, b + write_idx);
		m_read_idx = write_idx;
	}
	return ret;
}

void FrameMap::Item::FrameQueue::stop()
{
	m_stopped = true;
}

FrameMap::Item::Item()
	: m_map(NULL), m_last_pushed_frame(-1), m_last_frame(-1)
{
	DEB_CONSTRUCTOR();
}

FrameMap::Item::~Item()
{
	DEB_DESTRUCTOR();
	stopPollFrameFinished();
}

void FrameMap::Item::setFrameMap(FrameMap *map)
{
	DEB_MEMBER_FUNCT();
	m_map = map;
}

void FrameMap::Item::clear()
{
	DEB_MEMBER_FUNCT();
	m_frame_queue.clear();
	m_last_pushed_frame = -1;
	m_last_frame = -1;
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

FrameMap::Item::FrameDataList FrameMap::Item::pollFrameFinished()
{
	DEB_MEMBER_FUNCT();
	return m_frame_queue.pop_all();
}

FrameMap::Item::FinishInfoList
FrameMap::Item::getFrameFinishInfo(const FrameDataList& data_list)
{
	DEB_MEMBER_FUNCT();

	FinishInfoList finfo_list;
	FrameMap& m = *m_map;
	FrameDataList::const_iterator it, end = data_list.end();
	for (it = data_list.begin(); it != end; ++it) {
		FrameType frame = it->first;
		bool valid = it->second;
		FinishInfo finfo;
		finfo.first_lost = m_last_frame + 1;
		finfo.nb_lost = frame - finfo.first_lost + (!valid ? 1 : 0);
		for (FrameType f = m_last_frame + 1; f != (frame + 1); ++f) {
			int idx = f % m.m_buffer_size;
			AtomicCounter& count = m.m_frame_item_count_list[idx];
			bool finished = count.dec_test_and_reset(m.m_nb_items);
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

void FrameMap::Item::stopPollFrameFinished()
{
	DEB_MEMBER_FUNCT();
	m_frame_queue.stop();
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
	if (nb_items > m_nb_items) {
		ItemList::iterator it, end = m_item_list.end();
		for (it = m_item_list.begin() + m_nb_items; it != end; ++it)
			it->setFrameMap(this);
	}
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
	ItemList::iterator it, end = m_item_list.end();
	for (it = m_item_list.begin(); it != end; ++it)
		it->clear();
	CounterList& count_list = m_frame_item_count_list;
	CounterList::iterator cit, cend = count_list.end();
	for (cit = count_list.begin(); cit != cend; ++cit)
		cit->set(m_nb_items);
}

FrameArray FrameMap::getItemFrameArray() const
{
	FrameArray frame_array;
	ItemList::const_iterator it, end = m_item_list.end();
	for (it = m_item_list.begin(); it != end; ++it)
		frame_array.push_back(it->m_last_frame);
	return frame_array;
}

ostream& lima::SlsDetector::operator <<(ostream& os, const FrameMap& m)
{
	os << "<";
	os << "LastFinishedFrame=" << m.getLastFinishedFrame() << ", "
	   << "LastItemFrame=" << m.getLastItemFrame() << ", "
	   << "ItemFrameArray=" << m.getItemFrameArray();
	return os << ">";
}

