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
#include "SlsDetectorCamera.h"

using namespace std;
using namespace lima;
using namespace lima::SlsDetector;

FrameMap::Item::Item(FrameMap *map, int idx)
	: m_map(map), m_idx(idx), m_last_frame(-1)
{
	DEB_CONSTRUCTOR();
}

void FrameMap::Item::clear()
{
	DEB_MEMBER_FUNCT();
	m_last_frame = -1;
	m_bad_frame_list.clear();
}

void FrameMap::Item::checkFinishedFrame(FrameType frame)
{
	DEB_MEMBER_FUNCT();

	if (m_map->m_buffer_size == 0)
		THROW_HW_ERROR(InvalidValue) << "No buffer size defined";

	FrameType& last = m_last_frame;
	if (isValidFrame(last) && (frame <= last))
		THROW_HW_ERROR(Error) << DEB_VAR1(frame) << " finished already";
}

FrameMap::FinishInfo
FrameMap::Item::frameFinished(FrameType frame, bool no_check, bool valid)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR3(frame, no_check, valid);

	if (!no_check)
		checkFinishedFrame(frame);

	int nb_items = m_map->getNbItems();
	FinishInfo finfo;
	finfo.first_lost = m_last_frame + 1;
	finfo.nb_lost = frame - finfo.first_lost + (!valid ? 1 : 0);
	for (FrameType f = m_last_frame + 1; f != (frame + 1); ++f) {
		bool finished;
		if (nb_items > 1) {
			int idx = f % m_map->m_buffer_size;
			CounterList& cl = m_map->m_frame_item_count_list;
			AtomicCounter& count = cl[idx];
			double delay;
			bool finished = count.dec_test_and_reset(nb_items, delay);
			if (finished)
				m_map->m_delay_stat.add(frame, delay);
		} else {
			finished = true;
		}
		if (finished)
			finfo.finished.insert(f);
	}

	if (DEB_CHECK_ANY(DebTypeReturn)) {
		PrettySortedList finished_list(finfo.finished);
		DEB_RETURN() << DEB_VAR3(finfo.first_lost, finfo.nb_lost,
					 finished_list);
	}

	AutoMutex l = lock();

	m_last_frame = frame;

	FrameType f = finfo.first_lost;
	for (int i = 0; i < finfo.nb_lost; ++i, ++f)
		m_bad_frame_list.push_back(f);

	return finfo;
}

FrameMap::FrameMap(Camera *cam)
	: m_cam(cam), m_nb_items(0), m_buffer_size(0)
{
	DEB_CONSTRUCTOR();
}

void FrameMap::setNbItems(int nb_items)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR2(nb_items, m_nb_items);
	if (nb_items == m_nb_items)
		return;

	for (int i = m_nb_items; i < nb_items; ++i) {
		Item *item = new Item(this, i);
		m_item_list.push_back(item);
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

	ItemList::iterator iit, iend = m_item_list.end();
	for (iit = m_item_list.begin(); iit != iend; ++iit)
		(*iit)->clear();

	if (m_nb_items > 1) {
		CounterList& count_list = m_frame_item_count_list;
		CounterList::iterator cit, cend = count_list.end();
		for (cit = count_list.begin(); cit != cend; ++cit)
			cit->set(m_nb_items);

		m_delay_stat.reset();
	}
}

FrameArray FrameMap::getItemFrameArray() const
{
	FrameArray frame_array;
	ItemList::const_iterator it, end = m_item_list.end();
	for (it = m_item_list.begin(); it != end; ++it) {
		AutoMutex l = (*it)->lock();
		frame_array.push_back((*it)->m_last_frame);
	}
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

