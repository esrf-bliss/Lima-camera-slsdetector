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

#ifndef __SLS_DETECTOR_DOUBLE_BUFFER_H
#define __SLS_DETECTOR_DOUBLE_BUFFER_H

#include "lima/Debug.h"
#include "lima/Exceptions.h"
#include "lima/ThreadUtils.h"

#include <array>
#include <functional>

namespace lima 
{

namespace SlsDetector
{

template <class T> class DoubleBufferWriter;
template <class T> class DoubleBufferReader;

/*
 * DoubleBuffer
 */

template <class T>
class DoubleBuffer {
	DEB_CLASS_NAMESPC(DebModCamera, "DoubleBuffer", "SlsDetector");

 public:
	using Counter = std::size_t;

	template <class ... Args>
	DoubleBuffer(Args ... args)
		: m_base(std::forward<Args>(args)...),
		m_buf{{{0, m_base[0]}, {1, m_base[1]}}} {
		DEB_CONSTRUCTOR();
		DEB_TRACE() << DEB_VAR2(&m_buf[0].buffer, &m_buf[1].buffer);
	}

	~DoubleBuffer() {
		DEB_DESTRUCTOR();
		stop();
	}

	auto size() const { return m_base.size(); }

	auto begin() { return m_base.begin(); }
	auto begin() const { return m_base.begin(); }
	auto end() { return m_base.end(); }
	auto end() const { return m_base.end(); }

	auto operator[](std::size_t i) { return m_base[i]; }
	auto operator[](std::size_t i) const { return m_base[i]; }
	
	void reset() {
		DEB_MEMBER_FUNCT();
		stop();
		AutoMutex l = lock();
		m_buf[0].reset();
		m_buf[1].reset();
		m_stopped = false;
	}

	void stop() {
		DEB_MEMBER_FUNCT();
		AutoMutex l = lock();
		m_stopped = true;
		m_cond.broadcast();
		while (m_buf[0].busy() || m_buf[1].busy() || m_waiting)
			m_cond.wait();
	}

 private:
	friend class DoubleBufferWriter<T>;
	friend class DoubleBufferReader<T>;

	enum State {Empty, Writing, Ready, Reading};
	
	static bool valid(Counter c) { return (c != Counter(-1)); }

	struct BufferData {
		int idx;
		T& buffer;
		State state;
		Counter counter;

		BufferData(int i, T& b) : idx(i), buffer(b) { reset(); }
		void reset() { state = Empty, counter = -1; }
		bool writing() { return (state == Writing); }
		bool reading() { return (state == Reading); }
		bool busy() { return writing() || reading(); }
		bool newerThan(Counter c) {
			return (valid(counter) && (!valid(c) || (c < counter)));
		}
		bool newerThan(const BufferData& b) {
			return newerThan(b.counter);
		}
		bool canRead(Counter c) { return !busy() && newerThan(c); }
	};

	using BufferRef = std::unique_ptr<BufferData,
					  std::function<void(BufferData *)>>;

	AutoMutex lock() { return m_cond.mutex(); };

	BufferRef getWriter() {
		DEB_MEMBER_FUNCT();
		AutoMutex wl(m_write_mutex);
		AutoMutex l = lock();
		BufferData *buffer = &m_buf[0];
		if (m_buf[0].reading() ||
		    (!m_buf[1].busy() && m_buf[0].newerThan(m_buf[1])))
			buffer = &m_buf[1];
		buffer->state = Writing;
		DEB_RETURN() << DEB_VAR2(buffer->idx, &buffer->buffer);
		wl.leaveLocked();
		auto deleter = [&](auto b) { putWriter(b); };
		return {buffer, deleter};
	}

	void putWriter(BufferData *buffer) {
		DEB_MEMBER_FUNCT();
		DEB_PARAM() << DEB_VAR2(buffer->idx, &buffer->buffer);
		AutoMutex wl(m_write_mutex, AutoMutex::PrevLocked);
		AutoMutex l = lock();
		buffer->state = valid(buffer->counter) ? Ready : Empty;
		m_cond.broadcast();
	}

	BufferRef getReader(Counter c) {
		DEB_MEMBER_FUNCT();
		DEB_PARAM() << DEB_VAR1(c);
		AutoMutex rl(m_read_mutex);
		AutoMutex l = lock();
		BufferData *buffer;
		while (true) {
			bool ok0 = m_buf[0].canRead(c);
			bool ok1 = m_buf[1].canRead(c);
			if (ok0 && (!ok1 || m_buf[0].newerThan(m_buf[1]))) {
				buffer = &m_buf[0];
				break;
			} else if (ok1) {
				buffer = &m_buf[1];
				break;
			} else if (!valid(c)) {
				buffer = &m_buf[0];
				break;
			}
			DEB_TRACE() << "Start waiting";
			m_waiting = true;
			m_cond.wait();
			m_waiting = false;
			DEB_TRACE() << "Finished waiting";
			if (m_stopped) {
				m_cond.broadcast();
				return {};
			}
		}
		buffer->state = Reading;
		DEB_RETURN() << DEB_VAR2(buffer->idx, &buffer->buffer);
		rl.leaveLocked();
		auto deleter = [&](auto b) { putReader(b); };
		return {buffer, deleter};
	}

	void putReader(BufferData *buffer) {
		DEB_MEMBER_FUNCT();
		DEB_PARAM() << DEB_VAR2(buffer->idx, &buffer->buffer);
		AutoMutex rl(m_read_mutex, AutoMutex::PrevLocked);
		AutoMutex l = lock();
		buffer->state = Ready;
		m_cond.broadcast();
	}

	Cond m_cond;
	Mutex m_write_mutex, m_read_mutex;
	std::array<T, 2> m_base;
	std::array<BufferData, 2> m_buf;
	bool m_waiting{false};
	bool m_stopped{false};
};


/*
 * DoubleBufferWriter
 */

template <class T>
class DoubleBufferWriter {
	DEB_CLASS_NAMESPC(DebModCamera, "DoubleBufferWriter", "SlsDetector");

 public:
	using Counter = typename DoubleBuffer<T>::Counter;

        DoubleBufferWriter(DoubleBuffer<T>& db) : m_buf(db.getWriter()) {}

	T& getBuffer() { return m_buf->buffer; }

	void setCounter(Counter c) { m_buf->counter = c; }

 private:
	using Buffer = typename DoubleBuffer<T>::BufferRef;

	Buffer m_buf;
};


/*
 * DoubleBufferReader
 */

template <class T>
class DoubleBufferReader {
	DEB_CLASS_NAMESPC(DebModCamera, "DoubleBufferReader", "SlsDetector");

 public:
	using Counter = typename DoubleBuffer<T>::Counter;

	DoubleBufferReader(DoubleBuffer<T>& db, Counter c = -1)
		: m_buf(db.getReader(c)) {}

	operator bool() { return m_buf; }

	T& getBuffer() { return validBuffer()->buffer; }

	Counter getCounter() { return validBuffer()->counter;	}

 private:
	using BufferRef = typename DoubleBuffer<T>::BufferRef;

	auto validBuffer() {
		DEB_MEMBER_FUNCT();
		if (!m_buf)
			THROW_HW_ERROR(Error) << "Using aborted Reader";
		return m_buf.get();
	}

	BufferRef m_buf;
};

} // namespace SlsDetector

} // namespace lima



#endif // __SLS_DETECTOR_DOUBLE_BUFFER_H
