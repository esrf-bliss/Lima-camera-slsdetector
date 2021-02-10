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

#ifndef __SLS_DETECTOR_JUNGFRAU_H
#define __SLS_DETECTOR_JUNGFRAU_H

#include "SlsDetectorCamera.h"
#include "SlsDetectorDoubleBuffer.h"

#include "processlib/SinkTask.h"

namespace lima 
{

namespace SlsDetector
{

class Jungfrau : public Model
{
	DEB_CLASS_NAMESPC(DebModCamera, "Jungfrau", "SlsDetector");

 public:
	typedef unsigned char Byte;
	typedef unsigned short Word;
	typedef unsigned int Long;

	class ImgProcTask : public SinkTaskBase
	{
		DEB_CLASS_NAMESPC(DebModCamera, "Jungfrau::ImgProcTask",
				  "SlsDetector");
	public:
		ImgProcTask(Jungfrau *jungfrau);

		void setConfig(std::string  config);
		void getConfig(std::string &config);

		virtual void process(Data& data);
	private:
		Jungfrau *m_jungfrau;
	};

	Jungfrau(Camera *cam);
	~Jungfrau();

	virtual void getFrameDim(FrameDim& frame_dim, bool raw = false);

	virtual void getDetMap(Data& det_map);

	virtual std::string getName();
	virtual void getPixelSize(double& x_size, double& y_size);

	virtual void getDACInfo(NameList& name_list, IntList& idx_list,
				IntList& milli_volt_list);
	virtual void getADCInfo(NameList& name_list, IntList& idx_list,
				FloatList& factor_list, 
				FloatList& min_val_list);

	virtual void getTimeRanges(TimeRanges& time_ranges);

	// the returned object must be deleted by the caller
	ImgProcTask *createImgProcTask();

	void setHighVoltage(int  hvolt);
	void getHighVoltage(int& hvolt);

	void setThresholdEnergy(int  thres);
	void getThresholdEnergy(int& thres);

	void setImgProcConfig(std::string  config);
	void getImgProcConfig(std::string &config);

	void readGainADCMaps(Data& gain_map, Data& adc_map, FrameType& frame);

	virtual bool isXferActive();

 protected:
	virtual int getNbFrameMapItems();
	virtual void updateFrameMapItems(FrameMap *map);
	virtual void processBadItemFrame(FrameType frame, int item,
					 char *bptr);

	virtual void setThreadCPUAffinity(const CPUAffinityList& aff_list);

	virtual void updateImageSize();

	virtual bool checkSettings(Settings settings);

	virtual void prepareAcq();
	virtual void startAcq();
	virtual void stopAcq();

 private:
	class Recv
	{
		DEB_CLASS_NAMESPC(DebModCamera, "Jungfrau::Recv", "SlsDetector");
	public:
		typedef Receiver::ImageData RecvImageData;

		Recv(Jungfrau *jungfrau, int idx);

		void prepareAcq();

		bool processOneFrame(FrameType frame, char *bptr);
		void processBadFrame(FrameType frame, char *bptr);

	private:
		Jungfrau *m_jungfrau;
		int m_idx;
		bool m_raw;
		Receiver *m_recv;
		FrameDim m_frame_dim;
		int m_data_offset;
	};

	typedef std::vector<AutoPtr<Recv> > RecvList;

	class Thread : public lima::Thread
	{
		DEB_CLASS_NAMESPC(DebModCamera, "Jungfrau::Thread", "SlsDetector");
	public:
		enum State {
			Init, Ready, Running, Stopping, Quitting, End,
		};

		Thread(Jungfrau *jungfrau, int idx);
		virtual ~Thread();

		void setCPUAffinity(CPUAffinity aff);

		void prepareAcq();

		void startAcq()
		{ setState(Running); }
		void stopAcq()
		{
			setState(Stopping);
			AutoMutex l = lock();
			while (m_state != Ready)
				wait();
		}

	protected:
		virtual void threadFunction();

	private:
		friend class Jungfrau;

		AutoMutex lock()
		{ return m_jungfrau->lock(); }
		void wait()
		{ m_jungfrau->wait(); }
		void broadcast()
		{ m_jungfrau->broadcast(); }

		void setState(State state)
		{
			AutoMutex l = lock();
			m_state = state;
			broadcast();
		}

		Jungfrau *m_jungfrau;
		int m_idx;
		State m_state;
	};
	typedef std::vector<AutoPtr<Thread> > ThreadList;

	class ImgProcBase
	{
		DEB_CLASS_NAMESPC(DebModCamera, "Jungfrau::ImgProcBase", 
				  "SlsDetector");
	public:
		typedef std::vector<int> DataDims;

		ImgProcBase(Jungfrau *jungfrau, std::string name);
		virtual ~ImgProcBase();

		int getNbJungfrauModules()
		{ return m_nb_jungfrau_modules; }

		virtual void updateImageSize(Size size, bool raw);
		virtual void prepareAcq();
		virtual void processFrame(Data& data) = 0;

		static void updateDataSize(Data& d, Size size) {
			DataDims data_dims{size.getWidth(), size.getHeight()};
			if (d.empty() || (d.dimensions != data_dims)) {
				d.dimensions = data_dims;
				d.setBuffer(new Buffer(d.size()));
			}
		}

		static void clearData(Data& d) {
			memset(d.data(), 0, d.size());
		}

		static void makeDataRef(Data& src, Data& ref,
					Buffer::Callback *cb) {
			ref.type = src.type;
			ref.dimensions = src.dimensions;
			ref.frameNumber = src.frameNumber;
			ref.timestamp = src.timestamp;
			ref.header = src.header;
			Buffer *b = new Buffer;
			b->owner = Buffer::MAPPED;
			b->callback = cb;
			b->data = src.data();
			ref.setBuffer(b);
			b->unref();
		}

	protected:
		friend class Jungfrau;
		Jungfrau *m_jungfrau;
		std::string m_name;
		int m_nb_jungfrau_modules;
		bool m_raw;
		int m_pixels;
		Size m_frame_size;
		Point m_det_mods;
	};

	typedef std::vector<ImgProcBase *> ImgProcList;

	class GainADCMapImgProc : public ImgProcBase
	{
		DEB_CLASS_NAMESPC(DebModCamera, "Jungfrau::GainADCMapImgProc", 
				  "SlsDetector");
	public:
		GainADCMapImgProc(Jungfrau *jungfrau);

		virtual void updateImageSize(Size size, bool raw);
		virtual void prepareAcq();
		virtual void processFrame(Data& data);

		void readGainADCMaps(Data& gain_map, Data& adc_map,
				     FrameType& frame);

	private:
		struct MapData {
			Data gain_map;
			Data adc_map;

			MapData() {
				gain_map.type = Data::UINT8;
				adc_map.type = Data::UINT16;
			}
			void updateSize(Size size) {
				updateDataSize(gain_map, size);
				updateDataSize(adc_map, size);
			};
			void clear() {
				clearData(gain_map);
				clearData(adc_map);
			}
		};

		typedef DoubleBuffer<MapData> DBuffer;

		class ReaderHelper : public Buffer::Callback {
			DEB_CLASS_NAMESPC(DebModCamera, "Jungfrau::"
					  "GainADCMapImgProc::ReaderHelper", 
					  "SlsDetector");
		public:
			void addRead(DBuffer& b, Data& gain_map, Data& adc_map,
				     FrameType frame) {
				DEB_MEMBER_FUNCT();
				if (m_reader)
					THROW_HW_ERROR(Error)
						<< "A reader is already active";
				m_reader = new Reader(b, frame);
				MapData& m = m_reader->getBuffer();
				DEB_TRACE() << DEB_VAR1(&m);

				makeDataRef(m.gain_map, gain_map, this);
				++m_count;
				makeDataRef(m.adc_map, adc_map, this);
				++m_count;
				DEB_TRACE() << DEB_VAR1(m_count);

				frame = m_reader->getCounter();
			}

		protected:
			virtual void destroy(void *buffer) {
				DEB_MEMBER_FUNCT();
				DEB_PARAM() << DEB_VAR1(m_count);
				if ((m_count == 0) || !m_reader)
					DEB_ERROR() << DEB_VAR1(m_count);
				MapData& m = m_reader->getBuffer();
				if ((buffer != m.gain_map.data()) &&
				    (buffer != m.adc_map.data()))
					THROW_HW_ERROR(Error)
						<< "Bad buffer";
				if (--m_count == 0)
					m_reader = NULL;
			}

		private:
			typedef DoubleBufferReader<MapData> Reader;

			AutoPtr<Reader> m_reader;
			int m_count{0};
		};

		DBuffer m_buffer;
		AutoPtr<ReaderHelper> m_reader;
	};

	bool getRawMode() {
		bool raw;
		getCamera()->getRawMode(raw);
		return raw;
	}

	void addImgProc(ImgProcBase *img_proc);
	void removeImgProc(ImgProcBase *img_proc);
	void removeAllImgProc();

	ImgProcBase *createGainADCMapImgProc();

	int getNbJungfrauModules()
	{ return getNbDetModules(); }

	template <class DG>
	Defs::xy getModulePosition(const DG& det_geom, int idx);

	FrameDim getModuleFrameDim(int idx, bool raw);
	int getModuleDataOffset(int idx, bool raw);

	AutoMutex lock()
	{ return AutoMutex(m_cond.mutex()); }
	void wait()
	{ m_cond.wait(); }
	void broadcast()
	{ m_cond.broadcast(); }

	bool allFramesAcquired()
	{ return m_next_frame == m_nb_frames; }

	int getNbRecvs();

	int getNbProcessingThreads();
	void setNbProcessingThreads(int nb_proc_threads);

	void processOneFrame(AutoMutex& l);

	Cond m_cond;
	std::string m_img_proc_config;
	ImgProcList m_img_proc_list;
	RecvList m_recv_list;
	FrameType m_nb_frames;
	FrameType m_next_frame;
	FrameType m_last_frame;
	SortedIntList m_in_process;
	FrameMap::Item *m_frame_map_item;
	ThreadList m_thread_list;
};

} // namespace SlsDetector

} // namespace lima



#endif // __SLS_DETECTOR_JUNGFRAU_H
