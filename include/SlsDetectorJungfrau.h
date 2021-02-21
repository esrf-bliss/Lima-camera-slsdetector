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

#include <array>
#include <variant>

namespace lima 
{

namespace SlsDetector
{

typedef std::array<Data, 3> JungfrauGainDataSet;

class Jungfrau : public Model
{
	DEB_CLASS_NAMESPC(DebModCamera, "Jungfrau", "SlsDetector");

 public:
	typedef unsigned char Byte;
	typedef unsigned short Word;
	typedef unsigned int Long;

	class GainPed
	{
		DEB_CLASS_NAMESPC(DebModCamera, "Jungfrau::GainPed",
				  "SlsDetector");
	public:
		struct Calib {
			JungfrauGainDataSet gain_map;
			JungfrauGainDataSet ped_map;
			void clear() {
				for (auto& d: gain_map) clearData(d);
				for (auto& d: ped_map) clearData(d);
			}
		};

		enum MapType { Map16, Map32 };

		struct Map16Data {
			static constexpr MapType Type = Map16;
			static constexpr double DefaultCoeffs[3][2] = {
				{ 0.011555,   965.5},
				{-0.302124,  5519.0},
				{-4.254403, 64882.8},
			};
			static constexpr Data::TYPE DataType = Data::UINT16;
			using Pixel = unsigned short;
		};

		struct Map32Data {
			static constexpr MapType Type = Map32;
			static constexpr double DefaultCoeffs[3][2] = {
				{   0.999952,       0.4}, // effectively {1, 0}
				{ -26.144935,  394047.3}, // G0 x16
				{-368.163215, 5531211.9}, // G1 x14
			};
			static constexpr Data::TYPE DataType = Data::UINT32;
			using Pixel = unsigned int;
		};

		GainPed(Jungfrau *jungfrau);

		void setMapType(MapType  map_type);
		void getMapType(MapType& map_type);

		void updateImageSize(Size size, bool raw);

		void getDefaultCalib(Calib& calib);

		Data::TYPE getDataType();

		void processFrame(Data& data, Data& proc);

		void setCalib(const Calib& calib);
		void getCalib(Calib& calib);

	private:
		template <class M> struct Impl {
			DEB_CLASS_NAMESPC(DebModCamera,
					  "Jungfrau::GainPed::Impl",
					  "SlsDetector");
		public:
			using Map = M;
			Impl(Jungfrau *j) : m_jungfrau(j) {}
			void updateImageSize(Size size, bool raw);
			void getDefaultCalib(Calib& calib);
			void setDefaultCalib() { getDefaultCalib(m_calib); }
			void processFrame(Data& data, Data& proc);

			Jungfrau *m_jungfrau;
			Size m_size;
			bool m_raw;
			int m_pixels{0};
			Calib m_calib;
		};
		using AnyImpl = std::variant<Impl<Map16Data>, Impl<Map32Data>>;

		Jungfrau *m_jungfrau;
		AnyImpl m_impl;
		Size m_size;
		bool m_raw;
	};

	class ImgProcTask : public SinkTaskBase
	{
		DEB_CLASS_NAMESPC(DebModCamera, "Jungfrau::ImgProcTask",
				  "SlsDetector");
	public:
		ImgProcTask(Jungfrau *jungfrau);

		virtual void process(Data& data);
	private:
		Jungfrau *m_jungfrau;
	};

	enum ImgSrc { Raw, GainPedCorr };

	Jungfrau(Camera *cam);
	~Jungfrau();

	virtual void getFrameDim(FrameDim& frame_dim, bool raw = false);
	virtual void getAcqFrameDim(FrameDim& frame_dim, bool raw = false);

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

	void readGainPedProcMap(Data& proc_map, FrameType& frame);

	void setGainPedMapType(GainPed::MapType  map_type);
	void getGainPedMapType(GainPed::MapType& map_type);

	void getGainPedCalib(GainPed::Calib& calib)
	{ m_gain_ped_img_proc->m_gain_ped.getCalib(calib); }

	void setImgSrc(ImgSrc  img_src);
	void getImgSrc(ImgSrc& img_src);

	virtual bool isXferActive();

	virtual Reconstruction *getReconstruction()
	{ return m_reconstruction; }

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
	friend class GainPed;

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
		virtual void clear();
		virtual void processFrame(Data& data) = 0;

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

	template <class T>
	class ReaderHelper : public Buffer::Callback {
		DEB_CLASS_NAMESPC(DebModCamera, "Jungfrau::ReaderHelper", 
				  "SlsDetector");
	public:
		typedef DoubleBuffer<T> DBuffer;

		T& addRead(DBuffer& b, FrameType& frame) {
			DEB_MEMBER_FUNCT();
			if (m_reader)
				THROW_HW_ERROR(Error)
					<< "A reader is already active";
			m_reader = new Reader(b, frame);
			frame = m_reader->getCounter();
			return m_reader->getBuffer();
		}

		void addData(Data& src, Data& ref) {
			DEB_MEMBER_FUNCT();
			makeDataRef(src, ref, this);
			m_buffer_list.insert(ref.data());
			DEB_TRACE() << DEB_VAR1(m_buffer_list.size());
		}

	protected:
		virtual void destroy(void *buffer) {
			DEB_MEMBER_FUNCT();
			DEB_PARAM() << DEB_VAR1(m_buffer_list.size());
			if (m_buffer_list.empty() || !m_reader)
				DEB_ERROR() << "Unexpected "
					    << DEB_VAR1(buffer);
			BufferList::iterator it, end = m_buffer_list.end();
			it = find(m_buffer_list.begin(), end, buffer);
			if (it == end)
				DEB_ERROR() << "Bad " << DEB_VAR1(buffer);
			m_buffer_list.erase(it);
			if (m_buffer_list.empty())
				m_reader = NULL;
		}

	private:
		typedef DoubleBufferReader<T> Reader;
		typedef std::set<void *> BufferList;

		AutoPtr<Reader> m_reader;
		BufferList m_buffer_list;
	};

	class GainADCMapImgProc : public ImgProcBase
	{
		DEB_CLASS_NAMESPC(DebModCamera, "Jungfrau::GainADCMapImgProc", 
				  "SlsDetector");
	public:
		GainADCMapImgProc(Jungfrau *jungfrau);

		virtual void updateImageSize(Size size, bool raw);
		virtual void clear();
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

		typedef ReaderHelper<MapData> Reader;
		typedef typename Reader::DBuffer DBuffer;

		DBuffer m_buffer;
		AutoPtr<Reader> m_reader;
	};

	class GainPedImgProc : public ImgProcBase
	{
		DEB_CLASS_NAMESPC(DebModCamera, "Jungfrau::GainPedImgProc", 
				  "SlsDetector");
	public:
		GainPedImgProc(Jungfrau *jungfrau);

		virtual void updateImageSize(Size size, bool raw);
		virtual void clear();
		virtual void processFrame(Data& data);

		void readProcMap(Data& proc_map, FrameType& frame);

		GainPed m_gain_ped;

	private:
		struct MapData {
			Data proc_map;

			MapData() { proc_map.type = Data::UINT16; }

			void updateSize(Size size) {
				updateDataSize(proc_map, size);
			};
			void clear() { clearData(proc_map); }
		};

		typedef ReaderHelper<MapData> Reader;
		typedef typename Reader::DBuffer DBuffer;

		DBuffer m_buffer;
		AutoPtr<Reader> m_reader;
	};

	class ModelReconstruction : public SlsDetector::Reconstruction
	{
		DEB_CLASS_NAMESPC(DebModCamera, "Jungfrau::ModelReconstruction", 
				  "SlsDetector");
	public:
		ModelReconstruction(Jungfrau *jungfrau) : m_jungfrau(jungfrau)
		{}

		virtual Data process(Data& data);

	private:
		friend class Jungfrau;
		Jungfrau *m_jungfrau;
	};

	bool getRawMode() {
		bool raw;
		getCamera()->getRawMode(raw);
		return raw;
	}

	static void updateDataSize(Data& d, Size size) {
		std::vector<int> data_dims{size.getWidth(), size.getHeight()};
		if (d.empty() || (d.dimensions != data_dims)) {
			d.dimensions = data_dims;
			Buffer *b = new Buffer(d.size());
			d.setBuffer(b);
			b->unref();
			clearData(d);
		}
	}

	static void clearData(Data& d) {
		memset(d.data(), 0, d.size());
	}

	static void makeDataRef(Data& src, Data& ref, Buffer::Callback *cb) {
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

	static void initData(Data& d, Size size, Data::TYPE type) {
		std::vector<int> dims = {size.getWidth(), size.getHeight()};
		if ((d.type != type) || (d.dimensions != dims)) {
			d.type = type;
			d.dimensions = dims;
			Buffer *b = new Buffer(d.size());
			d.setBuffer(b);
			b->unref();
			clearData(d);
		}
	}

	void addImgProc(ImgProcBase *img_proc);
	void removeImgProc(ImgProcBase *img_proc);
	void removeAllImgProc();
	void doSetImgProcConfig(std::string config, bool force);

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
	AutoPtr<GainPedImgProc> m_gain_ped_img_proc;
	AutoPtr<GainADCMapImgProc> m_gain_adc_map_img_proc;
	std::string m_img_proc_config;
	ImgProcList m_img_proc_list;
	ImgSrc m_img_src;
	BufferCtrlObj m_acq_buffer_ctrl_obj;
	ModelReconstruction *m_reconstruction;
	RecvList m_recv_list;
	FrameType m_nb_frames;
	FrameType m_next_frame;
	FrameType m_last_frame;
	SortedIntList m_in_process;
	FrameMap::Item *m_frame_map_item;
	ThreadList m_thread_list;
};

std::ostream& operator <<(std::ostream& os, Jungfrau::GainPed::MapType map_type);
std::ostream& operator <<(std::ostream& os, Jungfrau::ImgSrc src);

} // namespace SlsDetector

} // namespace lima



#endif // __SLS_DETECTOR_JUNGFRAU_H
