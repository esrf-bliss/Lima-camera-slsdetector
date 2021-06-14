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
				{ 8.654262e+1, -83556.9},
				{-3.309899e+0,  18267.3},
				{-2.350506e-1,  15250.7},
			};
			static constexpr Data::TYPE DataType = Data::INT16;
			using Pixel = short;
		};

		struct Map32Data {
			static constexpr MapType Type = Map32;
			static constexpr double DefaultCoeffs[3][2] = {
				{ 1.000048e+0,    -0.4}, // effectively {1, 0}
				{-3.824832e-2, 15071.6}, // G0 x26
				{-2.716186e-3, 15023.8}, // G1 x14
			};
			static constexpr Data::TYPE DataType = Data::INT32;
			using Pixel = int;
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
	void readAveMap(Data& ave_map, FrameType& nb_frames, FrameType& frame);

	void setGainPedMapType(GainPed::MapType  map_type);
	void getGainPedMapType(GainPed::MapType& map_type);

	void setGainPedCalib(const GainPed::Calib& calib)
	{ m_gain_ped_img_proc->m_gain_ped.setCalib(calib); }
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

	virtual void updateImageSize();

	virtual bool checkSettings(Settings settings);

	virtual void prepareAcq();
	virtual void startAcq();
	virtual void stopAcq();

 private:
	friend class GainPed;

	typedef std::vector<Receiver *> RecvList;
	
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
		virtual bool consumesRawData() = 0;
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
	class ReadHelper : public Buffer::Callback {
		DEB_CLASS_NAMESPC(DebModCamera, "Jungfrau::ReadHelper", 
				  "SlsDetector");
	public:
		typedef DoubleBuffer<T> DBuffer;
		typedef DoubleBufferReader<T> DBufferReader;

		class Reader : public DBufferReader {
			DEB_CLASS_NAMESPC(DebModCamera,
					  "Jungfrau::ReadHelper::Reader", 
					  "SlsDetector");
		public:
			Reader(DBuffer& db, FrameType f, ReadHelper *h)
				: DBufferReader(db, f),	m_helper(h)
			{ DEB_CONSTRUCTOR(); }

			~Reader()
			{ DEB_DESTRUCTOR(); }

			void addData(Data& src, Data& ref) {
				DEB_MEMBER_FUNCT();
				makeDataRef(src, ref, m_helper);
				m_buffer_list.insert(ref.data());
				DEB_TRACE() << DEB_VAR1(m_buffer_list.size());
			}

		private:
			friend class ReadHelper;
			typedef std::set<void *> BufferList;

			std::pair<bool, bool> destroy(void *buffer) {
				DEB_MEMBER_FUNCT();
				DEB_PARAM() << DEB_VAR1(m_buffer_list.size());
				typedef typename BufferList::iterator BufferIt;
				BufferIt it, end = m_buffer_list.end();
				it = find(m_buffer_list.begin(), end, buffer);
				bool found = (it != end);
				if (found)
					m_buffer_list.erase(it);
				return {found, m_buffer_list.empty()};
			}

			ReadHelper *m_helper;
			BufferList m_buffer_list;
		};

		~ReadHelper() {
			DEB_DESTRUCTOR();
			AutoMutex l(m_mutex);
			while (!m_reader_list.empty())
				deleteReader(m_reader_list.begin(), l);
		}

		Reader *addRead(DBuffer& b, FrameType& frame) {
			DEB_MEMBER_FUNCT();
			Reader *r = new Reader(b, frame, this);
			frame = r->getCounter();
			AutoMutex l(m_mutex);
			m_reader_list.insert(r);
			return r;
		}

	protected:
		virtual void destroy(void *buffer) {
			DEB_MEMBER_FUNCT();
			AutoMutex l(m_mutex);
			ReaderIt it, end = m_reader_list.end();
			for (it = m_reader_list.begin(); it != end; ++it) {
				auto [found, empty] = (*it)->destroy(buffer);
				if (!found)
					continue;
				else if (empty)
					deleteReader(it, l);
				return;
			}
			DEB_ERROR() << "Bad " << DEB_VAR1(buffer);
		}

	private:
		typedef std::set<Reader *> ReaderList;
		typedef typename ReaderList::iterator ReaderIt;

		void deleteReader(ReaderIt it, AutoMutex& l) {
			DEB_MEMBER_FUNCT();
			Reader *r = *it;
			m_reader_list.erase(it);
			AutoMutexUnlock u(l);
			delete r;
		}

		Mutex m_mutex;
		ReaderList m_reader_list;
	};

	class GainADCMapImgProc : public ImgProcBase
	{
		DEB_CLASS_NAMESPC(DebModCamera, "Jungfrau::GainADCMapImgProc", 
				  "SlsDetector");
	public:
		GainADCMapImgProc(Jungfrau *jungfrau);

		virtual void updateImageSize(Size size, bool raw);
		virtual void clear();
		virtual bool consumesRawData() { return true; }
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

		typedef ReadHelper<MapData> Helper;
		typedef typename Helper::DBuffer DBuffer;

		DBuffer m_buffer;
		AutoPtr<Helper> m_helper;
	};

	class GainPedImgProc : public ImgProcBase
	{
		DEB_CLASS_NAMESPC(DebModCamera, "Jungfrau::GainPedImgProc", 
				  "SlsDetector");
	public:
		GainPedImgProc(Jungfrau *jungfrau);

		virtual void updateImageSize(Size size, bool raw);
		virtual void clear();
		virtual bool consumesRawData() { return true; }
		virtual void processFrame(Data& data);

		void readProcMap(Data& proc_map, FrameType& frame);

		GainPed m_gain_ped;

	private:
		struct MapData {
			Data proc_map;

			MapData() { proc_map.type = Data::INT32; }

			void updateSize(Size size) {
				updateDataSize(proc_map, size);
			};
			void clear() { clearData(proc_map); }
		};

		typedef ReadHelper<MapData> Helper;
		typedef typename Helper::DBuffer DBuffer;

		DBuffer m_buffer;
		AutoPtr<Helper> m_helper;
	};

	class AveImgProc : public ImgProcBase
	{
		DEB_CLASS_NAMESPC(DebModCamera, "Jungfrau::AveImgProc", 
				  "SlsDetector");
	public:
		AveImgProc(Jungfrau *jungfrau);

		virtual void updateImageSize(Size size, bool raw);
		virtual void clear();
		virtual bool consumesRawData() { return true; }
		virtual void processFrame(Data& data);

		void readAveMap(Data& ave_map, FrameType& nb_frames,
				FrameType& frame);

	private:
		template <class M>
		void processFrameFunct(Data& data);

		struct MapData {
			Data ave_map;
			FrameType nb_frames;

			MapData() { ave_map.type = Data::DOUBLE; }

			void updateSize(Size size) {
				updateDataSize(ave_map, size);
			};
			void clear() { clearData(ave_map); }
		};

		typedef ReadHelper<MapData> Helper;
		typedef typename Helper::DBuffer DBuffer;

		DBuffer m_buffer;
		AutoPtr<Helper> m_helper;
		Data m_acc;
		int m_nb_frames;
	};

	class ModelReconstruction : public SlsDetector::Reconstruction
	{
		DEB_CLASS_NAMESPC(DebModCamera, "Jungfrau::ModelReconstruction", 
				  "SlsDetector");
	public:
		ModelReconstruction(Jungfrau *jungfrau)
			: SlsDetector::Reconstruction(jungfrau->getCamera()),
			  m_jungfrau(jungfrau)
		{}

		virtual Data processModel(Data& data);

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
		void *p = d.data();
		if (p)
			memset(p, 0, d.size());
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

	Data getRawData(Data& data)
	{ return m_reconstruction->getRawData(data); }

	int getNbJungfrauModules()
	{ return getNbDetModules(); }

	template <class DG>
	Defs::xy getModulePosition(const DG& det_geom, int idx);

	FrameDim getModuleFrameDim(int idx, bool raw);
	int getModuleDataOffset(int idx, bool raw);

	int getNbRecvs();

	AutoPtr<GainPedImgProc> m_gain_ped_img_proc;
	AutoPtr<GainADCMapImgProc> m_gain_adc_map_img_proc;
	AutoPtr<AveImgProc> m_ave_img_proc;
	std::string m_img_proc_config;
	ImgProcList m_img_proc_list;
	ImgSrc m_img_src;
	ModelReconstruction *m_reconstruction;
	RecvList m_recv_list;
};

std::ostream& operator <<(std::ostream& os,
			  Jungfrau::GainPed::MapType map_type);
std::istream& operator >>(std::istream& is,
			  Jungfrau::GainPed::MapType& map_type);
std::ostream& operator <<(std::ostream& os, Jungfrau::ImgSrc src);
std::istream& operator >>(std::istream& is, Jungfrau::ImgSrc& src);

} // namespace SlsDetector

} // namespace lima



#endif // __SLS_DETECTOR_JUNGFRAU_H
