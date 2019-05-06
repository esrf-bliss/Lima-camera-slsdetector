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

#ifndef __SLS_DETECTOR_EIGER_H
#define __SLS_DETECTOR_EIGER_H

#include "SlsDetectorCamera.h"

#include "processlib/LinkTask.h"


namespace lima 
{

namespace SlsDetector
{

#define EIGER_PACKET_DATA_LEN	(4 * 1024)

#define MaxEigerNbPorts		32
#define MaxEigerNbThreads	32

class Eiger : public Model
{
	DEB_CLASS_NAMESPC(DebModCamera, "Eiger", "SlsDetector");

 public:
	typedef unsigned char Byte;
	typedef unsigned short Word;
	typedef unsigned int Long;

	typedef Defs::ClockDiv ClockDiv;

	typedef std::vector<int> ThreadBalance;

	enum ParallelMode {
		NonParallel, Parallel, Safe,
	};

	class Correction : public LinkTask
	{
		DEB_CLASS_NAMESPC(DebModCamera, "Eiger::Correction",
				  "SlsDetector");
	public:
		Correction(Eiger *eiger);

		virtual Data process(Data& data);
	private:
		Eiger *m_eiger;
	};

	class Geometry
	{
		DEB_CLASS_NAMESPC(DebModCamera, "Eiger::Geometry",
				  "SlsDetector");
	public:
		typedef std::bitset<MaxEigerNbPorts> Mask;

		class Recv
		{
			DEB_CLASS_NAMESPC(DebModCamera, "Eiger::Geometry::Recv",
					  "SlsDetector");
		public:
			class Port;

			struct FrameData {
				char *src[MaxEigerNbPorts];
				char *dst;
				Mask valid;
			};

			class Port
			{
				DEB_CLASS_NAMESPC(DebModCamera,
						  "Eiger::Geometry::Recv::Port",
						  "SlsDetector");
			public:
				Port(Recv *recv, int port);

				void prepareAcq();

				FrameDim getSrcFrameDim();

				void copy(char *dst, char *src, int thread_idx);

			private:
				friend class Recv;

				struct CalcData {
					int cw;		// chip width
					int lw;		// line width
				};

				struct ThreadData {
					int src_offset;
					int dst_offset;
					int xfer_lines;
					int port_blocks;
					int first_port;
					int src_len;
				};

				Recv *m_recv;
				int m_recv_idx;
				int m_port;
				bool m_top_half_recv;
				bool m_port_idx;
				bool m_raw;
				CalcData m_src;
				CalcData m_dst;
				int m_pchips;
				int m_raw_port_blocks;
				ThreadData m_td[MaxEigerNbThreads];
			};
			typedef std::vector<AutoPtr<Port> > PortList;

			Recv(Geometry *eiger_geom, int idx);

			int getNbPorts();
			Port *getPort(int idx);

			void setNbProcessingThreads(int nb_proc_threads)
			{
				if (nb_proc_threads != m_nb_proc_threads)
					m_thread_bal.clear();
				m_nb_proc_threads = nb_proc_threads;
			}

			void setThreadBalance(const ThreadBalance& thread_bal)
			{ m_thread_bal = thread_bal; }

			static
			ThreadBalance getDefaultThreadBalance(int nb_threads);

			void prepareAcq();

			void processFrame(const FrameData& data, int thread_idx)
			{
				if (m_pixel_depth_4)
					expandPixelDepth4(data, thread_idx);
				else
					copy(data, thread_idx);
			}

			void expandPixelDepth4(const FrameData& data,
					       int thread_idx);
			void copy(const FrameData& data, int thread_idx);

			void fillBadFrame(FrameType frame, char *bptr);

		private:
			friend class Port;
			friend class Geometry;

			Geometry *m_eiger_geom;
			int m_idx;
			PortList m_port_list;
			int m_nb_proc_threads;
			ThreadBalance m_thread_bal;
			bool m_pixel_depth_4;
		};
		typedef std::vector<AutoPtr<Recv> > RecvList;

		Geometry();

		void setNbRecvs(int nb_recv);
		int getNbRecvs()
		{ return m_recv_list.size(); }
		Recv *getRecv(int idx)
		{ return m_recv_list[idx]; }

		int getNbEigerModules()
		{ return getNbRecvs() / 2; }

		void setRaw(bool  raw)
		{ m_raw = raw; }
		bool getRaw()
		{ return m_raw; }

		void setImageType(ImageType image_type)
		{ m_image_type = image_type; }
		ImageType getImageType()
		{ return m_image_type; }

		void setPixelDepth(PixelDepth pixel_depth)
		{ m_pixel_depth = pixel_depth; }
		PixelDepth getPixelDepth()
		{ return m_pixel_depth; }
		bool isPixelDepth4()
		{ return (m_pixel_depth == PixelDepth4); }


		FrameDim getFrameDim(bool raw);
		FrameDim getRecvFrameDim(bool raw);

		int getInterModuleGap(int det);

		void prepareAcq();

	private:
		friend class Recv;
		friend class Recv::Port;
		friend class Eiger;

		bool m_raw;
		ImageType m_image_type;
		PixelDepth m_pixel_depth;
		FrameDim m_recv_frame_dim;
		RecvList m_recv_list;
	};

	Eiger(Camera *cam);
	~Eiger();

	virtual void getFrameDim(FrameDim& frame_dim, bool raw = false);

	virtual std::string getName();
	virtual void getPixelSize(double& x_size, double& y_size);

	virtual void getDACInfo(NameList& name_list, IntList& idx_list,
				IntList& milli_volt_list);
	virtual void getADCInfo(NameList& name_list, IntList& idx_list,
				FloatList& factor_list, 
				FloatList& min_val_list);

	virtual void getTimeRanges(TimeRanges& time_ranges);
	static void calcTimeRanges(PixelDepth pixel_depth,
				   ClockDiv clock_div,
				   ParallelMode parallel_mode, 
				   TimeRanges& time_ranges);

	// the returned object must be deleted by the caller
	Correction *createCorrectionTask();

	void setParallelMode(ParallelMode  mode);
	void getParallelMode(ParallelMode& mode);

	void setFixedClockDiv(bool  fixed_clock_div);
	void getFixedClockDiv(bool& fixed_clock_div);
	void setClockDiv(ClockDiv  clock_div);
	void getClockDiv(ClockDiv& clock_div);
	void setSubExpTime(double  sub_exp_time);
	void getSubExpTime(double& sub_exp_time);

	void setAllTrimBits(int sub_mod_idx, int  val);
	void getAllTrimBits(int sub_mod_idx, int& val);
	void getAllTrimBitsList(IntList& val_list);

	void setHighVoltage(int  hvolt);
	void getHighVoltage(int& hvolt);

	void setThresholdEnergy(int  thres);
	void getThresholdEnergy(int& thres);

	Geometry *getGeometry()
	{ return &m_geom; }

 protected:
	virtual int getNbFrameMapItems();
	virtual void updateFrameMapItems(FrameMap *map);
	virtual void processBadItemFrame(FrameType frame, int item,
					 char *bptr);

	virtual void updateImageSize();

	virtual bool checkSettings(Settings settings);

	virtual int getNbRecvs();
	virtual Model::Recv *getRecv(int recv_idx);

	virtual void prepareAcq();
	virtual void startAcq();
	virtual void stopAcq();

 private:
	friend class Correction;
	friend class CorrBase;

	class Recv : public Model::Recv
	{
		DEB_CLASS_NAMESPC(DebModCamera, "Eiger::Recv", "SlsDetector");
	public:
		typedef Geometry::Recv::FrameData FrameData;

		class Port : public Model::Recv::Port
		{
			DEB_CLASS_NAMESPC(DebModCamera, "Eiger::Recv::Port",
					  "SlsDetector");
		public:
			struct Sync {
				Recv *recv;
				int port;
				FrameType frame;
				char *src;
				char *dst;
				bool waiting, stopped;

				Sync(Recv *r, int p) : recv(r), port(p)
				{}

				void prepareAcq()
				{
					waiting = stopped = false;
					frame = -1;
				}

				bool triggerProcess(FrameType f, char *s,
						    char *d)
				{
					AutoMutex l = recv->lockPort();
					if (waiting)
						return false;
					frame = f;
					src = s;
					dst = d;
					waiting = true;
					recv->updatePortFrame(this);
					return true;
				}

				void waitProcess()
				{
					AutoMutex l = recv->lockPort();
					while (waiting && !stopped)
						recv->waitPort();
				}

				void triggerProcessCleanUp()
				{
					AutoMutex l = recv->lockPort();
					waiting = false;
				}

				void stopAcq()
				{
					AutoMutex l = recv->lockPort();
					stopped = true;
					recv->broadcastPort();
				}
			};

			Port(Recv *recv, int port);

			void prepareAcq();
			void stopAcq();
			virtual void processFrame(FrameType frame, char *dptr,
						  uint32_t dsize, char *bptr);

			bool triggerProcess(FrameType frame, char *dptr,
					    uint32_t dsize, char *bptr)
			{ return m_sync.triggerProcess(frame, dptr, bptr); }
			void waitProcess()
			{ m_sync.waitProcess(); }
			void triggerProcessCleanUp()
			{ m_sync.triggerProcessCleanUp(); }

		private:
			friend class Recv;

			Recv *m_recv;
			int m_port;
			Sync m_sync;
			Geometry::Recv::Port *m_geom;
		};
		typedef std::vector<AutoPtr<Port> > PortList;

		Recv(Eiger *eiger, int idx);

		virtual int getNbPorts();
		virtual Port *getPort(int port_idx);

		void prepareAcq();
		void startAcq();
		void stopAcq();

		virtual void processFileStart(uint32_t dsize);

		virtual int getNbProcessingThreads();
		virtual void setNbProcessingThreads(int nb_proc_threads);
		virtual pid_t getThreadID(int thread_idx)
		{ return m_thread_list[thread_idx]->getThreadID(); }

		void updateFrameMapItem(FrameMap::Item *item)
		{ m_frame_map_item = item; }

		void processBadFrame(FrameType frame, char *bptr);

	private:
		friend class Port::Sync;

		class Thread : public lima::Thread
		{
			DEB_CLASS_NAMESPC(DebModCamera, "Eiger::Recv::Thread",
					  "SlsDetector");
		public:
			virtual ~Thread();

			void init(Recv *recv, int idx);

			void addNewFrame(const FrameData& data)
			{
				AutoMutex l = lockThread();
				m_data = &data;
				signalThread();
			}

		protected:
			virtual void start();
			virtual void threadFunction();

		private:
			AutoMutex lockThread()
			{ return AutoMutex(m_cond.mutex()); }
			void waitThread()
			{ m_cond.wait(); }
			void signalThread()
			{ m_cond.signal(); }

			Recv *m_recv;
			int m_idx;
			bool m_end;
			Cond m_cond;
			const FrameData *m_data;
		};
		typedef std::vector<AutoPtr<Thread> > ThreadList;

		AutoMutex lockPort()
		{ return m_cond.mutex(); }
		void broadcastPort()
		{ m_cond.broadcast(); }
		void waitPort()
		{ m_cond.wait(); }

		void updatePortFrame(Port::Sync *sync);
		void updateProcessingFrame();

		Eiger *m_eiger;
		int m_idx;
		Cond m_cond;
		Geometry::Recv *m_geom;
		FrameType m_frame;
		FrameType m_candidate;
		int m_nb_ready_threads;
		Geometry::Recv::FrameData m_frame_data;
		ThreadList m_thread_list;
		PortList m_port_list;
		FrameMap::Item *m_frame_map_item;
	};

	typedef std::vector<AutoPtr<Recv> > RecvList;


	class CorrBase
	{
		DEB_CLASS_NAMESPC(DebModCamera, "Eiger::CorrBase", 
				  "SlsDetector");
	public:
		CorrBase(Eiger *eiger);
		virtual ~CorrBase();

		int getNbEigerModules()
		{ return m_nb_eiger_modules; }

		virtual void prepareAcq();
		virtual void correctFrame(FrameType frame, void *ptr) = 0;

	protected:
		friend class Eiger;
		Eiger *m_eiger;
		int m_nb_eiger_modules;
		FrameDim m_mod_frame_dim;
		Size m_frame_size;
		std::vector<int> m_inter_lines;
		// TODO: add ref count
	};

	typedef std::vector<CorrBase *> CorrList;

	class BadRecvFrameCorr : public CorrBase
	{
		DEB_CLASS_NAMESPC(DebModCamera, "Eiger::BadRecvFrameCorr", 
				  "SlsDetector");
	public:
		BadRecvFrameCorr(Eiger *eiger);

		virtual void prepareAcq();
		virtual void correctFrame(FrameType frame, void *ptr);

	protected:
		struct BadFrameData {
			int last_idx;
			IntList bad_frame_list;
			void reset();
		};

		Camera *m_cam;
		int m_nb_recvs;
		std::vector<BadFrameData> m_bfd_list;
	};

	class InterModGapCorr : public CorrBase
	{
		DEB_CLASS_NAMESPC(DebModCamera, "Eiger::InterModGapCorr", 
				  "SlsDetector");
	public:
		InterModGapCorr(Eiger *eiger);

		virtual void prepareAcq();
		virtual void correctFrame(FrameType frame, void *ptr);

	protected:
		typedef std::pair<int, int> Block;
		typedef std::vector<Block> BlockList;
		BlockList m_gap_list;
	};

	template <class T>
	class ChipBorderCorr : public CorrBase
	{
	public:
		ChipBorderCorr(Eiger *eiger)
			: CorrBase(eiger)
		{}

		virtual void prepareAcq()
		{
			CorrBase::prepareAcq();

			m_f.resize(m_nb_eiger_modules);
			std::vector<BorderFactor>::iterator it = m_f.begin();
			for (int i = 0; i < m_nb_eiger_modules; ++i, ++it) {
				it->resize(2);
				(*it)[0] = m_eiger->getBorderCorrFactor(i, 0);
				(*it)[1] = m_eiger->getBorderCorrFactor(i, 1);
			}
		}

		virtual void correctFrame(FrameType frame, void *ptr)
		{
			correctBorderCols(ptr);
			correctBorderRows(ptr);
			correctInterChipCols(ptr);
			correctInterChipRows(ptr);
		}

	private:
		typedef std::vector<double> BorderFactor;

		static void correctInterChipLine(T *d, int offset, int nb_iter, 
						 int step) 
		{
			for (int i = 0; i < nb_iter; ++i, d += step)
				d[0] = d[offset] /= 2;
		}

		static void correctBorderLine(T *d, int nb_iter, int step,
					      double f) 
		{
			for (int i = 0; i < nb_iter; ++i, d += step)
				d[0] /= f;
		}

		void correctInterChipCols(void *ptr)
		{
			int width = m_frame_size.getWidth();
			int height= m_frame_size.getHeight();
			T *d = static_cast<T *>(ptr);
			for (int i = 0; i < HalfModuleChips - 1; ++i) {
				d += ChipSize;
				correctInterChipLine(d++, -1, height, width);
				correctInterChipLine(d++, 1, height, width);
			}
		}

		void correctInterChipRows(void *ptr)
		{
			int width = m_frame_size.getWidth();
			int mod_height = m_mod_frame_dim.getSize().getHeight();
			T *p = static_cast<T *>(ptr);
			for (int i = 0; i < m_nb_eiger_modules; ++i) {
				T *d = p + ChipSize * width;
				correctInterChipLine(d, -width, width, 1);
				d += width;
				correctInterChipLine(d, width, width, 1);
				if (i == m_nb_eiger_modules - 1)
					continue;
				p += (mod_height + m_inter_lines[i]) * width;
			}
		}

		void correctBorderCols(void *ptr)
		{
			int width = m_frame_size.getWidth();
			int height= m_frame_size.getHeight();
			T *d = static_cast<T *>(ptr);
			correctBorderLine(d, height, width, 2);
			d += width - 1;
			correctBorderLine(d, height, width, 2);
		}

		void correctBorderRows(void *ptr)
		{
			int width = m_frame_size.getWidth();
			int mod_height = m_mod_frame_dim.getSize().getHeight();
			T *p = static_cast<T *>(ptr);
			for (int i = 0; i < m_nb_eiger_modules; ++i) {
				double f0 = m_f[i][0], f1 = m_f[i][1];
				T *d = p;
				correctBorderLine(d, width, 1, f0);
				d += width;
				correctBorderLine(d, width, 1, f1);
				d += (mod_height - 1 - 2) * width;
				correctBorderLine(d, width, 1, f1);
				d += width;
				correctBorderLine(d, width, 1, f0);
				p += (mod_height + m_inter_lines[i]) * width;
			}
		}

		std::vector<BorderFactor> m_f;
	};

	int getNbEigerModules()
	{ return m_geom.getNbEigerModules(); }

	const FrameDim& getRecvFrameDim()
	{ return m_geom.m_recv_frame_dim; }

	CorrBase *createBadRecvFrameCorr();
	CorrBase *createChipBorderCorr(ImageType image_type);
	CorrBase *createInterModGapCorr();

	void addCorr(CorrBase *corr);
	void removeCorr(CorrBase *corr);
	void removeAllCorr();

	double getBorderCorrFactor(int det, int line);
	int getInterModuleGap(int det)
	{ return m_geom.getInterModuleGap(det); }

	static const int ChipSize;
	static const int ChipGap;
	static const int HalfModuleChips;
	static const int NbRecvPorts;

	struct LinScale {
		double factor, offset;
		LinScale(double f, double o) : factor(f), offset(o) {}
		double calcY(double x) const
		{ return x * factor + offset; }
		double calcX(double y) const
		{ return (y - offset) / factor; }
	};

	static const int BitsPerXfer;
	static const int SuperColNbCols;
	static const double BaseChipXferFreq;
	static const double MaxFebBebBandwidth;
	static const LinScale ChipXfer2Buff;
	static const LinScale ChipRealReadout;

	Geometry m_geom;
	CorrList m_corr_list;
	RecvList m_recv_list;
	bool m_fixed_clock_div;
	ClockDiv m_clock_div;
};

std::ostream& operator <<(std::ostream& os, Eiger::ParallelMode mode);

} // namespace SlsDetector

} // namespace lima



#endif // __SLS_DETECTOR_EIGER_H
