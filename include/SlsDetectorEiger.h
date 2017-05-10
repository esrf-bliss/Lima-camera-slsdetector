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

class Eiger : public Camera::Model
{
	DEB_CLASS_NAMESPC(DebModCamera, "Eiger", "SlsDetector");

 public:
	typedef Defs::Settings Settings;
	typedef Defs::ReadoutFlags ReadoutFlags;
	typedef Camera::FrameType FrameType;
	typedef Camera::NameList NameList;
	typedef Camera::IntList IntList;
	typedef Camera::FloatList FloatList;

	typedef unsigned char Byte;
	typedef unsigned short Word;
	typedef unsigned int Long;

	class CorrBase
	{
		DEB_CLASS_NAMESPC(DebModCamera, "Eiger::CorrBase", 
				  "SlsDetector");
	public:
		CorrBase(Eiger *eiger);
		virtual ~CorrBase();

		bool getRaw();
		int getNbModules()
		{ return m_nb_modules; }

		virtual void prepareAcq();
		virtual void correctFrame(FrameType frame, void *ptr) = 0;

	protected:
		friend class Eiger;
		Eiger *m_eiger;
		int m_nb_modules;
		FrameDim m_mod_frame_dim;
		Size m_frame_size;
		std::vector<int> m_inter_lines;
	};

	typedef std::vector<AutoPtr<CorrBase> > CorrList;

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

	class Correction : public LinkTask
	{
		DEB_CLASS_NAMESPC(DebModCamera, "Eiger::Correction", 
				  "SlsDetector");
	public:
		Correction(Eiger *eiger);

		virtual Data process(Data& data);
	private:
		CorrList& m_corr_list;
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

	// the returned object must be deleted by the caller
	Correction *createCorrectionTask();

 protected:
	virtual void updateImageSize();

	virtual bool checkSettings(Settings settings);

	virtual ReadoutFlags getReadoutFlagsMask();
	virtual bool checkReadoutFlags(ReadoutFlags flags,
				       IntList& flag_list,
				       bool silent = false);

	virtual int getRecvPorts();

	virtual void prepareAcq();
	virtual void processRecvFileStart(int recv_idx, uint32_t dsize);
	virtual void processRecvPort(int recv_idx, FrameType frame, int port,
				     char *dptr, uint32_t dsize, Mutex& lock, 
				     char *bptr);

 private:
	friend class Correction;
	friend class CorrBase;

	class RecvPortGeometry
	{
		DEB_CLASS_NAMESPC(DebModCamera, "Eiger::RecvPortGeometry", 
				  "SlsDetector");
	public:
		RecvPortGeometry(Eiger *eiger, int recv_idx, int port);

		void prepareAcq();
		void processRecvFileStart(uint32_t dsize);
		void processRecvPort(FrameType frame, char *dptr, char *bptr);
	private:
		Eiger *m_eiger;
		int m_port;
		bool m_top_half_recv;
		bool m_port_idx;
		bool m_raw;
		int m_recv_idx;
		int m_port_offset;
		int m_dlw;			// dest line width
		int m_plw;			// packet line width
		int m_clw;			// chip line width
		int m_pchips;
	};

	typedef std::vector<AutoPtr<RecvPortGeometry> > PortGeometryList;

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

			m_f.resize(m_nb_modules);
			std::vector<BorderFactor>::iterator it = m_f.begin();
			for (int i = 0; i < m_nb_modules; ++i, ++it) {
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
			for (int i = 0; i < m_nb_modules; ++i) {
				T *d = p + ChipSize * width;
				correctInterChipLine(d, -width, width, 1);
				d += width;
				correctInterChipLine(d, width, width, 1);
				if (i == m_nb_modules - 1)
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
			for (int i = 0; i < m_nb_modules; ++i) {
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

	int countFlags(ReadoutFlags flags);

	void getRecvFrameDim(FrameDim& frame_dim, bool raw, bool geom);

	int getPortIndex(int recv_idx, int port)
	{ return recv_idx * RecvPorts + port; }

	CorrBase *createChipBorderCorr(ImageType image_type);
	CorrBase *createInterModGapCorr();

	void addCorr(CorrBase *corr);
	void removeCorr(CorrBase *corr);
	void removeAllCorr();

	double getBorderCorrFactor(int det, int line);
	int getInterModuleGap(int det);

	static const int ChipSize;
	static const int ChipGap;
	static const int HalfModuleChips;
	static const int RecvPorts;

	int m_nb_det_modules;
	FrameDim m_recv_frame_dim;
	CorrList m_corr_list;
	PortGeometryList m_port_geom_list;
};


} // namespace SlsDetector

} // namespace lima



#endif // __SLS_DETECTOR_EIGER_H
