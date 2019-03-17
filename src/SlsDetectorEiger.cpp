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

#include "SlsDetectorEiger.h"
#include "lima/MiscUtils.h"

#include <emmintrin.h>

using namespace std;
using namespace lima;
using namespace lima::SlsDetector;
using namespace lima::SlsDetector::Defs;

const int Eiger::ChipSize = 256;
const int Eiger::ChipGap = 2;
const int Eiger::HalfModuleChips = 4;
const int Eiger::NbRecvPorts = 2;

const int Eiger::BitsPerXfer = 4;
const int Eiger::SuperColNbCols = 8;
const double Eiger::BaseChipXferFreq = 200; // Mbit/s
const double Eiger::MaxFebBebBandwidth = 25600; // Mbit/s
const Eiger::LinScale Eiger::ChipXfer2Buff(2.59, 0.85);
const Eiger::LinScale Eiger::ChipRealReadout(1.074, -4);

Eiger::CorrBase::CorrBase(Eiger *eiger)
	: m_eiger(eiger)
{
	DEB_CONSTRUCTOR();
	m_nb_eiger_modules = m_eiger->getNbEigerModules();
}

Eiger::CorrBase::~CorrBase()
{
	DEB_DESTRUCTOR();
	if (m_eiger)
		m_eiger->removeCorr(this);
}

void Eiger::CorrBase::prepareAcq()
{
	DEB_MEMBER_FUNCT();

	if (!m_eiger)
		THROW_HW_ERROR(InvalidValue) << "Correction already removed";

	FrameDim& recv_dim = m_eiger->m_recv_frame_dim;
	m_mod_frame_dim = recv_dim * Point(1, 2);
	m_frame_size = m_mod_frame_dim.getSize() * Point(1, m_nb_eiger_modules);
	m_inter_lines.resize(m_nb_eiger_modules);
	for (int i = 0; i < m_nb_eiger_modules - 1; ++i) {
		m_inter_lines[i] = m_eiger->getInterModuleGap(i);
		m_frame_size += Point(0, m_inter_lines[i]);
	}
	m_inter_lines[m_nb_eiger_modules - 1] = 0;
}

void Eiger::BadRecvFrameCorr::BadFrameData::reset()
{
	last_idx = 0;
	bad_frame_list.clear();
}

Eiger::BadRecvFrameCorr::BadRecvFrameCorr(Eiger *eiger)
	: CorrBase(eiger)
{
	DEB_CONSTRUCTOR();

	m_cam = m_eiger->getCamera();
	m_nb_ports = m_cam->getTotNbPorts();
	m_bfd_list.resize(m_nb_ports);
}

void Eiger::BadRecvFrameCorr::prepareAcq()
{
	DEB_MEMBER_FUNCT();

	CorrBase::prepareAcq();

	for (int i = 0; i < m_nb_ports; ++i)
		m_bfd_list[i].reset();
}

void Eiger::BadRecvFrameCorr::correctFrame(FrameType frame, void *ptr)
{
	DEB_MEMBER_FUNCT();

	char *bptr = (char *) ptr;
	for (int i = 0; i < m_nb_ports; ++i) {
		BadFrameData& bfd = m_bfd_list[i];
		IntList& bfl = bfd.bad_frame_list;
		int& last_idx = bfd.last_idx;
		if (bfl.empty()) {
			int bad_frames = m_cam->getNbBadFrames(i);
			if (bad_frames == last_idx)
				continue;
			m_cam->getBadFrameList(i, last_idx, bad_frames, bfl);
		}
		IntList::iterator end = bfl.end();
		if (find(bfl.begin(), end, frame) != end) {
			Model::RecvPort *port = m_eiger->getRecvPort(i);
			port->processRecvPort(frame, NULL, 0, bptr);
		}
		if (*(end - 1) > int(frame))
			continue;
		last_idx += bfl.size();
		bfl.clear();
	}
}

Eiger::InterModGapCorr::InterModGapCorr(Eiger *eiger)
	: CorrBase(eiger)
{
	DEB_CONSTRUCTOR();
}

void Eiger::InterModGapCorr::prepareAcq()
{
	DEB_MEMBER_FUNCT();

	CorrBase::prepareAcq();

	m_gap_list.clear();

	int mod_size = m_mod_frame_dim.getMemSize();
	int width = m_mod_frame_dim.getSize().getWidth();
	int ilw = width * m_mod_frame_dim.getDepth();
	for (int i = 0, start = 0; i < m_nb_eiger_modules - 1; ++i) {
		start += mod_size;
		int size = m_inter_lines[i] * ilw;
		m_gap_list.push_back(Block(start, size));
		start += size;
	}
}

void Eiger::InterModGapCorr::correctFrame(FrameType frame, void *ptr)
{
	DEB_MEMBER_FUNCT();
	
	char *dest = static_cast<char *>(ptr);
	BlockList::const_iterator it, end = m_gap_list.end();
	for (it = m_gap_list.begin(); it != end; ++it) {
		int start = it->first;
		int size = it->second;
		memset(dest + start, 0, size);
	}
}

Eiger::Correction::Correction(Eiger *eiger)
	: m_eiger(eiger)
{
	DEB_CONSTRUCTOR();
}

Data Eiger::Correction::process(Data& data)
{
	DEB_MEMBER_FUNCT();

	DEB_PARAM() << DEB_VAR3(data.frameNumber, 
				_processingInPlaceFlag, data.data());
	
	Data ret = data;

	if (!_processingInPlaceFlag) {
		int size = data.size();
		Buffer *buffer = new Buffer(size);
		memcpy(buffer->data, data.data(), size);
		ret.setBuffer(buffer);
		buffer->unref();
	}

	FrameType frame = ret.frameNumber;
	void *ptr = ret.data();
	CorrList& corr_list = m_eiger->m_corr_list;
	CorrList::iterator it, end = corr_list.end();
	for (it = corr_list.begin(); it != end; ++it)
		(*it)->correctFrame(frame, ptr);

	return ret;
}

Eiger::RecvPort::RecvPort(Eiger *eiger, int recv_idx, int port)
	: m_eiger(eiger), m_port(port), m_recv_idx(recv_idx), m_nb_threads(2),
	  m_nb_buffers(1024), m_buffer_cb_mgr(m_buffer_alloc_mgr)
{
	DEB_CONSTRUCTOR();
	DEB_PARAM() << DEB_VAR1(m_recv_idx);
	m_top_half_recv = (m_recv_idx % 2 == 0);
}

void Eiger::RecvPort::prepareAcq()
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR1(m_recv_idx);

	m_pixel_depth_4 = m_eiger->isPixelDepth4();
	m_eiger->getCamera()->getRawMode(m_raw);
	m_thread_proc = (m_pixel_depth_4 && m_eiger->m_expand_4_in_threads);

	const FrameDim& frame_dim = m_eiger->m_recv_frame_dim;
	const Size& size = frame_dim.getSize();
	int depth = frame_dim.getDepth();
	m_dlw = size.getWidth() * depth;
	int recv_size = frame_dim.getMemSize();
	m_spo = 0;
	m_dpo = recv_size * m_recv_idx;	

	m_pchips = HalfModuleChips / NbRecvPorts;
	m_scw = ChipSize * depth;
	m_dcw = m_scw;
	if (m_pixel_depth_4)
		m_scw /= 2;
	m_slw = m_pchips * m_scw;

	m_copy_lines = ChipSize;
	if (m_thread_proc) {
		Camera *cam = m_eiger->getCamera();
		PixelDepthCPUAffinityMap aff_map;
		cam->getPixelDepthCPUAffinityMap(aff_map);
		PixelDepth pixel_depth;
		cam->getPixelDepth(pixel_depth);
		const RecvCPUAffinity& r = aff_map[pixel_depth].recv[m_recv_idx];
		CPUAffinity buffer_aff = r.writers[m_port];
		m_buffer_alloc_mgr.setCPUAffinityMask(buffer_aff);
		int nb_concat_frames = 8;
		int nb_buffers = m_nb_buffers / nb_concat_frames;
		FrameDim fdim(m_slw, ChipSize, Bpp8);
		DEB_ALWAYS() << "Aux: " << DEB_VAR2(buffer_aff, fdim);
		m_buffer_cb_mgr.allocBuffers(nb_buffers, nb_concat_frames, fdim);
		m_copy_lines /= m_nb_threads;
		m_sto = m_copy_lines * m_slw;
		m_dto = m_copy_lines * m_dlw;
		DEB_ALWAYS() << DEB_VAR3(m_copy_lines, m_sto, m_dto);

		m_last_recv_frame = m_last_proc_frame = -1;
		m_overrun = false;
	} else {
		m_buffer_cb_mgr.releaseBuffers();
	}

	if (m_raw) {
		// vert. port concat.
		m_dpo += ChipSize * m_dlw * m_port;
		return;
	} else {
		// inter-chip horz. gap
		m_dcw += ChipGap * depth;
		// horz. port concat.
		m_dpo += m_pchips * m_dcw * m_port;
	}

	int mod_idx = m_recv_idx / 2;
	for (int i = 0; i < mod_idx; ++i)
		m_dpo += m_eiger->getInterModuleGap(i) * m_dlw;

	if (m_top_half_recv) {
		// top-half module: vert-flipped data
		m_spo += (ChipSize - 1) * m_slw;
		m_slw *= -1;
		m_sto *= -1;
	} else {
		// bottom-half module: inter-chip vert. gap
		m_dpo += (ChipGap / 2) * m_dlw;
	}
}

void Eiger::RecvPort::processRecvFileStart(uint32_t dsize)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR2(m_recv_idx, dsize);
}

void Eiger::RecvPort::processRecvPort(FrameType frame, char *dptr,
				      uint32_t dsize, char *bptr)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR3(frame, m_recv_idx, m_port);

	bool valid_data = (dptr != NULL);
	if (!m_thread_proc || !valid_data) {
		if (m_pixel_depth_4 && valid_data)
			expandPixelDepth4(bptr, dptr, dsize, 0);
		else
			copy2LimaBuffer(bptr, dptr, 0);
		return;
	}

	if (frame - m_last_proc_frame > m_nb_buffers) {
		if (!m_overrun) {
			DEB_ERROR() << "OVERRUN: " 
				    << DEB_VAR5(m_last_proc_frame, frame,
						m_recv_idx, m_port,
						m_nb_buffers);
			m_overrun = true;
		}
	}

	char *dest = (char *) m_buffer_cb_mgr.getFrameBufferPtr(frame);
	const FrameDim& fdim = m_buffer_cb_mgr.getFrameDim();
	int size = fdim.getMemSize();
	memcpy(dest, dptr, size);

	m_last_recv_frame = frame;
}

void Eiger::RecvPort::expandPixelDepth4(char *dst, char *src, int len4,
					int thread_idx)
{
	DEB_MEMBER_FUNCT();

	bool valid_data = (src != NULL);
	src += m_spo + m_sto * thread_idx;
	unsigned long s = (unsigned long) src;
	if ((s & 15) != 0)
		THROW_HW_ERROR(Error) << "Missaligned src: "
				      << DEB_VAR1(DEB_HEX(s));
	dst += m_dpo + m_dto * thread_idx;
	unsigned long d = (unsigned long) dst;
	int dest_misalign = (d & 15);
	if (m_raw && dest_misalign)
		THROW_HW_ERROR(Error) << "Missaligned dest: "
				      << DEB_VAR1(DEB_HEX(d));
 	const int block_len = sizeof(__m128i);
	int nb_blocks = len4 / block_len;
	const int chip_blocks = ChipSize / 2 / block_len;
	const int xfer_lines = m_raw ? ChipSize : 1;
	const int port_blocks = chip_blocks * m_pchips * xfer_lines;
	DEB_TRACE() << DEB_VAR3(nb_blocks, chip_blocks, port_blocks);
	const __m128i *src128 = (const __m128i *) src;
	__m128i *dst128 = (__m128i *) dst;
	const __m128i m = _mm_set1_epi8(0xf);
	int block_bits = block_len * 8;
	int gap_bits = ChipGap * 8;
	const __m128i block64_bits128 = _mm_set_epi64x(0, 64);
	const __m128i gap_bits128 = _mm_set_epi64x(0, gap_bits);
	int shift_l;
	__m128i shift_l128, shift_r128;
	__m128i m64_0 = _mm_set_epi64x(0, -1);
	__m128i m64_1 = _mm_set_epi64x(-1, 0);
 	bool reverse = (m_slw < 0);
	__m128i m0, prev;
	int chip_count = 0;

#define load_dst128()							\
	do {								\
		dest_misalign = ((unsigned long) dst & 15);		\
		dst128 = (__m128i *) (dst - dest_misalign);		\
		shift_l = dest_misalign * 8;				\
		shift_l128 = _mm_set_epi64x(0, shift_l % 64);		\
		shift_r128 = _mm_sub_epi64(block64_bits128, shift_l128); \
		if (shift_l != 0) {					\
			m0 = _mm_srl_epi64(_mm_set1_epi8(0xff), shift_r128); \
			if (shift_l < 64)				\
				m0 = _mm_and_si128(m0, m64_0);		\
			prev = _mm_and_si128(_mm_load_si128(dst128), m0); \
		} else {						\
			prev = _mm_setzero_si128();			\
		}							\
	} while (0)

#define pad_dst128()							\
	do {								\
		shift_l += gap_bits;					\
		if (shift_l % 64 == 0)					\
			shift_l128 = _mm_setzero_si128();		\
		else							\
			shift_l128 = _mm_add_epi64(shift_l128, gap_bits128); \
		shift_r128 = _mm_sub_epi64(block64_bits128, shift_l128); \
		if (shift_l == block_bits) {				\
			_mm_store_si128(dst128++, prev);		\
			prev = _mm_setzero_si128();			\
			shift_l = 0;					\
		}							\
	} while (0)

#define sync_dst128()							\
	do {								\
		if (shift_l != 0) {					\
			m0 = _mm_sll_epi64(_mm_set1_epi8(0xff), shift_l128); \
			if (shift_l >= 64)				\
				m0 = _mm_and_si128(m0, m64_1);		\
			__m128i a = _mm_and_si128(_mm_load_si128(dst128), m0); \
			_mm_store_si128(dst128, _mm_or_si128(prev, a));	\
		}							\
	} while (0)

	if (!m_raw)
		load_dst128();
	for (int i = 0; i < nb_blocks; ++i) {
		if (i == 0)
			DEB_TRACE() << DEB_VAR2(DEB_HEX((unsigned long) src128),
						DEB_HEX((unsigned long) dst128));
		if (!m_raw && i && (i % chip_blocks == 0) &&
		    ((m_port == 0) || (++chip_count % m_pchips > 0)))
			pad_dst128();
		if (i && (i % port_blocks == 0)) {
			if (reverse)
				src128 -= 2 * port_blocks;
			if (!m_raw)
				sync_dst128();
			dst += m_dlw;
			if (!m_raw)
				load_dst128();
			else
				dst128 = (__m128i *) dst;
		}
		__m128i p4_raw;
		if (valid_data)
			p4_raw = _mm_load_si128(src128);
		else
			p4_raw = _mm_set1_epi8(0xff);
		++src128;
		__m128i p4_shr = _mm_srli_epi16(p4_raw, 4);
		__m128i i8_0 = _mm_and_si128(p4_raw, m);
		__m128i i8_1 = _mm_and_si128(p4_shr, m);
		__m128i p8_0 = _mm_unpacklo_epi8(i8_0, i8_1);
		__m128i p8_1 = _mm_unpackhi_epi8(i8_0, i8_1);
		if (m_raw) {
			_mm_store_si128(dst128++, p8_0);
			_mm_store_si128(dst128++, p8_1);
			continue;
		}
		__m128i p8_0l = _mm_sll_epi64(p8_0, shift_l128);
		__m128i p8_0r = _mm_srl_epi64(p8_0, shift_r128);
		__m128i p8_1l = _mm_sll_epi64(p8_1, shift_l128);
		__m128i p8_1r = _mm_srl_epi64(p8_1, shift_r128);
		__m128i d0, d1, d2, d3, d4;
		if (shift_l < 64) {
			d0 = p8_0l;
			d1 = p8_0r;
			d2 = p8_1l;
			d3 = p8_1r;
			d4 = _mm_setzero_si128();
		} else {
			d0 = _mm_setzero_si128();
			d1 = p8_0l;
			d2 = p8_0r;
			d3 = p8_1l;
			d4 = p8_1r;
		}
		__m128i d10 = _mm_slli_si128(d1, 8);
		__m128i d11 = _mm_srli_si128(d1, 8);
		__m128i d30 = _mm_slli_si128(d3, 8);
		__m128i d31 = _mm_srli_si128(d3, 8);
		prev = _mm_or_si128(prev, d0);
		_mm_store_si128(dst128++, _mm_or_si128(prev, d10));
		prev = _mm_or_si128(d11, d2);
		_mm_store_si128(dst128++, _mm_or_si128(prev, d30));
		prev = _mm_or_si128(d31, d4);
	}
	if (!m_raw)
		sync_dst128();
}

void Eiger::RecvPort::copy2LimaBuffer(char *dst, char *src, int thread_idx)
{
	DEB_MEMBER_FUNCT();

	bool valid_data = (src != NULL);
	src += m_spo + thread_idx * m_sto;
	dst += m_dpo + thread_idx * m_dto;
	for (int i = 0; i < m_copy_lines; ++i, src += m_slw, dst += m_dlw) {
		char *s = src;
		char *d = dst;
		for (int j = 0; j < m_pchips; ++j, s += m_scw, d += m_dcw)
			if (valid_data)
				memcpy(d, s, m_scw);
			else
				memset(d, 0xff, m_scw);
	}
}

int Eiger::RecvPort::getNbPortProcessingThreads()
{
	DEB_MEMBER_FUNCT();
	int nb_proc_threads = m_nb_threads;
	DEB_RETURN() << DEB_VAR1(nb_proc_threads);
	return nb_proc_threads;
}

void Eiger::RecvPort::processPortThread(FrameType frame, char *bptr,
					int thread_idx)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR4(frame, m_recv_idx, m_port, thread_idx);

	if (!m_thread_proc)
		return;

	char *dptr = (char *) m_buffer_cb_mgr.getFrameBufferPtr(frame);
	const FrameDim& fdim = m_buffer_cb_mgr.getFrameDim();
	int size = fdim.getMemSize() / m_nb_threads;
	expandPixelDepth4(bptr, dptr, size, thread_idx);

	// This is not perfect, but should more or less work
	m_last_proc_frame = frame;
}

Eiger::Eiger(Camera *cam)
	: Model(cam, EigerDet), m_fixed_clock_div(false),
	  m_expand_4_in_threads(true)
{
	DEB_CONSTRUCTOR();

	int nb_det_modules = getNbDetModules();
	DEB_TRACE() << "Using Eiger detector, " << DEB_VAR1(nb_det_modules);

	for (int i = 0; i < nb_det_modules; ++i) {
		for (int j = 0; j < NbRecvPorts; ++j) {
			RecvPort *g = new RecvPort(this, i, j);
			m_recv_port_list.push_back(g);
		}
	}

	updateCameraModel();

	getClockDiv(m_clock_div);
}

Eiger::~Eiger()
{
	DEB_DESTRUCTOR();
	removeAllCorr();
}

void Eiger::getFrameDim(FrameDim& frame_dim, bool raw)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR1(raw);
	getRecvFrameDim(frame_dim, raw, true);
	Size size = frame_dim.getSize();
	size *= Point(1, getNbDetModules());
	if (!raw)
		for (int i = 0; i < getNbEigerModules() - 1; ++i)
			size += Point(0, getInterModuleGap(i));
	frame_dim.setSize(size);
	DEB_RETURN() << DEB_VAR1(frame_dim);
}

void Eiger::getRecvFrameDim(FrameDim& frame_dim, bool raw, bool geom)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR1(raw);
	frame_dim.setImageType(getCamera()->getImageType());
	Size size(ChipSize * HalfModuleChips, ChipSize);
	if (raw) {
		size /= Point(NbRecvPorts, 1);
		size *= Point(1, NbRecvPorts);
	} else if (geom) {
		size += Point(ChipGap, ChipGap) * Point(3, 1) / Point(1, 2);
	}
	frame_dim.setSize(size);
	DEB_RETURN() << DEB_VAR1(frame_dim);
}

string Eiger::getName()
{
	DEB_MEMBER_FUNCT();
	ostringstream os;
	os << "PSI/Eiger-";
	int nb_modules = getNbEigerModules();
	if (nb_modules == 1) {
		os << "500k";
	} else if (nb_modules % 2 == 0) {
		os << (nb_modules / 2) << "M";
	} else {
		os << nb_modules << "-Modules";
	}
	string name = os.str();
	DEB_RETURN() << DEB_VAR1(name);
	return name;
}

void Eiger::getPixelSize(double& x_size, double& y_size)
{
	DEB_MEMBER_FUNCT();
	x_size = y_size = 75e-6;
	DEB_RETURN() << DEB_VAR2(x_size, y_size);
}

void Eiger::getDACInfo(NameList& name_list, IntList& idx_list,
		       IntList& milli_volt_list)
{
	DEB_MEMBER_FUNCT();

#define EIGER_DAC(x)			{x, 0}
#define EIGER_DAC_MV(x)			{x, 1}

	static struct DACData {
		DACIndex idx;
		int milli_volt;
	} EigerDACList[] = {
		EIGER_DAC(EigerVrf),
		EIGER_DAC(EigerVrs),
		EIGER_DAC(EigerVtr),
		EIGER_DAC(EigerVcal),
		EIGER_DAC(EigerVcp),
		EIGER_DAC(EigerVcmpLL),
		EIGER_DAC(EigerVcmpLR),
		EIGER_DAC(EigerVcmpRL),
		EIGER_DAC(EigerVcmpRR),
		EIGER_DAC(Threshold),
	};
	const unsigned int size = C_LIST_SIZE(EigerDACList);

	name_list.resize(size);
	idx_list.resize(size);
	milli_volt_list.resize(size);
	struct DACData *data = EigerDACList;
	for (unsigned int i = 0; i < size; ++i, ++data) {
		ostringstream os;
		os << data->idx;
		name_list[i] = os.str();
		idx_list[i] = int(data->idx);
		milli_volt_list[i] = data->milli_volt;
		DEB_RETURN() << DEB_VAR2(name_list[i], idx_list[i]);
	}
}

void Eiger::getADCInfo(NameList& name_list, IntList& idx_list,
		       FloatList& factor_list, FloatList& min_val_list)
{
	DEB_MEMBER_FUNCT();

#define EIGER_TEMP_FACTOR		(1 / 1000.0)

#define EIGER_TEMP(x)			{x, EIGER_TEMP_FACTOR}

	static struct ADCData {
		ADCIndex idx;
		double factor, min_val;
	} EigerADCList[] = {
		EIGER_TEMP(TempFPGA),
		EIGER_TEMP(TempFPGAExt),
		EIGER_TEMP(Temp10GE),
		EIGER_TEMP(TempDCDC),
		EIGER_TEMP(TempSODL),
		EIGER_TEMP(TempSODR),
		EIGER_TEMP(TempFPGAFL),
		EIGER_TEMP(TempFPGAFR),
	};
	const unsigned int size = C_LIST_SIZE(EigerADCList);

	name_list.resize(size);
	idx_list.resize(size);
	factor_list.resize(size);
	min_val_list.resize(size);
	struct ADCData *data = EigerADCList;
	for (unsigned int i = 0; i < size; ++i, ++data) {
		ostringstream os;
		os << data->idx;
		name_list[i] = os.str();
		idx_list[i] = int(data->idx);
		factor_list[i] = data->factor;
		min_val_list[i] = data->min_val;
		DEB_RETURN() << DEB_VAR4(name_list[i], idx_list[i], 
					 factor_list[i], min_val_list[i]);
	}
}

void Eiger::getTimeRanges(TimeRanges& time_ranges)
{
	DEB_MEMBER_FUNCT();

	PixelDepth pixel_depth;
	Camera* cam = getCamera();
	cam->getPixelDepth(pixel_depth);
	ClockDiv clock_div;
	getClockDiv(clock_div);
	ParallelMode parallel_mode;
	getParallelMode(parallel_mode);

	calcTimeRanges(pixel_depth, clock_div, parallel_mode, time_ranges);
}

void Eiger::calcTimeRanges(PixelDepth pixel_depth, ClockDiv clock_div,
			   ParallelMode parallel_mode,
			   TimeRanges& time_ranges)
{
	DEB_STATIC_FUNCT();
	DEB_PARAM() << DEB_VAR3(pixel_depth, clock_div, parallel_mode);

	// times are in usec, freq in MHz
	int period_factor = 1 << int(clock_div);
	double xfer_freq = BaseChipXferFreq / period_factor;
	DEB_TRACE() << DEB_VAR2(period_factor, xfer_freq);

	double xfer_2_buff = ChipXfer2Buff.calcY(period_factor);
	DEB_TRACE() << DEB_VAR1(xfer_2_buff);

	const int super_col_pixels = SuperColNbCols * ChipSize;
	double theo_single_readout = super_col_pixels * BitsPerXfer / xfer_freq;
	// theoretical -> measured correction
	double real_single_readout = ChipRealReadout.calcY(theo_single_readout);
	DEB_TRACE() << DEB_VAR2(theo_single_readout, real_single_readout);

	int readout_cycles;
	if (pixel_depth == PixelDepth4)
		readout_cycles = 1;
	else if (pixel_depth == PixelDepth8)
		readout_cycles = 2;
	else
		readout_cycles = 3;
	double min_chip_readout = real_single_readout * readout_cycles;
	DEB_TRACE() << DEB_VAR2(readout_cycles, min_chip_readout);

	int mem_xfer_blocks = pixel_depth / BitsPerXfer;
	const int nb_super_cols = ChipSize / SuperColNbCols * HalfModuleChips;
	double feb_beb_bw = (xfer_freq * nb_super_cols / readout_cycles *
			     mem_xfer_blocks);
	DEB_TRACE() << DEB_VAR2(feb_beb_bw, MaxFebBebBandwidth);
	double chip_readout = min_chip_readout;
	if (feb_beb_bw > MaxFebBebBandwidth) {
		chip_readout *= feb_beb_bw / MaxFebBebBandwidth;
		DEB_TRACE() << "limiting chip readout freq: "
			    << DEB_VAR2(min_chip_readout, chip_readout);
	}
	double full_readout = xfer_2_buff + chip_readout;
	DEB_TRACE() << DEB_VAR1(full_readout);

	bool parallel = (parallel_mode == Parallel);

	double min_exp = 10;
	double min_period = (parallel ? 0 : min_exp) + full_readout;
	double min_lat = parallel ? xfer_2_buff : full_readout;

	// Timing hardware uses 32-bit, base-10 floating-point registers:
	//   29 bits: mantissa
	//    3 bits: exponent
	// Time unit: 10 nsec 
	// Max value: 2^29 * 10^(2^3 - 1) * 10 nsec = 1.7 years
	// Using 1000 sec as a reasonable upper limit

	time_ranges.min_exp_time = min_exp * 1e-6;
	time_ranges.max_exp_time = 1e3;
	time_ranges.min_lat_time = min_lat * 1e-6;
	time_ranges.max_lat_time = 1e3;
	time_ranges.min_frame_period = min_period * 1e-6;
	time_ranges.max_frame_period = 1e3;

	DEB_RETURN() << DEB_VAR2(time_ranges.min_exp_time, 
				 time_ranges.max_exp_time);
	DEB_RETURN() << DEB_VAR2(time_ranges.min_lat_time, 
				 time_ranges.max_lat_time);
	DEB_RETURN() << DEB_VAR2(time_ranges.min_frame_period, 
				 time_ranges.max_frame_period);
}

void Eiger::updateImageSize()
{
	DEB_MEMBER_FUNCT();

	removeAllCorr();

	createBadRecvFrameCorr();

	Camera *cam = getCamera();

	bool raw;
	cam->getRawMode(raw);
	if (raw)
		return;

	ImageType image_type = getCamera()->getImageType();
	createChipBorderCorr(image_type);

	if (getNbEigerModules() > 1)
		createInterModGapCorr();

	PixelDepth pixel_depth;
	cam->getPixelDepth(pixel_depth);
	if (pixel_depth == PixelDepth32) {
		setClockDiv(QuarterSpeed);
	} else if (m_fixed_clock_div) {
		ClockDiv curr_clock_div;
		getClockDiv(curr_clock_div);
		if (curr_clock_div != m_clock_div) {
			try {
				DEB_ALWAYS() << "Restoring " 
					     << DEB_VAR1(m_clock_div);
				setClockDiv(m_clock_div);
			} catch (...) {
				DEB_WARNING() << "Could not set " 
					      << DEB_VAR1(m_clock_div) << " "
					      << "keeping " 
					      << DEB_VAR1(curr_clock_div);
				m_clock_div = curr_clock_div;
			}
		}
	}
}

bool Eiger::checkSettings(Settings settings)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR1(settings);
	bool ok;
	switch (settings) {
	case Defs::Standard:
		ok = true;
		break;
	default:
		ok = false;
	}

	DEB_RETURN() << DEB_VAR1(ok);
	return ok;
}

void Eiger::setParallelMode(ParallelMode mode)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR1(mode);
	m_det->setParallelMode(mode);
	updateTimeRanges();
}

void Eiger::getParallelMode(ParallelMode& mode)
{
	DEB_MEMBER_FUNCT();
	mode = ParallelMode(m_det->setParallelMode(-1));
	DEB_RETURN() << DEB_VAR1(mode);
}

void Eiger::setFixedClockDiv(bool fixed_clock_div)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR1(fixed_clock_div);
	m_fixed_clock_div = fixed_clock_div;
}

void Eiger::getFixedClockDiv(bool& fixed_clock_div)
{
	DEB_MEMBER_FUNCT();
	fixed_clock_div = m_fixed_clock_div;
	DEB_RETURN() << DEB_VAR1(fixed_clock_div);
}

void Eiger::setClockDiv(ClockDiv clock_div)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR1(clock_div);
	if (clock_div == SuperSlowSpeed)
		THROW_HW_ERROR(NotSupported) << "SuperSlowSpeed not tested yet";
	PixelDepth pixel_depth;
	Camera* cam = getCamera();
	cam->getPixelDepth(pixel_depth);
	if ((pixel_depth == PixelDepth32) && (clock_div != QuarterSpeed))
		THROW_HW_ERROR(InvalidValue) << "32-bit works only on "
					     << "QuarterSpeed";
	m_det->setClockDivider(clock_div);
	m_clock_div = clock_div;
	updateTimeRanges();
}

void Eiger::getClockDiv(ClockDiv& clock_div)
{
	DEB_MEMBER_FUNCT();
	int ret = m_det->setClockDivider(-1);
	if (ret == MultiSlsDetectorErr)
		THROW_HW_ERROR(Error) << "Error getting clock divider";
	clock_div = ClockDiv(ret);
	DEB_RETURN() << DEB_VAR1(clock_div);
}

void Eiger::setSubExpTime(double sub_exp_time)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR1(sub_exp_time);
	if (sub_exp_time != 0)
		THROW_HW_ERROR(NotSupported) << "SubExpTime not supported";
	updateTimeRanges();
}

void Eiger::getSubExpTime(double& sub_exp_time)
{
	DEB_MEMBER_FUNCT();
	sub_exp_time = 0;
	DEB_RETURN() << DEB_VAR1(sub_exp_time);
}

void Eiger::setAllTrimBits(int sub_mod_idx, int val)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR2(sub_mod_idx, val);

	if ((sub_mod_idx < -1) || (sub_mod_idx >= getNbDetSubModules()))
		THROW_HW_ERROR(InvalidValue) << DEB_VAR1(sub_mod_idx);

	int ret = m_det->setAllTrimbits(val, sub_mod_idx);
	if (ret == MultiSlsDetectorErr)
		THROW_HW_ERROR(Error) << "Error setting all trim bits"
				      << " on (sub)module " << sub_mod_idx;
}

void Eiger::getAllTrimBits(int sub_mod_idx, int& val)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR1(sub_mod_idx);

	if ((sub_mod_idx < 0) || (sub_mod_idx >= getNbDetSubModules()))
		THROW_HW_ERROR(InvalidValue) << DEB_VAR1(sub_mod_idx);

	int ret = m_det->setAllTrimbits(-1, sub_mod_idx);
	if (ret == MultiSlsDetectorErr)
		THROW_HW_ERROR(Error) << "Error getting all trim bits"
				      << " on (sub)module " << sub_mod_idx;
	val = ret;
	DEB_RETURN() << DEB_VAR1(val);
}

void Eiger::getAllTrimBitsList(IntList& val_list)
{
	DEB_MEMBER_FUNCT();
	int nb_sub_modules = getNbDetSubModules();
	val_list.resize(nb_sub_modules);
	for (int i = 0; i < nb_sub_modules; ++i)
		getAllTrimBits(i, val_list[i]);
}

void Eiger::setHighVoltage(int hvolt)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR1(hvolt);
	m_det->setHighVoltage(hvolt);
}

void Eiger::getHighVoltage(int& hvolt)
{
	DEB_MEMBER_FUNCT();
	hvolt = m_det->setHighVoltage(-1);
	DEB_RETURN() << DEB_VAR1(hvolt);
}

void Eiger::setThresholdEnergy(int thres)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR1(thres);
	m_det->setThresholdEnergy(thres);
}

void Eiger::getThresholdEnergy(int& thres)
{
	DEB_MEMBER_FUNCT();
	thres = m_det->getThresholdEnergy();
	DEB_RETURN() << DEB_VAR1(thres);
}

void Eiger::setExpand4InThreads(bool expand_4_in_threads)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR1(expand_4_in_threads);
	m_expand_4_in_threads = expand_4_in_threads;
}

void Eiger::getExpand4InThreads(bool& expand_4_in_threads)
{
	DEB_MEMBER_FUNCT();
	expand_4_in_threads = m_expand_4_in_threads;
	DEB_RETURN() << DEB_VAR1(expand_4_in_threads);
}

int Eiger::getNbRecvPorts()
{
	DEB_MEMBER_FUNCT();
	DEB_RETURN() << DEB_VAR1(NbRecvPorts);
	return NbRecvPorts;
}

Model::RecvPort *Eiger::getRecvPort(int port_idx)
{
	DEB_MEMBER_FUNCT();
	return m_recv_port_list[port_idx];
}

void Eiger::prepareAcq()
{
	DEB_MEMBER_FUNCT();

	bool raw;
	getCamera()->getRawMode(raw);
	getRecvFrameDim(m_recv_frame_dim, raw, true);
	
	DEB_TRACE() << DEB_VAR2(raw, m_recv_frame_dim);

	RecvPortList::iterator git, gend = m_recv_port_list.end();
	for (git = m_recv_port_list.begin(); git != gend; ++git)
		(*git)->prepareAcq();

	CorrList::iterator cit, cend = m_corr_list.end();
	for (cit = m_corr_list.begin(); cit != cend; ++cit)
		(*cit)->prepareAcq();
}

Eiger::Correction *Eiger::createCorrectionTask()
{
	DEB_MEMBER_FUNCT();
	return new Correction(this);
}

Eiger::CorrBase *Eiger::createBadRecvFrameCorr()
{
	DEB_MEMBER_FUNCT();
	CorrBase *brf_corr = new BadRecvFrameCorr(this);
	addCorr(brf_corr);
	DEB_RETURN() << DEB_VAR1(brf_corr);
	return brf_corr;
}

Eiger::CorrBase *Eiger::createChipBorderCorr(ImageType image_type)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR1(image_type);

	CorrBase *border_corr;
	switch (image_type) {
	case Bpp8:
		border_corr = new ChipBorderCorr<Byte>(this);
		break;
	case Bpp16:
		border_corr = new ChipBorderCorr<Word>(this);
		break;
	case Bpp32:
		border_corr = new ChipBorderCorr<Long>(this);
		break;
	default:
		THROW_HW_ERROR(NotSupported) 
			<< "Eiger correction not supported for " << image_type;
	}

	addCorr(border_corr);

	DEB_RETURN() << DEB_VAR1(border_corr);
	return border_corr;
}

Eiger::CorrBase *Eiger::createInterModGapCorr()
{
	DEB_MEMBER_FUNCT();
	CorrBase *gap_corr = new InterModGapCorr(this);
	addCorr(gap_corr);
	DEB_RETURN() << DEB_VAR1(gap_corr);
	return gap_corr;
}

void Eiger::addCorr(CorrBase *corr)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR1(corr);
	m_corr_list.push_back(corr);
}

void Eiger::removeCorr(CorrBase *corr)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR1(corr);

	CorrList::iterator it, end = m_corr_list.end();
	it = find(m_corr_list.begin(), end, corr);
	if (it != end)
		m_corr_list.erase(it);
	corr->m_eiger = NULL;
}

void Eiger::removeAllCorr()
{
	DEB_MEMBER_FUNCT();
	CorrList::reverse_iterator it, end = m_corr_list.rend();
	for (it = m_corr_list.rbegin(); it != end; ++it)
		delete *it;
	if (m_corr_list.size() > 0)
		THROW_HW_ERROR(Error) << "Correction list not empty!";
}

double Eiger::getBorderCorrFactor(int det, int line)
{
	DEB_MEMBER_FUNCT();
	switch (line) {
	case 0: return 2.0;
	case 1: return 1.3;
	default: return 1;
	}
}

int Eiger::getInterModuleGap(int det)
{
	DEB_MEMBER_FUNCT();
	if (det >= getNbEigerModules() - 1)
		THROW_HW_ERROR(InvalidValue) << "Invalid " << DEB_VAR1(det);
	return 36;
}

ostream& lima::SlsDetector::operator <<(ostream& os, Eiger::ParallelMode mode)
{
	const char *name = "Invalid";
	switch (mode) {
	case Eiger::NonParallel:	name = "NonParallel";	break;
	case Eiger::Parallel:		name = "Parallel";	break;
	case Eiger::Safe:		name = "Safe";		break;
	}
	return os << name;
}

