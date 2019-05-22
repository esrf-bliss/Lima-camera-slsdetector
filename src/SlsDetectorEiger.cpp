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

	const FrameDim& recv_dim = m_eiger->getRecvFrameDim();
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
	m_nb_recvs = m_eiger->getNbRecvs();
	m_bfd_list.resize(m_nb_recvs);
}

void Eiger::BadRecvFrameCorr::prepareAcq()
{
	DEB_MEMBER_FUNCT();

	CorrBase::prepareAcq();

	for (int i = 0; i < m_nb_recvs; ++i)
		m_bfd_list[i].reset();
}

void Eiger::BadRecvFrameCorr::correctFrame(FrameType frame, void *ptr)
{
	DEB_MEMBER_FUNCT();

	char *bptr = (char *) ptr;
	int nb_recv_ports = m_eiger->getNbRecvPorts();
	for (int i = 0; i < m_nb_recvs; ++i) {
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
			Eiger::Geometry *geom = m_eiger->getGeometry();
			Eiger::Geometry::Recv *recv = geom->getRecv(i);
			recv->fillBadFrame(frame, bptr);
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

Eiger::Geometry::Recv::Port::Port(Recv *recv, int port)
	: m_recv(recv), m_recv_idx(m_recv->m_idx), m_port(port)
{
	DEB_CONSTRUCTOR();
	DEB_PARAM() << DEB_VAR2(m_recv_idx, m_port);
	m_top_half_recv = (m_recv_idx % 2 == 0);
}

void Eiger::Geometry::Recv::Port::prepareAcq()
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR1(m_recv_idx);

	Geometry *geom = m_recv->m_eiger_geom;
	m_raw = geom->m_raw;
	const FrameDim& frame_dim = geom->m_recv_frame_dim;
	const Size& size = frame_dim.getSize();
	int depth = frame_dim.getDepth();
	m_dst.lw = size.getWidth() * depth;
	int recv_size = frame_dim.getMemSize();
	int src_port_offset = 0;
	int dst_port_offset = recv_size * m_recv_idx;

	m_pchips = HalfModuleChips / NbRecvPorts;
	m_src.cw = ChipSize * depth;
	m_dst.cw = m_src.cw;
	if (m_recv->m_pixel_depth_4)
		m_src.cw /= 2;
	m_src.lw = m_pchips * m_src.cw;

	if (m_raw) {
		// vert. port concat.
		dst_port_offset += ChipSize * m_dst.lw * m_port;
	} else {
		// inter-chip horz. gap
		m_dst.cw += ChipGap * depth;
		// horz. port concat.
		dst_port_offset += m_pchips * m_dst.cw * m_port;

		int mod_idx = m_recv_idx / 2;
		for (int i = 0; i < mod_idx; ++i)
			dst_port_offset += (geom->getInterModuleGap(i) *
					    m_dst.lw);
		if (m_top_half_recv) {
			// top-half module: vert-flipped data
			src_port_offset += (ChipSize - 1) * m_src.lw;
			m_src.lw *= -1;
		} else {
			// bottom-half module: inter-chip vert. gap
			dst_port_offset += (ChipGap / 2) * m_dst.lw;
		}
	}

	int copy_lines = ChipSize;
	if (m_raw)
		copy_lines *= NbRecvPorts;

	int nb_threads = m_recv->m_nb_proc_threads;
	ThreadBalance& thread_bal = m_recv->m_thread_bal;
	if (thread_bal.empty()) {
		thread_bal = m_recv->getDefaultThreadBalance(nb_threads);
		DEB_ALWAYS() << PrettyIntList(thread_bal);
	}
	if (thread_bal.size() != nb_threads)
		THROW_HW_ERROR(InvalidValue) <<
			DEB_VAR2(thread_bal.size(), nb_threads);
	int tot_thread_bal = 0;
	for (int i = 0; i < nb_threads; ++i)
		tot_thread_bal += thread_bal[i];
	if (copy_lines % tot_thread_bal != 0)
		THROW_HW_ERROR(InvalidValue) << "tot_thread_bal";

	int thread_base_lines = copy_lines / tot_thread_bal;
	DEB_TRACE() << DEB_VAR2(copy_lines, thread_base_lines);

	const int block_len = sizeof(__m128i);
	const int chip_blocks = ChipSize / 2 / block_len;
	m_raw_port_blocks = chip_blocks * m_pchips * ChipSize;

	int first_port = 0;
	int first_line = 0;
	ThreadData *td = m_td;
	for (int i = 0; i < nb_threads; ++i, ++td) {
		int line = (m_raw && m_port > first_port) ? 0 : first_line;
		const int remaining_lines = ChipSize - line;
		const int port_lines = m_raw ? remaining_lines : 1;
		td->port_blocks = chip_blocks * m_pchips * port_lines;
		DEB_TRACE() << DEB_VAR2(port_lines, td->port_blocks);
		td->xfer_lines = thread_base_lines * thread_bal[i];
		const int xfer_ports = m_raw ? 1 : NbRecvPorts;
		DEB_TRACE() << DEB_VAR2(thread_bal[i], td->xfer_lines);
		td->src_len = td->xfer_lines * abs(m_src.lw) * xfer_ports;
		td->first_port = first_port;
		td->src_offset = src_port_offset + line * m_src.lw;
		td->dst_offset = dst_port_offset + line * m_dst.lw;
		DEB_TRACE() << DEB_VAR4(td->src_len, td->first_port,
					td->src_offset, td->dst_offset);
		first_line += td->xfer_lines;
		if (m_raw) {
			first_port += first_line / ChipSize;
			first_line %= ChipSize;
		}
	}
}

FrameDim Eiger::Geometry::Recv::Port::getSrcFrameDim()
{
	DEB_MEMBER_FUNCT();
	FrameDim fdim(abs(m_src.lw), ChipSize, Bpp8);
	DEB_RETURN() << DEB_VAR1(fdim);
	return fdim;
}

void Eiger::Geometry::Recv::Port::copy(char *dst, char *src, int thread_idx)
{
	DEB_MEMBER_FUNCT();

	bool valid_data = (src != NULL);
	ThreadData *td = &m_td[thread_idx];
	src += td->src_offset;
	dst += td->dst_offset;

	const int& lines = td->xfer_lines;
	if (m_raw) {
		int size = m_src.lw * lines;
		if (valid_data)
			memcpy(dst, src, size);
		else
			memset(dst, 0xff, size);
		return;
	}

	for (int i = 0; i < lines; ++i, src += m_src.lw, dst += m_dst.lw) {
		char *s = src;
		char *d = dst;
		for (int j = 0; j < m_pchips; ++j, s += m_src.cw, d += m_dst.cw)
			if (valid_data)
				memcpy(d, s, m_src.cw);
			else
				memset(d, 0xff, m_src.cw);
	}
}

Eiger::Geometry::Recv::Recv(Geometry *eiger_geom, int idx)
	: m_eiger_geom(eiger_geom), m_idx(idx), m_nb_proc_threads(1)
{
	DEB_CONSTRUCTOR();
	DEB_PARAM() << DEB_VAR1(m_idx);

	for (int i = 0; i < NbRecvPorts; ++i) {
		Port *p = new Port(this, i);
		m_port_list.push_back(p);
	}
}

int Eiger::Geometry::Recv::getNbPorts()
{
	DEB_MEMBER_FUNCT();
	DEB_RETURN() << DEB_VAR1(NbRecvPorts);
	return NbRecvPorts;
}

Eiger::Geometry::Recv::Port *Eiger::Geometry::Recv::getPort(int port_idx)
{
	DEB_MEMBER_FUNCT();
	return m_port_list[port_idx];
}

Eiger::ThreadBalance
Eiger::Geometry::Recv::getDefaultThreadBalance(int nb_threads)
{
	DEB_STATIC_FUNCT();

#define append(d, s) d.insert(d.end(), s.begin(), s.end())

	ThreadBalance thread_bal;
	static ThreadBalance b3 {5, 5, 6};
	static ThreadBalance b5 {3, 3, 3, 3, 4};
	static ThreadBalance b7 {2, 2, 2, 3, 2, 2, 3};
	static ThreadBalance b9 {4, 3, 4, 3, 4, 3, 4, 3, 4};
	switch (nb_threads) {
	case 1:
	case 2:
	case 4:
	case 8:
		thread_bal = ThreadBalance(nb_threads, 1);
		break;
	case 6:
		append(thread_bal, b3);
	case 3:
		append(thread_bal, b3);
		break;
	case 10:
		append(thread_bal, b5);
	case 5:
		append(thread_bal, b5);
		break;
	case 7:
		thread_bal = b7;
		break;
	case 9:
		thread_bal = b9;
		break;
	default:
		THROW_HW_ERROR(NotSupported) << DEB_VAR1(nb_threads);
	}

#undef append

	return thread_bal;
}

void Eiger::Geometry::Recv::prepareAcq()
{
	DEB_MEMBER_FUNCT();

	m_pixel_depth_4 = (m_eiger_geom->m_pixel_depth == PixelDepth4);

	PortList::iterator it, end = m_port_list.end();
	for (it = m_port_list.begin(); it != end; ++it)
		(*it)->prepareAcq();
}

void Eiger::Geometry::Recv::copy(const FrameData& data, int thread_idx)
{
	DEB_MEMBER_FUNCT();
	for (int i = 0; i < getNbPorts(); ++i)
		m_port_list[i]->copy(data.dst, data.src[i], thread_idx);
}

void Eiger::Geometry::Recv::fillBadFrame(FrameType frame, char *bptr)
{
	DEB_MEMBER_FUNCT();
	FrameData data;
	for (int i = 0; i < getNbPorts(); ++i)
		data.src[i] = NULL;
	data.dst = bptr;
	for (int i = 0; i < m_nb_proc_threads; ++i)
		processFrame(data, i);
}

void Eiger::Geometry::Recv::expandPixelDepth4(const FrameData& data,
					      int thread_idx)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR3(DEB_HEX((unsigned long) data.src[0]),
				DEB_HEX((unsigned long) data.src[1]),
				DEB_HEX((unsigned long) data.dst));

	const int nb_ports = NbRecvPorts;
	Port *port = m_port_list[0];
	Port::ThreadData *td = &port->m_td[thread_idx];
	const bool raw = port->m_raw;
	const int first_port = td->first_port;
	int pi;
	char *src[MaxEigerNbPorts];
	for (pi = first_port; pi < nb_ports; ++pi) {
		td = &m_port_list[pi]->m_td[thread_idx];
		src[pi] = data.src[pi] + td->src_offset;
		unsigned long s = (unsigned long) src[pi];
		if ((s & 15) != 0)
			THROW_HW_ERROR(Error) << "Missaligned src: "
					      << DEB_VAR1(DEB_HEX(s));
	}
	pi = first_port;
	td = &m_port_list[pi]->m_td[thread_idx];
	char *dst = data.dst + td->dst_offset;
	unsigned long d = (unsigned long) dst;
	int dest_misalign = (d & 15);
	if (raw && dest_misalign)
		THROW_HW_ERROR(Error) << "Missaligned dest: "
				      << DEB_VAR1(DEB_HEX(d));

	DEB_TRACE() << DEB_VAR2(thread_idx, dest_misalign);
	DEB_TRACE() << DEB_VAR3(DEB_HEX((unsigned long) src[0]),
				DEB_HEX((unsigned long) src[1]),
				DEB_HEX((unsigned long) dst));
	bool valid_data = data.valid.test(pi);
	const int block_len = sizeof(__m128i);
	int nb_blocks = td->src_len / block_len;
	const int chip_blocks = ChipSize / 2 / block_len;
	int port_blocks = td->port_blocks;
	const __m128i *src128 = (const __m128i *) src[pi];
	__m128i *dst128 = (__m128i *) dst;
	const __m128i m = _mm_set1_epi8(0xf);
	const int block_bits = block_len * 8;
	const int gap_bits = ChipGap * 8;
	const __m128i block64_bits128 = _mm_set_epi64x(0, 64);
	const __m128i gap_bits128 = _mm_set_epi64x(0, gap_bits);
	int shift_l;
	__m128i shift_l128, shift_r128;
	__m128i m64_0 = _mm_set_epi64x(0, -1);
	__m128i m64_1 = _mm_set_epi64x(-1, 0);
 	bool reverse = (port->m_src.lw < 0);
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

	if (!raw)
		load_dst128();
	for (int i = 0; i < nb_blocks; ++i) {
		if (i == 0)
			DEB_TRACE() << DEB_VAR3(DEB_HEX((unsigned long) src128),
						DEB_HEX((unsigned long) dst128),
						nb_blocks);
		if (!raw && i && (i % chip_blocks == 0) &&
		    (++chip_count % HalfModuleChips > 0))
			pad_dst128();
		if (i && (i % port_blocks == 0)) {
			if (reverse)
				src128 -= 2 * port_blocks;
			src[pi++] = (char *) src128;
			pi %= nb_ports;
			valid_data = data.valid.test(pi);
			src128 = (const __m128i *) src[pi];
			if (raw)
				port_blocks = port->m_raw_port_blocks;
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
		if (raw) {
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
	if (!raw)
		sync_dst128();
}

Eiger::Geometry::Geometry()
	: m_raw(false), m_image_type(Bpp16), m_pixel_depth(PixelDepth16)
{
	DEB_CONSTRUCTOR();
}

int Eiger::Geometry::getInterModuleGap(int det)
{
	DEB_MEMBER_FUNCT();
	if (det >= getNbEigerModules() - 1)
		THROW_HW_ERROR(InvalidValue) << "Invalid " << DEB_VAR1(det);
	return 36;
}

void Eiger::Geometry::setNbRecvs(int nb_recv)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR1(nb_recv);

	int curr_nb_recv = m_recv_list.size();
	if (curr_nb_recv > nb_recv) {
		m_recv_list.resize(nb_recv);
		return;
	}

	for (int i = curr_nb_recv; i < nb_recv; ++i) {
		Recv *r = new Recv(this, i);
		m_recv_list.push_back(r);
	}
}

void Eiger::Geometry::prepareAcq()
{
	DEB_MEMBER_FUNCT();

	m_recv_frame_dim = getRecvFrameDim(m_raw);

	RecvList::iterator rit, rend = m_recv_list.end();
	for (rit = m_recv_list.begin(); rit != rend; ++rit)
		(*rit)->prepareAcq();
}

FrameDim Eiger::Geometry::getFrameDim(bool raw)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR1(raw);
	FrameDim frame_dim = getRecvFrameDim(raw);
	Size size = frame_dim.getSize();
	size *= Point(1, getNbRecvs());
	if (!raw)
		for (int i = 0; i < getNbEigerModules() - 1; ++i)
			size += Point(0, getInterModuleGap(i));
	frame_dim.setSize(size);
	DEB_RETURN() << DEB_VAR1(frame_dim);
	return frame_dim;
}

FrameDim Eiger::Geometry::getRecvFrameDim(bool raw)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR1(raw);
	FrameDim frame_dim;
	frame_dim.setImageType(m_image_type);
	Size size(ChipSize * HalfModuleChips, ChipSize);
	if (raw) {
		size /= Point(NbRecvPorts, 1);
		size *= Point(1, NbRecvPorts);
	} else {
		size += Point(ChipGap, ChipGap) * Point(3, 1) / Point(1, 2);
	}
	frame_dim.setSize(size);
	DEB_RETURN() << DEB_VAR1(frame_dim);
	return frame_dim;
}

Eiger::Recv::Port::Port(Recv *recv, int port)
	: m_recv(recv), m_port(port), m_sync(m_recv, m_port)
{
	DEB_CONSTRUCTOR();
	DEB_PARAM() << DEB_VAR2(m_recv->m_idx, m_port);
	m_geom = m_recv->m_geom->getPort(m_port);
}

void Eiger::Recv::Port::prepareAcq()
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR2(m_recv->m_idx, m_port);
	m_sync.prepareAcq();
}

void Eiger::Recv::Port::stopAcq()
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR2(m_recv->m_idx, m_port);
	m_sync.stopAcq();
}

void Eiger::Recv::Port::processFrame(FrameType frame, char *dptr,
				     uint32_t dsize, char *bptr)
{
	DEB_MEMBER_FUNCT();
	bool valid_data = (dptr != NULL);
	DEB_PARAM() << DEB_VAR4(frame, m_recv->m_idx, m_port, valid_data);
	while (!triggerProcess(frame, dptr, dsize, bptr))
		Sleep(10e-6);
	waitProcess();
}

Eiger::Recv::Thread::~Thread()
{
	DEB_DESTRUCTOR();

	if (!hasStarted())
		return;

	AutoMutex l = lockThread();
	m_end = true;
	signalThread();
}

void Eiger::Recv::Thread::init(Recv *recv, int idx)
{
	DEB_MEMBER_FUNCT();
	m_recv = recv;
	m_idx = idx;
	m_data = NULL;

	start();
}

void Eiger::Recv::Thread::start()
{
	DEB_MEMBER_FUNCT();

	AutoMutex l = lockThread();

	m_end = true;
	lima::Thread::start();

	struct sched_param param;
	param.sched_priority = 50;
	int ret = pthread_setschedparam(m_thread, SCHED_RR, &param);
	if (ret != 0)
		DEB_ERROR() << "Could not set real-time priority!!";

	while (m_end)
		waitThread();
}

void Eiger::Recv::Thread::threadFunction()
{
	DEB_MEMBER_FUNCT();

	AutoMutex l = lockThread();
	m_end = false;
	signalThread();
	while (!m_end) {
		while (!m_data)
			waitThread();
		const FrameData& data = *m_data;
		m_data = NULL;
		AutoMutexUnlock ul(l);
		Geometry::Recv *geom = m_recv->m_geom;
		geom->processFrame(data, m_idx);
		m_recv->updateProcessingFrame();
	}
}

Eiger::Recv::Recv(Eiger *eiger, int idx)
	: m_eiger(eiger), m_idx(idx)
{
	DEB_CONSTRUCTOR();
	DEB_PARAM() << DEB_VAR1(m_idx);

	m_geom = m_eiger->m_geom.getRecv(m_idx);

	for (int i = 0; i < NbRecvPorts; ++i) {
		Port *p = new Port(this, i);
		m_port_list.push_back(p);
	}

	setNbProcessingThreads(1);
}

int Eiger::Recv::getNbPorts()
{
	DEB_MEMBER_FUNCT();
	int nb_ports = m_geom->getNbPorts();
	DEB_RETURN() << DEB_VAR1(nb_ports);
	return nb_ports;
}

Eiger::Recv::Port *Eiger::Recv::getPort(int port_idx)
{
	DEB_MEMBER_FUNCT();
	return m_port_list[port_idx];
}

void Eiger::Recv::prepareAcq()
{
	DEB_MEMBER_FUNCT();

	m_frame = 0;
	m_nb_ready_threads = 0;
	m_frame_data.valid.reset();

	m_geom->prepareAcq();

	PortList::iterator it, end = m_port_list.end();
	for (it = m_port_list.begin(); it != end; ++it)
		(*it)->prepareAcq();
}

void Eiger::Recv::startAcq()
{
	DEB_MEMBER_FUNCT();
}

void Eiger::Recv::stopAcq()
{
	DEB_MEMBER_FUNCT();
	PortList::iterator it, end = m_port_list.end();
	for (it = m_port_list.begin(); it != end; ++it)
		(*it)->stopAcq();
}

void Eiger::Recv::processFileStart(uint32_t dsize)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR2(m_idx, dsize);
}

int Eiger::Recv::getNbProcessingThreads()
{
	DEB_MEMBER_FUNCT();
	int nb_proc_threads = m_thread_list.size();
	DEB_RETURN() << DEB_VAR1(nb_proc_threads);
	return nb_proc_threads;
}

void Eiger::Recv::setNbProcessingThreads(int nb_proc_threads)
{
	DEB_MEMBER_FUNCT();
	int curr_nb_proc_threads = m_thread_list.size();
	DEB_PARAM() << DEB_VAR2(curr_nb_proc_threads, nb_proc_threads);

	if (nb_proc_threads < 1)
		THROW_HW_ERROR(InvalidValue) << DEB_VAR1(nb_proc_threads);
	if (nb_proc_threads == curr_nb_proc_threads)
		return;

	if (nb_proc_threads < curr_nb_proc_threads)
		m_thread_list.resize(nb_proc_threads);

	for (int i = curr_nb_proc_threads; i < nb_proc_threads; ++i) {
		Thread *t = new Thread();
		t->init(this, i);
		m_thread_list.push_back(t);
	}

	m_geom->setNbProcessingThreads(nb_proc_threads);
}

void Eiger::Recv::updatePortFrame(Port::Sync *sync)
{
	DEB_MEMBER_FUNCT();

	int port = sync->port;
	DEB_TRACE() << DEB_VAR3(m_frame, port, sync->frame);

	bool ok = ((sync->frame == m_frame) && sync->src);
	m_frame_data.src[port] = sync->src;

	if (m_nb_ready_threads == 0)
		m_candidate = sync->frame;
	else
		m_candidate = min(m_candidate, sync->frame);

	m_frame_data.valid.set(port, ok);
	if (++m_nb_ready_threads < NbRecvPorts)
		return;

	if (m_candidate != m_frame) {
		// next frame is not the expected one
		m_frame = m_candidate;
		for (int i = 0; i < NbRecvPorts; ++i) {
			Port::Sync *sync = &m_port_list[i]->m_sync;
			bool ok = ((sync->frame == m_frame) && sync->src);
			m_frame_data.valid.set(i, ok);
		}
	}

	m_frame_map_item->checkFinishedFrame(m_frame);

	m_frame_data.dst = sync->dst;
	m_nb_ready_threads = 0;

	ThreadList::iterator tit, tend = m_thread_list.end();
	for (tit = m_thread_list.begin(); tit != tend; ++tit)
		(*tit)->addNewFrame(m_frame_data);
}

void Eiger::Recv::updateProcessingFrame()
{
	DEB_MEMBER_FUNCT();

	AutoMutex l = lockPort();
	DEB_TRACE() << DEB_VAR2(m_frame, m_nb_ready_threads);
	if (++m_nb_ready_threads < m_thread_list.size())
		return;

	m_frame_data.valid.reset();
	m_nb_ready_threads = 0;

	for (int i = 0; i < NbRecvPorts; ++i) {
		Port::Sync *sync = &m_port_list[i]->m_sync;
		sync->waiting = (sync->frame > m_frame);
		if (sync->waiting) {
			if (m_nb_ready_threads == 0)
				m_candidate = sync->frame;
			else
				m_candidate = min(m_candidate, sync->frame);
			++m_nb_ready_threads;
		}
		bool ok = ((sync->frame == m_frame + 1) && sync->src);
		m_frame_data.valid.set(i, ok);
		m_frame_data.src[i] = sync->src;
	}
	FrameType frame = m_frame++;
	broadcastPort();
	l.unlock();

	FinishInfo finfo = m_frame_map_item->frameFinished(frame, true, true);
	m_eiger->processFinishInfo(finfo);
}

void Eiger::Recv::processBadFrame(FrameType frame, char *bptr)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR2(m_idx, frame);

	class AutoProcess
	{
		DEB_CLASS_NAMESPC(DebModCamera,
				  "Eiger::Recv::processBadFrame::AutoProcess",
				  "SlsDetector");
	public:
		AutoProcess(Port *port, FrameType frame, char *bptr)
			: m_port(port), m_waited(false)
		{
			DEB_CONSTRUCTOR();
			if (!m_port->triggerProcess(frame, NULL, 0, bptr))
				THROW_HW_ERROR(Error)
					<< "cannot process bad frame: "
					<< "receiving data";
		}

		~AutoProcess()
		{
			DEB_DESTRUCTOR();
			if (!m_waited)
				m_port->triggerProcessCleanUp();
		}

		void waitProcess()
		{
			m_port->waitProcess();
			m_waited = true;
		}

	private:
		Port *m_port;
		bool m_waited;
	};

	typedef std::vector<AutoPtr<AutoProcess> > AutoProcessList;
	AutoProcessList proc_list;
	PortList::iterator it, end = m_port_list.end();
	for (it = m_port_list.begin(); it != end; ++it)
		proc_list.push_back(new AutoProcess(*it, frame, bptr));
	AutoProcessList::iterator pit, pend = proc_list.end();
	for (pit = proc_list.begin(); pit != pend; ++pit)
		(*pit)->waitProcess();
}

Eiger::Eiger(Camera *cam)
	: Model(cam, EigerDet), m_fixed_clock_div(false)
{
	DEB_CONSTRUCTOR();

	int nb_det_modules = getNbDetModules();
	DEB_TRACE() << "Using Eiger detector, " << DEB_VAR1(nb_det_modules);

	m_geom.setNbRecvs(nb_det_modules);

	for (int i = 0; i < nb_det_modules; ++i) {
		Recv *r = new Recv(this, i);
		m_recv_list.push_back(r);
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
	frame_dim = m_geom.getFrameDim(raw);
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

	double readout_time;
	measureReadoutTime(readout_time);
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

void Eiger::measureReadoutTime(double& readout_time)
{
	DEB_MEMBER_FUNCT();

	Camera* cam = getCamera();

	class SyncParams
	{


	private:
		double prev_exp, prev_period;
		TrigMode prev_trig_mode;
	};

	// exp, period, trigmode, nb_frames
	double prev_exp, prev_period;
	cam->getExpTime(prev_exp);
	cam->getFramePeriod(prev_period);
	FrameType prev_nb_frames;
	cam->getNbFrames(prev_nb_frames);
	Defs::TrigMode prev_trig_mode;
	cam->getTrigMode(prev_trig_mode);


	/*
	DEB_ALWAYS() << "calling startAcquisition";
	cam->m_det->startAcquisition();

	DEB_ALWAYS() << "calling stopAcquisition";
	cam->m_det->stopAcquisition();

	 */
}

int Eiger::getNbFrameMapItems()
{
	DEB_MEMBER_FUNCT();
	int nb_items = getNbRecvs();
	DEB_RETURN() << DEB_VAR1(nb_items);
	return nb_items;
}

void Eiger::updateFrameMapItems(FrameMap *map)
{
	DEB_MEMBER_FUNCT();
	for (int i = 0; i < m_recv_list.size(); ++i)
		m_recv_list[i]->updateFrameMapItem(map->getItem(i));
}

void Eiger::processBadItemFrame(FrameType frame, int item, char *bptr)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR2(frame, item);
	m_recv_list[item]->processBadFrame(frame, bptr);
}

void Eiger::updateImageSize()
{
	DEB_MEMBER_FUNCT();

	removeAllCorr();

	createBadRecvFrameCorr();

	Camera *cam = getCamera();

	bool raw;
	cam->getRawMode(raw);
	ImageType image_type = cam->getImageType();
	PixelDepth pixel_depth;
	cam->getPixelDepth(pixel_depth);

	DEB_TRACE() << DEB_VAR3(raw, image_type, pixel_depth);

	m_geom.setRaw(raw);
	m_geom.setPixelDepth(pixel_depth);
	m_geom.setImageType(image_type);
	m_geom.prepareAcq();

	if (raw)
		return;

	createChipBorderCorr(image_type);

	if (getNbEigerModules() > 1)
		createInterModGapCorr();

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

int Eiger::getNbRecvs()
{
	DEB_MEMBER_FUNCT();
	int nb_recvs = m_recv_list.size();
	DEB_RETURN() << DEB_VAR1(nb_recvs);
	return nb_recvs;
}

Model::Recv *Eiger::getRecv(int recv_idx)
{
	DEB_MEMBER_FUNCT();
	return m_recv_list[recv_idx];
}

void Eiger::prepareAcq()
{
	DEB_MEMBER_FUNCT();

	m_geom.prepareAcq();

	RecvList::iterator rit, rend = m_recv_list.end();
	for (rit = m_recv_list.begin(); rit != rend; ++rit)
		(*rit)->prepareAcq();

	CorrList::iterator cit, cend = m_corr_list.end();
	for (cit = m_corr_list.begin(); cit != cend; ++cit)
		(*cit)->prepareAcq();
}

void Eiger::startAcq()
{
	DEB_MEMBER_FUNCT();
	RecvList::iterator it, end = m_recv_list.end();
	for (it = m_recv_list.begin(); it != end; ++it)
		(*it)->startAcq();
}

void Eiger::stopAcq()
{
	DEB_MEMBER_FUNCT();
	RecvList::iterator it, end = m_recv_list.end();
	for (it = m_recv_list.begin(); it != end; ++it)
		(*it)->stopAcq();
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

