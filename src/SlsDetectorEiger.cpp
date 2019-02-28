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

Eiger::PixelDepth4Corr::PixelDepth4Corr(Eiger *eiger)
	: CorrBase(eiger), m_recv_port_list(eiger->m_recv_port_list)
{
	DEB_CONSTRUCTOR();
}

void Eiger::PixelDepth4Corr::correctFrame(FrameType frame, void *ptr)
{
	DEB_MEMBER_FUNCT();

	RecvPortList::iterator it, end = m_recv_port_list.end();
	for (it = m_recv_port_list.begin(); it != end; ++it)
		(*it)->expandPixelDepth4(frame, (char *) ptr);
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
	: m_eiger(eiger), m_port(port), m_recv_idx(recv_idx)
{
	DEB_CONSTRUCTOR();
	DEB_PARAM() << DEB_VAR1(m_recv_idx);
	m_top_half_recv = (m_recv_idx % 2 == 0);
}

void Eiger::RecvPort::prepareAcq()
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR1(m_recv_idx);

	const FrameDim& frame_dim = m_eiger->m_recv_frame_dim;
	const Size& size = frame_dim.getSize();
	int depth = frame_dim.getDepth();
	m_ilw = size.getWidth() * depth;
	int recv_size = frame_dim.getMemSize();
	m_port_offset = recv_size * m_recv_idx;	

	m_pchips = HalfModuleChips / NbRecvPorts;
	m_scw = ChipSize * depth;
	m_dcw = m_scw;
	if (m_eiger->isPixelDepth4())
		m_scw /= 2;

	m_eiger->getCamera()->getRawMode(m_raw);
	if (m_raw) {
		// vert. port concat.
		m_port_offset += ChipSize * m_ilw * m_port;
		return;
	} else {
		// inter-chip horz. gap
		m_dcw += ChipGap * depth;
		// horz. port concat.
		m_port_offset += m_pchips * m_dcw * m_port;
	}

	int mod_idx = m_recv_idx / 2;
	for (int i = 0; i < mod_idx; ++i)
		m_port_offset += m_eiger->getInterModuleGap(i) * m_ilw;

	if (m_top_half_recv) {
		// top-half module: vert-flipped data
		m_port_offset += (ChipSize - 1) * m_ilw;
		m_ilw *= -1;
	} else {
		// bottom-half module: inter-chip vert. gap
		m_port_offset += (ChipGap / 2) * m_ilw;
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
	char *src = dptr;
	char *dest = bptr + m_port_offset;	
	for (int i = 0; i < ChipSize; ++i, dest += m_ilw) {
		char *d = dest;
		for (int j = 0; j < m_pchips; ++j, src += m_scw, d += m_dcw)
			if (valid_data)
				memcpy(d, src, m_scw);
			else
				memset(d, 0xff, m_scw);
	}
}

void Eiger::RecvPort::expandPixelDepth4(FrameType frame, char *ptr)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR3(frame, m_recv_idx, m_port);

	ptr += m_port_offset;
	for (int i = 0; i < ChipSize; ++i, ptr += m_ilw) {
		char *chip = ptr;
		for (int j = 0; j < m_pchips; ++j, chip += m_dcw) {
			char *src = chip + m_scw;
			char *dest = chip + 2 * m_scw;
			for (int k = 0; k < ChipSize / 2; ++k) {
				unsigned char b = *--src;
				*--dest = b >> 4;
				*--dest = b & 0xf;
			}
		}
	}
}

Eiger::Eiger(Camera *cam)
	: Model(cam, EigerDet), m_fixed_clock_div(false)
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

	if (isPixelDepth4())
		createPixelDepth4Corr();

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

Eiger::CorrBase *Eiger::createPixelDepth4Corr()
{
	DEB_MEMBER_FUNCT();
	CorrBase *pd4_corr = new PixelDepth4Corr(this);
	addCorr(pd4_corr);
	DEB_RETURN() << DEB_VAR1(pd4_corr);
	return pd4_corr;
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

