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

#include "sls/detectors/eiger/Geometry.h"

#include <emmintrin.h>
#include <sched.h>

using namespace std;
using namespace lima;
using namespace lima::SlsDetector;
using namespace lima::SlsDetector::Defs;

#define applyDetGeom(e, f, raw)						\
	using namespace sls::Eiger::Geom;				\
	Defs::xy det_size = e->m_det->getDetectorSize();		\
	auto any_geom = AnyDetGeomFromDetSize({det_size.x, det_size.y}); \
	std::visit([&](auto const &geom) {				\
	    if (raw)							\
		f(geom.raw_geom);					\
	    else							\
		f(geom.asm_wg_geom);					\
	}, any_geom);


const int Eiger::ChipSize = 256;
const int Eiger::ChipGap = 2;
const int Eiger::HalfModuleChips = 4;
const int Eiger::NbRecvPorts = EigerNbRecvPorts;

const int Eiger::BitsPerXfer = 4;
const int Eiger::SuperColNbCols = 8;
const double Eiger::BaseChipXferFreq = 200; // Mbit/s
const double Eiger::MaxFebBebBandwidth = 25600; // Mbit/s
const Eiger::LinScale Eiger::ChipXfer2Buff(2.59, 0.85);
const Eiger::LinScale Eiger::ChipRealReadout(1.074, -4);

const unsigned long Eiger::BebFpgaWritePtrAddr = 0xD10000C4;
const unsigned long Eiger::BebFpgaReadPtrAddr = 0xD10000E4;
const unsigned long Eiger::BebFpgaPtrRange = 0x10000000;

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

	m_mod_frame_dim = m_eiger->getModFrameDim();
	m_frame_size = m_mod_frame_dim.getSize() * Point(1, m_nb_eiger_modules);
	m_inter_lines.resize(m_nb_eiger_modules);
	for (int i = 0; i < m_nb_eiger_modules - 1; ++i) {
		m_inter_lines[i] = m_eiger->getInterModuleGap(i);
		m_frame_size += Point(0, m_inter_lines[i]);
	}
	m_inter_lines[m_nb_eiger_modules - 1] = 0;
}

Data Eiger::ModelReconstruction::processModel(Data& data)
{
	DEB_MEMBER_FUNCT();

	DEB_PARAM() << DEB_VAR4(m_eiger, data.frameNumber, 
				_processingInPlaceFlag, data.data());
	if (!m_eiger)
		return data;

	Data ret = _processingInPlaceFlag ? data : data.copy();

	FrameType frame = ret.frameNumber;
	void *ptr = ret.data();
	CorrList& corr_list = m_eiger->m_corr_list;
	CorrList::iterator it, end = corr_list.end();
	for (it = corr_list.begin(); it != end; ++it)
		(*it)->correctFrame(frame, ptr);

	return ret;
}

Eiger::Beb::Beb(const std::string& host_name)
	: shell(host_name), fpga_mem(shell)
{
}

Eiger::Eiger(Camera *cam)
	: Model(cam, EigerDet), m_signed_image_mode(false),
	  m_fixed_clock_div(false)
{
	DEB_CONSTRUCTOR();

	int nb_det_modules = getNbDetModules();
	DEB_TRACE() << "Using Eiger detector, " << DEB_VAR1(nb_det_modules);

	NameList host_name_list = cam->getHostnameList();
	NameList::const_iterator it, end = host_name_list.end();
	for (it = host_name_list.begin(); it != end; ++it) {
		Beb *beb = new Beb(*it);
		m_beb_list.push_back(beb);
	}

	if (isTenGigabitEthernetEnabled()) {
		DEB_TRACE() << "Forcing 10G Ethernet flow control";
		setFlowControl10G(true);
	}

	m_reconstruction = new ModelReconstruction(this);

	updateCameraModel();

	getClockDiv(m_clock_div);
}

Eiger::~Eiger()
{
	DEB_DESTRUCTOR();
	getCamera()->waitAcqState(Idle);
	m_reconstruction->m_eiger = NULL;
	m_reconstruction->unref();
	removeAllCorr();
}

void Eiger::getFrameDim(FrameDim& frame_dim, bool raw)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR1(raw);

	Size size;
	auto f = [&](auto const &det_geom) {
		size = Size(det_geom.size.x, det_geom.size.y);
		DEB_TRACE() << DEB_VAR1(size);
	};
	applyDetGeom(this, f, raw);

	frame_dim = FrameDim(size, getImageType());
	DEB_RETURN() << DEB_VAR1(frame_dim);
}

FrameDim Eiger::getModFrameDim()
{
	DEB_MEMBER_FUNCT();

	bool raw;
	getCamera()->getRawMode(raw);

	Size size;
	auto f = [&](auto const &det_geom) {
        	auto mod_size = det_geom.mod_geom_size;
		size = Size(mod_size.x, mod_size.y);
		DEB_TRACE() << DEB_VAR1(size);
	};
	applyDetGeom(this, f, raw);

	FrameDim frame_dim(size, getImageType());
	DEB_RETURN() << DEB_VAR1(frame_dim);
	return frame_dim;
}

int Eiger::getInterModuleGap(int det)
{
	return sls::Eiger::Geom::ModGap.y;
}

ImageType Eiger::getImageType()
{
	DEB_MEMBER_FUNCT();

	PixelDepth pixel_depth;
	getCamera()->getPixelDepth(pixel_depth);
	DEB_TRACE() << DEB_VAR1(pixel_depth);

	bool signed_img = m_signed_image_mode;
	ImageType image_type;
	switch (pixel_depth) {
	case PixelDepth4:
	case PixelDepth8:
		image_type = signed_img ? Bpp8S  : Bpp8;	break;
	case PixelDepth16:
		image_type = signed_img ? Bpp16S : Bpp16;	break;
	case PixelDepth32:
		image_type = signed_img ? Bpp32S : Bpp32;	break;
	default:
		THROW_HW_ERROR(InvalidValue) << DEB_VAR1(pixel_depth);
	}

	DEB_RETURN() << DEB_VAR1(image_type);
	return image_type;
}

void Eiger::getDetMap(Data& /*det_map*/)
{
	DEB_MEMBER_FUNCT();
	THROW_HW_ERROR(NotSupported) << "DetMap not implemented yet";
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

#define EIGER_TEMP(x)			{x, EIGER_TEMP_FACTOR, 0}

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

void Eiger::measureReadoutTime(double& /*readout_time*/)
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

	readout_time = 0;
	 */
}

bool Eiger::checkTrigMode(TrigMode trig_mode)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR1(trig_mode);
	bool valid_mode = true;
	DEB_RETURN() << DEB_VAR1(valid_mode);
	return valid_mode;
}

void Eiger::updateImageSize()
{
	DEB_MEMBER_FUNCT();

	removeAllCorr();

	PixelDepth pixel_depth;
	getCamera()->getPixelDepth(pixel_depth);
	DEB_TRACE() << DEB_VAR1(pixel_depth);

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

	bool raw;
	getCamera()->getRawMode(raw);
	DEB_TRACE() << DEB_VAR1(raw);

	if (raw)
		return;

	createChipBorderCorr(getImageType());
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
	EXC_CHECK(m_det->setParallelMode(mode));
	updateCameraTimeRanges();
}

void Eiger::getParallelMode(ParallelMode& mode)
{
	DEB_MEMBER_FUNCT();
	bool sls_mode;
	const char *err_msg = "Parallel mode is different";
	EXC_CHECK(sls_mode = m_det->getParallelMode().tsquash(err_msg));
	mode = ParallelMode(sls_mode);
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
	PixelDepth pixel_depth;
	Camera* cam = getCamera();
	cam->getPixelDepth(pixel_depth);
	if ((pixel_depth == PixelDepth32) && (clock_div != QuarterSpeed))
		THROW_HW_ERROR(InvalidValue) << "32-bit works only on "
					     << "QuarterSpeed";
	typedef slsDetectorDefs::speedLevel Speed;
	EXC_CHECK(m_det->setReadoutSpeed(Speed(clock_div)));
	m_clock_div = clock_div;
	updateCameraTimeRanges();
}

void Eiger::getClockDiv(ClockDiv& clock_div)
{
	DEB_MEMBER_FUNCT();
	const char *err_msg = "Speed is different";
	typedef slsDetectorDefs::speedLevel Speed;
	sls::Result<Speed> res;
	EXC_CHECK(res = m_det->getReadoutSpeed());
	EXC_CHECK(clock_div = ClockDiv(res.tsquash(err_msg)));
	DEB_RETURN() << DEB_VAR1(clock_div);
}

void Eiger::setSubExpTime(double sub_exp_time)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR1(sub_exp_time);
	if (sub_exp_time != 0)
		THROW_HW_ERROR(NotSupported) << "SubExpTime not supported";
	updateCameraTimeRanges();
}

void Eiger::getSubExpTime(double& sub_exp_time)
{
	DEB_MEMBER_FUNCT();
	sub_exp_time = 0;
	DEB_RETURN() << DEB_VAR1(sub_exp_time);
}

void Eiger::setAllTrimBits(int mod_idx, int val)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR2(mod_idx, val);

	if ((mod_idx < -1) || (mod_idx >= getNbDetModules()))
		THROW_HW_ERROR(InvalidValue) << DEB_VAR1(mod_idx);

	Positions pos = Idx2Pos(mod_idx);
	EXC_CHECK(m_det->setAllTrimbits(val, pos));
}

void Eiger::getAllTrimBits(int mod_idx, int& val)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR1(mod_idx);

	if ((mod_idx < 0) || (mod_idx >= getNbDetModules()))
		THROW_HW_ERROR(InvalidValue) << DEB_VAR1(mod_idx);

	Positions pos = Idx2Pos(mod_idx);
	EXC_CHECK(val = m_det->getAllTrimbits(pos).front());
	DEB_RETURN() << DEB_VAR1(val);
}

void Eiger::getAllTrimBitsList(IntList& val_list)
{
	DEB_MEMBER_FUNCT();
	int nb_modules = getNbDetModules();
	val_list.resize(nb_modules);
	for (int i = 0; i < nb_modules; ++i)
		getAllTrimBits(i, val_list[i]);
}

void Eiger::setHighVoltage(int hvolt)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR1(hvolt);
	DEB_ALWAYS() << "Setting high voltage (" << DEB_VAR1(hvolt) << ") ...";
	EXC_CHECK(m_det->setHighVoltage(hvolt));
}

void Eiger::getHighVoltage(int& hvolt)
{
	DEB_MEMBER_FUNCT();
	Positions pos = Idx2Pos(0);
	EXC_CHECK(hvolt = m_det->getHighVoltage(pos).front());
	DEB_RETURN() << DEB_VAR1(hvolt);
}

void Eiger::setThresholdEnergy(int thres)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR1(thres);
	EXC_CHECK(m_det->setThresholdEnergy(thres));
}

void Eiger::getThresholdEnergy(int& thres)
{
	DEB_MEMBER_FUNCT();
	EXC_CHECK(thres = m_det->getThresholdEnergy().squash(-1));
	DEB_RETURN() << DEB_VAR1(thres);
}

void Eiger::setTxFrameDelay(int tx_frame_delay)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR1(tx_frame_delay);
	EXC_CHECK(m_det->setTransmissionDelayFrame(tx_frame_delay));
}

void Eiger::getTxFrameDelay(int& tx_frame_delay)
{
	DEB_MEMBER_FUNCT();
	sls::Result<int> res;
	EXC_CHECK(res = m_det->getTransmissionDelayFrame());
	const char *err_msg = "Tx frame delay is different";
	EXC_CHECK(tx_frame_delay = res.tsquash(err_msg));
	DEB_RETURN() << DEB_VAR1(tx_frame_delay);
}

void Eiger::setSignedImageMode(bool signed_image_mode)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR1(signed_image_mode);
	m_signed_image_mode = signed_image_mode;
	getCamera()->updateImageSize();
}

void Eiger::getSignedImageMode(bool& signed_image_mode)
{
	DEB_MEMBER_FUNCT();
	signed_image_mode = m_signed_image_mode;
	DEB_RETURN() << DEB_VAR1(signed_image_mode);
}

bool Eiger::isTenGigabitEthernetEnabled()
{
	DEB_MEMBER_FUNCT();
	bool enabled;
	const char *err_msg = "Ten-giga is different";
	EXC_CHECK(enabled = m_det->getTenGiga().tsquash(err_msg));
	DEB_RETURN() << DEB_VAR1(enabled);
	return enabled;
}

void Eiger::setFlowControl10G(bool enabled)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR1(enabled);
	EXC_CHECK(m_det->setTenGigaFlowControl(enabled));
}

void Eiger::prepareAcq()
{
	DEB_MEMBER_FUNCT();

	CorrList::iterator cit, cend = m_corr_list.end();
	for (cit = m_corr_list.begin(); cit != cend; ++cit)
		(*cit)->prepareAcq();
}

void Eiger::startAcq()
{
	DEB_MEMBER_FUNCT();
}

void Eiger::stopAcq()
{
	DEB_MEMBER_FUNCT();
}

Eiger::CorrBase *Eiger::createChipBorderCorr(ImageType image_type)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR1(image_type);

	CorrBase *border_corr;
	switch (image_type) {
	case Bpp8:
		border_corr = new ChipBorderCorr<unsigned char>(this);
		break;
	case Bpp8S:
		border_corr = new ChipBorderCorr<char>(this);
		break;
	case Bpp16:
		border_corr = new ChipBorderCorr<unsigned short>(this);
		break;
	case Bpp16S:
		border_corr = new ChipBorderCorr<short>(this);
		break;
	case Bpp32:
		border_corr = new ChipBorderCorr<unsigned int>(this);
		break;
	case Bpp32S:
		border_corr = new ChipBorderCorr<int>(this);
		break;
	default:
		THROW_HW_ERROR(NotSupported) 
			<< "Eiger correction not supported for " << image_type;
	}

	addCorr(border_corr);

	DEB_RETURN() << DEB_VAR1(border_corr);
	return border_corr;
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
	while (m_corr_list.size() > 0)
		delete *m_corr_list.rbegin();
}

double Eiger::getBorderCorrFactor(int /*det*/, int line)
{
	DEB_MEMBER_FUNCT();
	switch (line) {
	case 0: return 2.0;
	case 1: return 1.3;
	default: return 1;
	}
}

void Eiger::getFpgaFramePtrDiff(PtrDiffList& ptr_diff)
{
	DEB_MEMBER_FUNCT();
	for (unsigned int i = 0; i != m_beb_list.size(); ++i) {
		BebFpgaMem& fpga_mem = m_beb_list[i]->fpga_mem;
		unsigned long wr_ptr = fpga_mem.read(BebFpgaWritePtrAddr);
		unsigned long rd_ptr = fpga_mem.read(BebFpgaReadPtrAddr);
		if (rd_ptr > wr_ptr)
			wr_ptr += BebFpgaPtrRange;
		unsigned long diff = wr_ptr - rd_ptr;
		DEB_RETURN() << DEB_VAR2(i, diff);
		ptr_diff.push_back(diff);
	}
}

bool Eiger::isXferActive()
{
	DEB_MEMBER_FUNCT();
	PtrDiffList ptr_diff;
	getFpgaFramePtrDiff(ptr_diff);
	PtrDiffList::const_iterator it, end = ptr_diff.end();
	bool xfer_active = false;
	for (it = ptr_diff.begin(); (it != end) && !xfer_active; ++it)
		xfer_active = (*it != 0);
	DEB_RETURN() << DEB_VAR1(xfer_active);
	return xfer_active;
}

ostream& lima::SlsDetector::operator <<(ostream& os, Eiger::ParallelMode mode)
{
	const char *name = "Invalid";
	switch (mode) {
	case Eiger::NonParallel:	name = "NonParallel";	break;
	case Eiger::Parallel:		name = "Parallel";	break;
	}
	return os << name;
}

