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
const int Eiger::RecvPorts = 2;

Eiger::CorrBase::CorrBase(Eiger *eiger)
	: m_eiger(eiger)
{
	DEB_CONSTRUCTOR();
	m_nb_modules = m_eiger->m_nb_det_modules / 2;
}

Eiger::CorrBase::~CorrBase()
{
	DEB_DESTRUCTOR();
	if (m_eiger)
		m_eiger->removeCorr(this);
}

bool Eiger::CorrBase::getRaw()
{
	DEB_MEMBER_FUNCT();
	bool raw;
	m_eiger->getCamera()->getRawMode(raw);
	DEB_RETURN() << DEB_VAR1(raw);
	return raw; 
}

void Eiger::CorrBase::prepareAcq()
{
	DEB_MEMBER_FUNCT();

	if (!m_eiger)
		THROW_HW_ERROR(InvalidValue) << "Correction already removed";

	FrameDim& recv_dim = m_eiger->m_recv_frame_dim;
	m_mod_frame_dim = recv_dim * Point(1, 2);
	m_frame_size = m_mod_frame_dim.getSize() * Point(1, m_nb_modules);
	m_inter_lines.resize(m_nb_modules);
	for (int i = 0; i < m_nb_modules - 1; ++i) {
		m_inter_lines[i] = m_eiger->getInterModuleGap(i);
		m_frame_size += Point(0, m_inter_lines[i]);
	}
	m_inter_lines[m_nb_modules - 1] = 0;
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
	int dlw = width * m_mod_frame_dim.getDepth();
	for (int i = 0, start = 0; i < m_nb_modules - 1; ++i) {
		start += mod_size;
		int size = m_inter_lines[i] * dlw;
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
	: m_corr_list(eiger->m_corr_list)
{
	DEB_CONSTRUCTOR();
}

Data Eiger::Correction::process(Data& data)
{
	DEB_MEMBER_FUNCT();

	CorrBase *corr = m_corr_list[0];
	bool raw = corr->getRaw();
	DEB_PARAM() << DEB_VAR4(data.frameNumber, raw, 
				_processingInPlaceFlag, data.data());
	
	Data ret = data;

	if (!_processingInPlaceFlag) {
		int size = data.size();
		Buffer *buffer = new Buffer(size);
		memcpy(buffer->data, data.data(), size);
		ret.setBuffer(buffer);
		buffer->unref();
	}

	if (!raw) {
		FrameType frame = ret.frameNumber;
		void *ptr = ret.data();
		CorrList::iterator it, end = m_corr_list.end();
		for (it = m_corr_list.begin(); it != end; ++it)
			(*it)->correctFrame(frame, ptr);
	}
	return ret;
}

Eiger::RecvPortGeometry::RecvPortGeometry(Eiger *eiger, int recv_idx, int port)
	: m_eiger(eiger), m_port(port), m_recv_idx(recv_idx)
{
	DEB_CONSTRUCTOR();
	DEB_PARAM() << DEB_VAR1(m_recv_idx);
	m_top_half_recv = (m_recv_idx % 2 == 0);
}

void Eiger::RecvPortGeometry::prepareAcq()
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR1(m_recv_idx);

	const FrameDim& frame_dim = m_eiger->m_recv_frame_dim;
	const Size& size = frame_dim.getSize();
	int depth = frame_dim.getDepth();
	m_dlw = size.getWidth() * depth;
	int recv_size = frame_dim.getMemSize();
	m_port_offset = recv_size * m_recv_idx;	

	m_eiger->getCamera()->getRawMode(m_raw);
	if (m_raw) {
		m_port_offset += ChipSize * m_dlw * m_port;
		return;
	}

	int mod_idx = m_recv_idx / 2;
	for (int i = 0; i < mod_idx; ++i)
		m_port_offset += m_eiger->getInterModuleGap(i) * m_dlw;

	if (m_top_half_recv) {
		m_port_offset += (ChipSize - 1) * m_dlw;
		m_dlw *= -1;
	} else {
		m_port_offset += (ChipGap / 2) * m_dlw;
	}

	m_plw = ChipSize * depth;
	m_pchips = HalfModuleChips / RecvPorts;
	m_clw = m_plw + ChipGap * depth;
	m_port_offset += m_pchips * m_clw * m_port;
}

void Eiger::RecvPortGeometry::processRecvFileStart(uint32_t dsize)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR2(m_recv_idx, dsize);
}

void Eiger::RecvPortGeometry::processRecvPort(FrameType frame, char *dptr,
					      char *bptr)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR3(frame, m_recv_idx, m_port);

	char *dest = bptr + m_port_offset;	
	if (m_raw) {
		memcpy(dest, dptr, m_dlw * ChipSize);
		return;
	}
	
	char *src = dptr;
	for (int i = 0; i < ChipSize; ++i, dest += m_dlw) {
		char *d = dest;
		for (int j = 0; j < m_pchips; ++j, src += m_plw, d += m_clw)
			memcpy(d, src, m_plw);
	}
}

Eiger::Eiger(Camera *cam)
	: Camera::Model(cam, Camera::EigerDet)
{
	DEB_CONSTRUCTOR();

	m_nb_det_modules = getCamera()->getNbDetModules();
	DEB_TRACE() << "Using Eiger detector, " << DEB_VAR1(m_nb_det_modules);

	for (int i = 0; i < m_nb_det_modules; ++i) {
		for (int j = 0; j < RecvPorts; ++j) {
			RecvPortGeometry *g = new RecvPortGeometry(this, i, j);
			m_port_geom_list.push_back(g);
		}
	}

	updateCameraModel();
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
	size *= Point(1, m_nb_det_modules);
	if (!raw)
		for (int i = 0; i < m_nb_det_modules / 2 - 1; ++i)
			size += Point(0, getInterModuleGap(i));
	frame_dim.setSize(size);
	DEB_RETURN() << DEB_VAR1(frame_dim);
}

void Eiger::getRecvFrameDim(FrameDim& frame_dim, bool raw, bool geom)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR1(raw);
	frame_dim.setImageType(getCamera()->getImageType());
	Size chip_size(ChipSize, ChipSize);
	if (raw) {
		int port_chips = HalfModuleChips / RecvPorts;
		frame_dim.setSize(chip_size * Point(port_chips, RecvPorts));
	} else {
		Size size = chip_size * Point(HalfModuleChips, 2);
		if (geom)
			size += Point(ChipGap, ChipGap) * Point(3, 1);
		frame_dim.setSize(size / Point(1, 2));
	}
	DEB_RETURN() << DEB_VAR1(frame_dim);
}

string Eiger::getName()
{
	DEB_MEMBER_FUNCT();
	ostringstream os;
	os << "PSI/Eiger-";
	int nb_modules = m_nb_det_modules / 2;
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

#define EIGER_DAC_MV(x)			{x, 1}
#define EIGER_DAC_OTHER(x)		{x, 0}

	static struct DACData {
		DACIndex idx;
		int milli_volt;
	} EigerDACList[] = {
		EIGER_DAC_MV(EigerSvP),
		EIGER_DAC_MV(EigerSvN),
		EIGER_DAC_MV(EigerVrf),
		EIGER_DAC_MV(EigerVrs),
		EIGER_DAC_MV(EigerVtr),
		EIGER_DAC_MV(EigerVtgstv),
		EIGER_DAC_MV(EigerVcal),
		EIGER_DAC_MV(EigerVcp),
		EIGER_DAC_MV(EigerVcn),
		EIGER_DAC_MV(EigerVis),
		EIGER_DAC_MV(EigerVcmpLL),
		EIGER_DAC_MV(EigerVcmpLR),
		EIGER_DAC_MV(EigerVcmpRL),
		EIGER_DAC_MV(EigerVcmpRR),
		EIGER_DAC_MV(EigerRxbLB),
		EIGER_DAC_MV(EigerRxbRB),
		EIGER_DAC_MV(Threshold),
		EIGER_DAC_OTHER(IODelay),
		EIGER_DAC_OTHER(HVNew),
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

void Eiger::updateImageSize()
{
	DEB_MEMBER_FUNCT();

	removeAllCorr();

	CorrBase *corr;
	ImageType image_type = getCamera()->getImageType();
	corr = createChipBorderCorr(image_type);
	m_corr_list.push_back(corr);

	if (corr->getNbModules() > 1) {
		corr = createInterModGapCorr();
		m_corr_list.push_back(corr);
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

Eiger::ReadoutFlags Eiger::getReadoutFlagsMask()
{
	DEB_MEMBER_FUNCT();
	ReadoutFlags flags = ReadoutFlags(Parallel | NonParallel | Safe | 
					  StoreInRAM | Continous);
	DEB_RETURN() << DEB_VAR1(flags);
	return flags;
}

bool Eiger::checkReadoutFlags(ReadoutFlags flags, IntList& flag_list, 
			      bool silent)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR2(flags, silent);

	bool ok = false;
	ReadoutFlags mask, result;

	mask = ReadoutFlags(Parallel | NonParallel | Safe);
	result = ReadoutFlags(flags & mask);
	flags = ReadoutFlags(flags & ~mask);
	if (countFlags(result) != 1) {
		if (!silent)
			DEB_ERROR() << "Invalid number of readout-mode flags";
		goto out;
	}
	flag_list.push_back(result);

	mask = ReadoutFlags(StoreInRAM | Continous);
	result = ReadoutFlags(flags & mask);
	flags = ReadoutFlags(flags & ~mask);
	if (countFlags(result) != 1) {
		if (!silent)
			DEB_ERROR() << "Invalid number of store-in-mem flags";
		goto out;
	}
	flag_list.push_back(result);

	if (flags != 0) {
		if (!silent)
			DEB_ERROR() << "Invalid flags for Eiger: " << flags;
		goto out;
	}

	ok = true;
 out:
	DEB_RETURN() << DEB_VAR1(ok);
	return ok;
}

int Eiger::countFlags(ReadoutFlags flags)
{
	const unsigned int nb_bits = sizeof(flags) * 8;
	int count = 0;
	for (unsigned int i = 0; i < nb_bits; ++i)
		if (flags & (1 << i))
			count++;
	return count;
}

int Eiger::getRecvPorts()
{
	DEB_MEMBER_FUNCT();
	DEB_RETURN() << DEB_VAR1(RecvPorts);
	return RecvPorts;
}

void Eiger::prepareAcq()
{
	DEB_MEMBER_FUNCT();

	bool raw;
	getCamera()->getRawMode(raw);
	getRecvFrameDim(m_recv_frame_dim, raw, true);
	
	DEB_TRACE() << DEB_VAR2(raw, m_recv_frame_dim);

	for (int i = 0; i < m_nb_det_modules * RecvPorts; ++i)
		m_port_geom_list[i]->prepareAcq();

	CorrList::iterator it, end = m_corr_list.end();
	for (it = m_corr_list.begin(); it != end; ++it)
		(*it)->prepareAcq();
}

void Eiger::processRecvFileStart(int recv_idx, uint32_t dsize)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR2(recv_idx, dsize);
	for (int i = 0; i < RecvPorts; ++i) {
		int port_idx = getPortIndex(recv_idx, i);
		m_port_geom_list[port_idx]->processRecvFileStart(dsize);
	}
}

void Eiger::processRecvPort(int recv_idx, FrameType frame, int port, char *dptr, 
			    uint32_t dsize, Mutex& lock, char *bptr)
{
	DEB_MEMBER_FUNCT();
	int port_idx = getPortIndex(recv_idx, port);
	m_port_geom_list[port_idx]->processRecvPort(frame, dptr, bptr);
}

Eiger::Correction *Eiger::createCorrectionTask()
{
	DEB_MEMBER_FUNCT();
	return new Correction(this);
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
	if (it == end) {
		DEB_WARNING() << DEB_VAR1(corr) << " already removed";
		return;
	}

	corr->m_eiger = NULL;
	m_corr_list.erase(it);
}

void Eiger::removeAllCorr()
{
	DEB_MEMBER_FUNCT();
	while (!m_corr_list.empty())
		m_corr_list.erase(m_corr_list.begin());
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
	if (det >= (m_nb_det_modules / 2) - 1)
		THROW_HW_ERROR(InvalidValue) << "Invalid " << DEB_VAR1(det);
	return 36;
}

