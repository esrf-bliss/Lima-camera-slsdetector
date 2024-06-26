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

#include "SlsDetectorJungfrau.h"
#include "lima/MiscUtils.h"

#include "sls/detectors/jungfrau/Geometry.h"
#include "sls/Pixel.h"

#include <emmintrin.h>
#include <sched.h>

using namespace std;
using namespace lima;
using namespace lima::SlsDetector;
using namespace lima::SlsDetector::Defs;

#define applyDetGeom(j, f, raw)						\
	using namespace sls::Jungfrau::Geom;				\
	int ifaces;							\
	j->getNbUDPInterfaces(ifaces);					\
	auto any_nb_ifaces = AnyNbUDPIfacesFromNbUDPIfaces(ifaces);	\
	std::visit([&](auto nb) {					\
	    using nb_udp_ifaces = decltype(nb);				\
	    Defs::xy det_size = j->m_det->getDetectorSize();		\
	    auto any_geom = AnyDetGeomFromDetSize<nb_udp_ifaces>({det_size.x, \
								  det_size.y});\
	    std::visit([&](auto const &geom) {				\
	        if (raw)						\
			f(geom.raw_geom);				\
		else							\
			f(geom.asm_wg_geom);				\
	      }, any_geom);						\
	  }, any_nb_ifaces);


/*
 * Jungfrau::GainPed::Impl class
 */

template <class M>
void Jungfrau::GainPed::GainPed::Impl<M>::updateImageSize(Size size, bool raw)
{
	m_size = size;
	m_raw = raw;
	m_pixels = Point(m_size).getArea();
	std::vector<int> dims = {m_size.getWidth(), m_size.getHeight()};
	if (m_calib.gain_map[0].dimensions != dims)
		setDefaultCalib();
}

template <class M>
void Jungfrau::GainPed::Impl<M>::getDefaultCalib(Calib& calib)
{
	DEB_MEMBER_FUNCT();
	auto init = [&](auto& d) {
		initData(d, m_size, Data::DOUBLE);
		// gap pixels are set to 0
		if (!m_raw)
			memset(d.data(), 0, d.size());
	};
	double *p[3][2];
	for (int g = 0; g < 3; ++g) {
		init(calib.gain_map[g]);
		p[g][0] = (double *) calib.gain_map[g].data();
		init(calib.ped_map[g]);
		p[g][1] = (double *) calib.ped_map[g].data();
	}
	auto f = [&](auto const &det_geom) {
		using namespace sls::Geom;
		det_for_each_pixel(det_geom,
				   [&](auto const &chip_view,
				       auto const &pixel) {
			int i = chip_view.calcMapPixelIndex(pixel);
			for (int g = 0; g < 3; ++g) {
				p[g][0][i] = M::DefaultCoeffs[g][0];
				p[g][1][i] = M::DefaultCoeffs[g][1];
			}
		   });
	};
	applyDetGeom(m_jungfrau, f, m_raw);
}

template <class M>
void Jungfrau::GainPed::Impl<M>::processFrame(Data& data, Data& proc)
{
	DEB_MEMBER_FUNCT();

	auto calib_coeff = [&](auto gain, auto type) {
		Data& d = (type == 0) ? m_calib.gain_map[gain] :
					m_calib.ped_map[gain];
		return (double *) d.data();
	};
	double *coeffs[3][2] = {{calib_coeff(0, 0), calib_coeff(0, 1)},
				{calib_coeff(1, 0), calib_coeff(1, 1)},
				{calib_coeff(2, 0), calib_coeff(2, 1)}};
	{
		unsigned short *src;
		src = reinterpret_cast<unsigned short *>(data.data());
		using P = typename M::Pixel;
		P *dst = reinterpret_cast<P *>(proc.data());
		for (int i = 0; i < m_pixels; ++i, ++src, ++dst) {
			int gain = *src >> 14;
			int adc = *src & 0x3fff;
			DEB_TRACE() << DEB_VAR3(i, gain, adc);
			if (gain == 3)
				gain = 2;
			else if (gain == 2) {
				*dst = std::numeric_limits<P>::max() - 0x10;
				continue;
			}
			if (coeffs[gain][0][i] != 0)
				*dst = ((adc - coeffs[gain][1][i]) / coeffs[gain][0][i]
					+ 0.5);
			else
				*dst = std::numeric_limits<P>::min() + 0x10;
			DEB_TRACE() << DEB_VAR1(*dst);
		}
	}
}


/*
 * Jungfrau::GainPed class
 */

Jungfrau::GainPed::GainPed(Jungfrau *jungfrau)
	: m_jungfrau(jungfrau),
	  m_impl(std::in_place_index_t<0>(), jungfrau) // Instantiate Map16
{
	DEB_MEMBER_FUNCT();
	setMapType(Map32); // Switch Map16 -> Map32 forces initialization
}

void Jungfrau::GainPed::setMapType(MapType map_type)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR1(map_type);
	MapType curr_type;
	getMapType(curr_type);
	if (map_type == curr_type)
		return;
	if (map_type == Map16)
		m_impl = Impl<Map16Data>(m_jungfrau);
	else
		m_impl = Impl<Map32Data>(m_jungfrau);
	if (m_size.isEmpty())
		return;
	updateImageSize(m_size, m_raw);
	std::visit([&](auto& impl) { impl.setDefaultCalib(); }, m_impl);
}

void Jungfrau::GainPed::getMapType(MapType& map_type)
{
	DEB_MEMBER_FUNCT();
	std::visit([&](auto& impl) {
		map_type = std::decay_t<decltype(impl)>::Map::Type;
	    }, m_impl);
	DEB_RETURN() << DEB_VAR1(map_type);
}

void Jungfrau::GainPed::setCalib(const Calib& calib)
{
	DEB_MEMBER_FUNCT();
	std::visit([&](auto& impl) { impl.m_calib = calib; }, m_impl);
}

void Jungfrau::GainPed::getCalib(Calib& calib)
{
	DEB_MEMBER_FUNCT();
	std::visit([&](auto& impl) { calib = impl.m_calib; }, m_impl);
}

void Jungfrau::GainPed::updateImageSize(Size size, bool raw)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR2(size, raw);
	m_size = size;
	m_raw = raw;
	std::visit([&](auto& impl) { impl.updateImageSize(size, raw); }, m_impl);
}

void Jungfrau::GainPed::getDefaultCalib(Calib& calib)
{
	DEB_MEMBER_FUNCT();
	std::visit([&](auto& impl) { impl.getDefaultCalib(calib); }, m_impl);
}

Data::TYPE Jungfrau::GainPed::getDataType()
{
	DEB_MEMBER_FUNCT();
	return std::visit([&](auto& impl) {
		return std::decay_t<decltype(impl)>::Map::DataType;
	    }, m_impl);
}


void Jungfrau::GainPed::processFrame(Data& data, Data& proc)
{
	DEB_MEMBER_FUNCT();
	std::visit([&](auto& impl) { impl.processFrame(data, proc); }, m_impl);
}

std::ostream& lima::SlsDetector::operator <<(std::ostream& os,
					     Jungfrau::GainPed::MapType map_type)
{
	const char *name = "Unknown";
	switch (map_type) {
	case Jungfrau::GainPed::Map16:	name = "Map16";	break;	
	case Jungfrau::GainPed::Map32:	name = "Map32";	break;	
	}
	return os << name;
}

std::istream& lima::SlsDetector::operator >>(std::istream& is,
					     Jungfrau::GainPed::MapType& map_type)
{
	std::string s;
	is >> s;
	if (s == "Map16")
		map_type = Jungfrau::GainPed::Map16;
	else if (s == "Map32")
		map_type = Jungfrau::GainPed::Map32;
	else
		throw LIMA_HW_EXC(InvalidValue,
				  "Invalid Jungfrau::GainPed::MapType: ") << s;
	return is;
}

/*
 * Jungfrau::ImgProcTask class
 */

Jungfrau::ImgProcTask::ImgProcTask(Jungfrau *jungfrau)
	: m_jungfrau(jungfrau)
{
	DEB_CONSTRUCTOR();
}

void Jungfrau::ImgProcTask::process(Data& data)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR2(data.frameNumber, data.data());
}


/*
 * Jungfrau::ImgProcBase class
 */

Jungfrau::ImgProcBase::ImgProcBase(Jungfrau *jungfrau, std::string name)
	: m_jungfrau(jungfrau), m_name(name)
{
	DEB_CONSTRUCTOR();
	DEB_PARAM() << DEB_VAR1(m_name);
	m_nb_jungfrau_modules = m_jungfrau->getNbJungfrauModules();
	auto f = [&](auto const &det_geom) {
		m_det_mods = Point(det_geom.det_mods.x, det_geom.det_mods.y);
		DEB_TRACE() << DEB_VAR2(m_det_mods, m_nb_jungfrau_modules);
	};
	applyDetGeom(m_jungfrau, f, false);
}

Jungfrau::ImgProcBase::~ImgProcBase()
{
	DEB_DESTRUCTOR();
	if (m_jungfrau)
		m_jungfrau->removeImgProc(this);
}

void Jungfrau::ImgProcBase::updateImageSize(Size size, bool raw)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR3(m_name, size, raw);
	m_pixels = Point(size).getArea();
	m_frame_size = size;
	m_raw = raw;
}

void Jungfrau::ImgProcBase::prepareAcq()
{
	DEB_MEMBER_FUNCT();

	if (!m_jungfrau)
		THROW_HW_ERROR(InvalidValue) << "ImgProc already removed";

	clear();
}

void Jungfrau::ImgProcBase::clear()
{
	DEB_MEMBER_FUNCT();
}

/*
 * Jungfrau::GainADCMapImgProc class
 */

Jungfrau::GainADCMapImgProc::GainADCMapImgProc(Jungfrau *jungfrau)
	: ImgProcBase(jungfrau, "gain_adc_map"), m_helper(new Helper)
{
	DEB_CONSTRUCTOR();
}

void Jungfrau::GainADCMapImgProc::updateImageSize(Size size, bool raw)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR3(m_name, size, raw);
	ImgProcBase::updateImageSize(size, raw);

	for (auto& d : m_buffer)
		d.updateSize(size);
}

void Jungfrau::GainADCMapImgProc::clear()
{
	DEB_MEMBER_FUNCT();
	ImgProcBase::clear();

	for (auto& d : m_buffer)
		d.clear();
	m_buffer.reset();
}

void Jungfrau::GainADCMapImgProc::processFrame(Data& data)
{
	DEB_MEMBER_FUNCT();
	long frame = data.frameNumber;
	DEB_PARAM() << DEB_VAR1(frame);

	Data raw = m_jungfrau->getRawData(data);

	DoubleBufferWriter<MapData> w(m_buffer);
	MapData& m = w.getBuffer();
	DEB_TRACE() << DEB_VAR1(&m);
	unsigned short *src;
	{
		src = (unsigned short *) raw.data();
		unsigned char *dst = (unsigned char *) m.gain_map.data();
		for (int i = 0; i < m_pixels; ++i, ++src, ++dst) {
			int gain = *src >> 14;
			*dst = (gain == 3) ? 2 : gain;
		}
		m.gain_map.frameNumber = frame;
	}
	{
		src = (unsigned short *) raw.data();
		unsigned short *dst = (unsigned short *) m.adc_map.data();
		for (int i = 0; i < m_pixels; ++i, ++src, ++dst)
			*dst = *src & 0x3fff;
		m.adc_map.frameNumber = frame;
	}
	w.setCounter(frame);
}

void Jungfrau::GainADCMapImgProc::readGainADCMaps(Data& gain_map, Data& adc_map,
						  FrameType& frame)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR1(frame);
	Helper::ReaderPtr r = m_helper->createReader(m_buffer, frame);
	MapData &m = r->getBuffer();
	m_helper->addReaderData(r, m.gain_map, gain_map);
	m_helper->addReaderData(r, m.adc_map, adc_map);
	DEB_RETURN() << DEB_VAR3(gain_map, adc_map, frame);
}

/*
 * Jungfrau::GainPedImgProc class
 */

Jungfrau::GainPedImgProc::GainPedImgProc(Jungfrau *jungfrau)
	: ImgProcBase(jungfrau, "gain_ped"), m_gain_ped(jungfrau),
	  m_helper(new Helper)
{
	DEB_CONSTRUCTOR();
}

void Jungfrau::GainPedImgProc::updateImageSize(Size size, bool raw)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR3(m_name, size, raw);
	ImgProcBase::updateImageSize(size, raw);

	for (auto& d : m_buffer)
		d.updateSize(size);

	m_gain_ped.updateImageSize(size, raw);
}

void Jungfrau::GainPedImgProc::clear()
{
	DEB_MEMBER_FUNCT();
	ImgProcBase::clear();

	Data::TYPE data_type = m_gain_ped.getDataType();
	DEB_TRACE() << DEB_VAR1(data_type);
	for (auto& d : m_buffer)
		if (d.proc_map.type != data_type) {
			DEB_TRACE() << "Reallocating proc_map";
			d.proc_map.releaseBuffer();
			d.proc_map.type = data_type;
			d.updateSize(m_frame_size);
		} else
			d.clear();
	m_buffer.reset();
}

void Jungfrau::GainPedImgProc::processFrame(Data& data)
{
	DEB_MEMBER_FUNCT();
	long frame = data.frameNumber;
	DEB_PARAM() << DEB_VAR1(frame);

	Data raw = m_jungfrau->getRawData(data);

	DoubleBufferWriter<MapData> w(m_buffer);
	MapData& m = w.getBuffer();
	DEB_TRACE() << DEB_VAR1(&m);
	m_gain_ped.processFrame(raw, m.proc_map);
	m.proc_map.frameNumber = frame;
	w.setCounter(frame);
}

void Jungfrau::GainPedImgProc::readProcMap(Data& proc_map, FrameType& frame)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR1(frame);
	Helper::ReaderPtr r = m_helper->createReader(m_buffer, frame);
	MapData &m = r->getBuffer();
	m_helper->addReaderData(r, m.proc_map, proc_map);
	DEB_RETURN() << DEB_VAR2(proc_map, frame);
}

/*
 * Jungfrau::AveImgProc class
 */

Jungfrau::AveImgProc::AveImgProc(Jungfrau *jungfrau)
	: ImgProcBase(jungfrau, "ave"), m_helper(new Helper), m_nb_frames(0)
{
	DEB_CONSTRUCTOR();
	m_acc.type = Data::UINT32;
}

void Jungfrau::AveImgProc::updateImageSize(Size size, bool raw)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR3(m_name, size, raw);
	ImgProcBase::updateImageSize(size, raw);

	for (auto& d : m_buffer)
		d.updateSize(size);

	updateDataSize(m_acc, size);
}

void Jungfrau::AveImgProc::clear()
{
	DEB_MEMBER_FUNCT();
	ImgProcBase::clear();

	for (auto& d : m_buffer)
		d.clear();
	m_buffer.reset();

	clearData(m_acc);
	m_nb_frames = 0;
}

template <class M>
void Jungfrau::AveImgProc::processFrameFunct(Data& data)
{
	DEB_MEMBER_FUNCT();
	long frame = data.frameNumber;
	DEB_PARAM() << DEB_VAR1(frame);

	DoubleBufferWriter<MapData> w(m_buffer);
	MapData& m = w.getBuffer();
	DEB_TRACE() << DEB_VAR1(&m);
	++m_nb_frames;

	using S = typename M::Pixel;
	S *src = (S *) data.data();
	unsigned int *acc = (unsigned int *) m_acc.data();
	double *ave = (double *) m.ave_map.data();
	for (int i = 0; i < m_pixels; ++i, ++src, ++acc, ++ave) {
		*acc += *src & 0x3fff;
		*ave = *acc / m_nb_frames;
	}
	m.ave_map.frameNumber = frame;
	m.nb_frames = m_nb_frames;
	w.setCounter(frame);
}

void Jungfrau::AveImgProc::processFrame(Data& data)
{
	DEB_MEMBER_FUNCT();
	Data raw = m_jungfrau->getRawData(data);
	processFrameFunct<GainPed::Map16Data>(raw);
}

void Jungfrau::AveImgProc::readAveMap(Data& ave_map, FrameType& nb_frames,
				      FrameType& frame)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR1(frame);
	Helper::ReaderPtr r = m_helper->createReader(m_buffer, frame);
	MapData &m = r->getBuffer();
	m_helper->addReaderData(r, m.ave_map, ave_map);
	nb_frames = m.nb_frames;
	DEB_RETURN() << DEB_VAR3(ave_map, nb_frames, frame);
}

/*
 * Jungfrau::ModelReconstruction class
 */

Data Jungfrau::ModelReconstruction::processModel(Data& data)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR4(m_jungfrau, data.frameNumber,
				_processingInPlaceFlag, data.data());
	if (!m_jungfrau)
		return data;

	Data ret = _processingInPlaceFlag ? data : data.copy();

	ImgProcList& img_proc_list = m_jungfrau->m_img_proc_list;
	ImgProcList::iterator it, end = img_proc_list.end();
	for (it = img_proc_list.begin(); it != end; ++it)
		if ((*it)->consumesRawData())
			(*it)->processFrame(data);

	if (m_jungfrau->m_img_src == Raw)
		return data;

	Data raw = m_jungfrau->getRawData(data);
	DEB_TRACE() << DEB_VAR1(raw);
	GainPed& gain_ped = m_jungfrau->m_gain_ped_img_proc->m_gain_ped;
	gain_ped.processFrame(raw, ret);
	DEB_TRACE() << DEB_VAR1(ret);

	for (it = img_proc_list.begin(); it != end; ++it)
		if (!(*it)->consumesRawData())
			(*it)->processFrame(ret);

	return ret;
}


/*
 * Jungfrau detector class
 */

Jungfrau::Jungfrau(Camera *cam)
	: Model(cam, JungfrauDet), m_img_src(Raw)
{
	DEB_CONSTRUCTOR();

	int nb_det_modules = getNbDetModules();

	DEB_TRACE() << "Using Jungfrau detector, " << DEB_VAR1(nb_det_modules);

	m_gain_ped_img_proc = new GainPedImgProc(this);
	m_gain_adc_map_img_proc = new GainADCMapImgProc(this);
	m_ave_img_proc = new AveImgProc(this);

	m_reconstruction = new ModelReconstruction(this);

	updateCameraModel();
}

Jungfrau::~Jungfrau()
{
	DEB_DESTRUCTOR();
	getCamera()->waitAcqState(Idle);
	m_reconstruction->m_jungfrau = NULL;
	m_reconstruction->unref();
	removeAllImgProc();
}

void Jungfrau::setImgSrc(ImgSrc img_src)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR2(img_src, m_img_src);
	if (img_src == m_img_src)
		return;

	m_img_src = img_src;
	bool do_corr = (m_img_src == GainPedCorr);

	doSetImgProcConfig(m_img_proc_config, true);

	Reconstruction::LimaBufferMode lbm;
	lbm = do_corr ? Reconstruction::CorrData : Reconstruction::RawData;
	m_reconstruction->setLimaBufferMode(lbm);

	Camera *cam = getCamera();
	BufferMgr *buffer = cam->getBuffer();
	buffer->releaseBuffers();
	updateCameraImageSize();
}

void Jungfrau::getImgSrc(ImgSrc& img_src)
{
	DEB_MEMBER_FUNCT();
	img_src = m_img_src;
	DEB_RETURN() << DEB_VAR1(img_src);
}

void Jungfrau::getFrameDim(FrameDim& frame_dim, bool raw)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR1(raw);
	getAcqFrameDim(frame_dim, raw);
	if (m_img_src == GainPedCorr) {
		GainPed::MapType map_type;
		m_gain_ped_img_proc->m_gain_ped.getMapType(map_type);
		bool is_map32 = (map_type == GainPed::Map32);
		frame_dim.setImageType(is_map32 ? Bpp32S : Bpp16S);
	}
	DEB_RETURN() << DEB_VAR1(frame_dim);
}

void Jungfrau::getAcqFrameDim(FrameDim& frame_dim, bool raw)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR1(raw);
	Size size;
	auto f = [&](auto const &det_geom) {
		size = Size(det_geom.size.x, det_geom.size.y);
		DEB_TRACE() << DEB_VAR1(size);
	};
	applyDetGeom(this, f, raw);
	frame_dim = FrameDim(size, Bpp16);
	DEB_RETURN() << DEB_VAR1(frame_dim);
}

void Jungfrau::getDetMap(Data& det_map)
{
	DEB_MEMBER_FUNCT();
	bool raw = getRawMode();
	FrameDim frame_dim;
	getFrameDim(frame_dim, raw);
	initData(det_map, frame_dim.getSize(), Data::UINT32);
	// gap pixels are set to -1
	if (!raw)
		memset(det_map.data(), 0xff, det_map.size());

	auto f = [&](auto const &det_geom) {
		using namespace sls::Geom;
		unsigned int *p = (unsigned int *) det_map.data();
		int chip_idx = 0;
		det_for_each_chip(det_geom,
				  [&](auto const &chip, auto const &chip_view) {
		    view_for_each_pixel(chip_view,
					[&](auto const &chip_view,
					    auto const &pixel) {
			    int i = chip_view.calcMapPixelIndex(pixel);
			    p[i] = chip_idx;
			});
		    ++chip_idx;
		  });
	};
	applyDetGeom(this, f, raw);
}

string Jungfrau::getName()
{
	DEB_MEMBER_FUNCT();
	ostringstream os;
	os << "PSI/Jungfrau-";
	int nb_modules = getNbDetModules();
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

void Jungfrau::getPixelSize(double& x_size, double& y_size)
{
	DEB_MEMBER_FUNCT();
	x_size = y_size = 75e-6;
	DEB_RETURN() << DEB_VAR2(x_size, y_size);
}

void Jungfrau::getDACInfo(NameList& name_list, IntList& idx_list,
		       IntList& milli_volt_list)
{
	DEB_MEMBER_FUNCT();

#define JUNGFRAU_DAC(x)			{x, 0}
#define JUNGFRAU_DAC_MV(x)		{x, 1}

	static struct DACData {
		DACIndex idx;
		int milli_volt;
	} JungfrauDACList[] = {
		JUNGFRAU_DAC(Threshold),
	};
	const unsigned int size = C_LIST_SIZE(JungfrauDACList);

	name_list.resize(size);
	idx_list.resize(size);
	milli_volt_list.resize(size);
	struct DACData *data = JungfrauDACList;
	for (unsigned int i = 0; i < size; ++i, ++data) {
		ostringstream os;
		os << data->idx;
		name_list[i] = os.str();
		idx_list[i] = int(data->idx);
		milli_volt_list[i] = data->milli_volt;
		DEB_RETURN() << DEB_VAR2(name_list[i], idx_list[i]);
	}
}

void Jungfrau::getADCInfo(NameList& name_list, IntList& idx_list,
		       FloatList& factor_list, FloatList& min_val_list)
{
	DEB_MEMBER_FUNCT();

#define JUNGFRAU_TEMP_FACTOR		(1 / 1000.0)

#define JUNGFRAU_TEMP(x)			{x, JUNGFRAU_TEMP_FACTOR, 0}

	static struct ADCData {
		ADCIndex idx;
		double factor, min_val;
	} JungfrauADCList[] = {
		JUNGFRAU_TEMP(TempFPGA),
		JUNGFRAU_TEMP(TempFPGAExt),
		JUNGFRAU_TEMP(Temp10GE),
		JUNGFRAU_TEMP(TempDCDC),
		JUNGFRAU_TEMP(TempSODL),
		JUNGFRAU_TEMP(TempSODR),
		JUNGFRAU_TEMP(TempFPGAFL),
		JUNGFRAU_TEMP(TempFPGAFR),
	};
	const unsigned int size = C_LIST_SIZE(JungfrauADCList);

	name_list.resize(size);
	idx_list.resize(size);
	factor_list.resize(size);
	min_val_list.resize(size);
	struct ADCData *data = JungfrauADCList;
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

bool Jungfrau::checkTrigMode(TrigMode trig_mode)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR1(trig_mode);
	bool valid_mode = (trig_mode != Defs::SoftTriggerExposure);
	DEB_RETURN() << DEB_VAR1(valid_mode);
	return valid_mode;
}

void Jungfrau::getTimeRanges(TimeRanges& time_ranges)
{
	DEB_MEMBER_FUNCT();

	int nb_udp_ifaces;
	getNbUDPInterfaces(nb_udp_ifaces);

	double min_exp = 0.1;
	double min_period = 1000 / nb_udp_ifaces;
	double min_lat = 0.1;

	time_ranges.min_exp_time = min_exp * 1e-6;
	time_ranges.max_exp_time = 1e3;
	time_ranges.min_lat_time = min_lat * 1e-6;
	time_ranges.max_lat_time = 1e3;
	time_ranges.min_frame_period = min_period * 1e-6;
	time_ranges.max_frame_period = 1e3;
}

void Jungfrau::updateImageSize()
{
	DEB_MEMBER_FUNCT();

	bool raw = getRawMode();
	FrameDim frame_dim;
	getAcqFrameDim(frame_dim, raw);
	Size size = frame_dim.getSize();
	DEB_TRACE() << DEB_VAR2(size, raw);

 	m_gain_ped_img_proc->updateImageSize(size, raw);
	m_gain_adc_map_img_proc->updateImageSize(size, raw);
	m_ave_img_proc->updateImageSize(size, raw);
}

bool Jungfrau::checkSettings(Settings settings)
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

void Jungfrau::setHighVoltage(int hvolt)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR1(hvolt);
	DEB_ALWAYS() << "Setting high voltage (" << DEB_VAR1(hvolt) << ") ...";
	EXC_CHECK(m_det->setHighVoltage(hvolt));
}

void Jungfrau::getHighVoltage(int& hvolt)
{
	DEB_MEMBER_FUNCT();
	Positions pos = Idx2Pos(0);
	EXC_CHECK(hvolt = m_det->getHighVoltage(pos).front());
	DEB_RETURN() << DEB_VAR1(hvolt);
}

void Jungfrau::setGainMode(GainMode gain_mode)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR1(gain_mode);
	typedef slsDetectorDefs::gainMode SlsMode;
	SlsMode sls_mode = SlsMode(gain_mode);
	EXC_CHECK(m_det->setGainMode(sls_mode));
}

void Jungfrau::getGainMode(GainMode& gain_mode)
{
	DEB_MEMBER_FUNCT();
	typedef slsDetectorDefs::gainMode SlsMode;
	SlsMode sls_mode;
	const char *err_msg = "Detector gain modes are different";
	EXC_CHECK(sls_mode = m_det->getGainMode().tsquash(err_msg));
	gain_mode = GainMode(sls_mode);
	DEB_RETURN() << DEB_VAR1(gain_mode);
}

void Jungfrau::prepareAcq()
{
	DEB_MEMBER_FUNCT();

	ImgProcList::iterator pit, pend = m_img_proc_list.end();
	for (pit = m_img_proc_list.begin(); pit != pend; ++pit)
		(*pit)->prepareAcq();
}

void Jungfrau::startAcq()
{
	DEB_MEMBER_FUNCT();
}

void Jungfrau::stopAcq()
{
	DEB_MEMBER_FUNCT();
}

bool Jungfrau::isXferActive()
{
	DEB_MEMBER_FUNCT();
	bool xfer_active = 0;
	DEB_RETURN() << DEB_VAR1(xfer_active);
	return xfer_active;
}

Jungfrau::ImgProcTask *Jungfrau::createImgProcTask()
{
	DEB_MEMBER_FUNCT();
	return false ? new ImgProcTask(this) : NULL;
}

void Jungfrau::addImgProc(ImgProcBase *img_proc)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR1(img_proc);
	m_img_proc_list.push_back(img_proc);
	bool raw = getRawMode();
	FrameDim frame_dim;
	getFrameDim(frame_dim, raw);
	img_proc->updateImageSize(frame_dim.getSize(), raw);
}

void Jungfrau::removeImgProc(ImgProcBase *img_proc)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR1(img_proc);

	ImgProcList::iterator it, end = m_img_proc_list.end();
	it = find(m_img_proc_list.begin(), end, img_proc);
	if (it != end)
		m_img_proc_list.erase(it);
	img_proc->m_jungfrau = NULL;
}

void Jungfrau::removeAllImgProc()
{
	DEB_MEMBER_FUNCT();
	m_img_proc_list.clear();
	m_img_proc_config.clear();
}

void Jungfrau::setImgProcConfig(std::string config)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR1(config);
	doSetImgProcConfig(config, false);
}

void Jungfrau::getImgProcConfig(std::string &config)
{
	DEB_MEMBER_FUNCT();
	config = m_img_proc_config;
	DEB_RETURN() << DEB_VAR1(config);
}

void Jungfrau::doSetImgProcConfig(std::string config, bool force)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR3(m_img_proc_config, config, force);
	if ((config == m_img_proc_config) && !force)
		return;
	StringList config_list = SplitString(config);
	removeAllImgProc();
	bool do_gain_ped_corr = (m_img_src == GainPedCorr);
	if (config_list.empty()) {
		m_reconstruction->setActive(do_gain_ped_corr);
		return;
	}
	ImgProcList img_proc_list;
	auto checkConfigAndDelete = [&](auto s) {
		StringList::iterator it, end = config_list.end();
		it = find(config_list.begin(), end, s);
		if (it == end)
			return false;
		config_list.erase(it);
		return true;
	};
	if (checkConfigAndDelete("gain_ped")) {
		if (!do_gain_ped_corr) {
			DEB_TRACE() << "Adding gain_ped";
			img_proc_list.push_back(m_gain_ped_img_proc);
		} else {
			DEB_TRACE() << "Ignoring gain_ped";
			m_gain_ped_img_proc->clear();
		}
	}
	if (checkConfigAndDelete("gain_adc_map")) {
		DEB_TRACE() << "Adding gain_adc_map";
		img_proc_list.push_back(m_gain_adc_map_img_proc);
	}
	if (checkConfigAndDelete("ave")) {
		DEB_TRACE() << "Adding ave";
		img_proc_list.push_back(m_ave_img_proc);
	}
	if (!config_list.empty())
		THROW_HW_ERROR(InvalidValue) << "Invalid " << DEB_VAR1(config);

	for (auto img_proc: img_proc_list)
		addImgProc(img_proc);

	m_img_proc_config = config;

	bool do_reconst = do_gain_ped_corr || !m_img_proc_config.empty();
	m_reconstruction->setActive(do_reconst);
}

void Jungfrau::readGainADCMaps(Data& gain_map, Data& adc_map, FrameType& frame)
{
	DEB_MEMBER_FUNCT();
	m_gain_adc_map_img_proc->readGainADCMaps(gain_map, adc_map, frame);
}


void Jungfrau::readGainPedProcMap(Data& proc_map, FrameType& frame)
{
	DEB_MEMBER_FUNCT();
	m_gain_ped_img_proc->readProcMap(proc_map, frame);
}

void Jungfrau::readAveMap(Data& ave_map, FrameType& nb_frames, FrameType& frame)
{
	DEB_MEMBER_FUNCT();
	m_ave_img_proc->readAveMap(ave_map, nb_frames, frame);
}

void Jungfrau::setGainPedMapType(GainPed::MapType map_type)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR1(map_type);
	GainPed& gain_ped = m_gain_ped_img_proc->m_gain_ped;
	gain_ped.setMapType(map_type);
	if (m_img_src == GainPedCorr)
		updateCameraImageSize();
}

void Jungfrau::getGainPedMapType(GainPed::MapType& map_type)
{
	DEB_MEMBER_FUNCT();
	GainPed& gain_ped = m_gain_ped_img_proc->m_gain_ped;
	gain_ped.getMapType(map_type);
	DEB_RETURN() << DEB_VAR1(map_type);
}

std::ostream& lima::SlsDetector::operator <<(std::ostream& os,
					     Jungfrau::ImgSrc img_src)
{
	const char *name = "Unknown";
	switch (img_src) {
	case Jungfrau::Raw:		name = "Raw";		break;	
	case Jungfrau::GainPedCorr:	name = "GainPedCorr";	break;	
	}
	return os << name;
}

std::istream& lima::SlsDetector::operator >>(std::istream& is,
					     Jungfrau::ImgSrc& img_src)
{
	std::string s;
	is >> s;
	if (s == "Raw")
		img_src = Jungfrau::Raw;
	else if (s == "GainPedCorr")
		img_src = Jungfrau::GainPedCorr;
	else
		throw LIMA_HW_EXC(InvalidValue, "Invalid Jungfrau::ImgSrc: ")
			<< s;
	return is;
}

