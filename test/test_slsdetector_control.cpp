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
#include "SlsDetectorJungfrau.h"
#include "SlsDetectorInterface.h"
#include "lima/CtTestApp.h"

using namespace std;
using namespace lima;
using namespace lima::SlsDetector;

DEB_GLOBAL(DebModTest);

class TestApp : public CtTestApp
{
	DEB_CLASS_NAMESPC(DebModTest, "TestApp", "SlsDetector");

 public:
	class Pars : public CtTestApp::Pars
	{
		DEB_CLASS_NAMESPC(DebModTest, "TestApp::Pars", "SlsDetector");
	public:
		int cam_det_id{0};
		std::string cam_config_fname;
		bool cam_raw_mode{false};
		Jungfrau::GainPed::MapType jungfrau_gain_ped_map_type
			{Jungfrau::GainPed::Map16};
		Jungfrau::ImgSrc jungfrau_img_src{Jungfrau::Raw};
		PixelDepth eiger_pixel_depth{PixelDepth4};
		bool eiger_test_pattern{true};
		int eiger_dac_threshold{2300};

		Pars();
	};

	TestApp(int argc, char *argv[]) : CtTestApp(argc, argv) {}

 protected:
	virtual CtTestApp::Pars *getPars();
	virtual CtControl *getCtControl();
	virtual index_map getIndexMap() { return {}; }
	virtual void configureAcq(const index_map& indexes) {}

	AutoPtr<Pars> m_pars;
	AutoPtr<Camera> m_cam;
	AutoPtr<Eiger> m_eiger;
	AutoPtr<Jungfrau> m_jungfrau;
	AutoPtr<Interface> m_interface;
	AutoPtr<CtControl> m_ct;
};


TestApp::Pars::Pars()
{
	DEB_CONSTRUCTOR();
	const char *env_config = getenv("SLSDETECTOR_CONFIG");
	if (env_config)
		cam_config_fname = env_config;

#define AddOpt(var, opt, par)	\
	m_opt_list.insert(MakeOpt(var, "", opt, par))

	AddOpt(cam_det_id, "--cam-det-id", "multi-detector ID");

	AddOpt(cam_config_fname, "--cam-config-fname", "detector config file");

	AddOpt(cam_raw_mode, "--cam-raw-mode", "raw mode");

	AddOpt(jungfrau_gain_ped_map_type, "--jungfrau-gain-ped-map-type",
	       "Jungfrau::GainPed map type");

	AddOpt(jungfrau_img_src, "--jungfrau-img-src", "Jungfrau image source");

	AddOpt(eiger_pixel_depth, "--eiger-pixel-depth",
	       "Eiger sensor pixel depth");

	AddOpt(eiger_test_pattern, "--eiger-test-pattern",
	       "Eiger test trimbits pattern");

	AddOpt(eiger_dac_threshold, "--eiger-dac-threshold",
	       "Eiger DAC threshold valud");
}

CtTestApp::Pars *TestApp::getPars()
{
	m_pars = new Pars();
	return m_pars;
}

CtControl *TestApp::getCtControl()
{
	DEB_MEMBER_FUNCT();

	if (m_pars->cam_config_fname.empty())
		THROW_HW_ERROR(Error) << "Missing config file";

	m_cam = new Camera(m_pars->cam_config_fname, m_pars->cam_det_id);
	m_interface = new Interface(*m_cam);

	Type det_type = m_cam->getType();
	Reconstruction *r = NULL;
	if (det_type == EigerDet) {
		m_eiger = new Eiger(m_cam);
		r = m_eiger->getReconstruction();
	} else if (det_type == JungfrauDet) {
		m_jungfrau = new Jungfrau(m_cam);
		r = m_jungfrau->getReconstruction();
	} else
		THROW_HW_ERROR(Error) << "Unknown detector: " << det_type;
	if (!r)
		THROW_HW_ERROR(Error) << "Invalid NULL reconstruction";
	m_interface->setReconstruction(r);

	m_cam->setRawMode(m_pars->cam_raw_mode);

	if (m_jungfrau) {
		Jungfrau::GainPed::MapType map_type =
			m_pars->jungfrau_gain_ped_map_type;
		DEB_ALWAYS() << "Jungfrau: GainPed::MapType=" << map_type;
		m_jungfrau->setGainPedMapType(map_type);
		Jungfrau::ImgSrc img_src = m_pars->jungfrau_img_src;
		DEB_ALWAYS() << "Jungfrau: ImgSrc=" << img_src;
		m_jungfrau->setImgSrc(img_src);
	} else {
		PixelDepth pixel_depth = m_pars->eiger_pixel_depth;
		DEB_ALWAYS() << "Eiger: PixelDepth=" << pixel_depth;
		m_cam->setPixelDepth(pixel_depth);
		bool test_pattern = m_pars->eiger_test_pattern;
		DEB_ALWAYS() << "Eiger: TestPattern=" << test_pattern;
		if (test_pattern) {
			std::string settings_dir;
			settings_dir = m_cam->getCmd("settingspath");
			std::string test_prefix;
			test_prefix = settings_dir + "/standard/eigernoise";
			std::string cmd = "trimbits " + test_prefix;
			DEB_ALWAYS() << DEB_VAR1(cmd);
			m_cam->putCmd(cmd);
			int dac_threshold = m_pars->eiger_dac_threshold;
			DEB_ALWAYS() << DEB_VAR1(dac_threshold);
			m_cam->setDAC(-1, Defs::DACIndex::Threshold,
				      dac_threshold);
		}
	}

	m_ct = new CtControl(m_interface);
	return m_ct;
}

int main(int argc, char *argv[])
{
	DEB_GLOBAL_FUNCT();
        try {
		TestApp app(argc, argv);
		app.run();
        } catch (Exception& e) {
	        DEB_ERROR() << "LIMA Exception:" << e.getErrMsg();
        }
	return 0;
};

