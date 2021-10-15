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
#include "lima/HwTestApp.h"

using namespace std;
using namespace lima;
using namespace lima::SlsDetector;

DEB_GLOBAL(DebModTest);

class TestApp : public HwTestApp
{
	DEB_CLASS_NAMESPC(DebModTest, "TestApp", "SlsDetector");

 public:
	class Pars : public HwTestApp::Pars
	{
		DEB_CLASS_NAMESPC(DebModTest, "TestApp::Pars", "SlsDetector");
	public:
		int det_id{0};
		std::string config_fname;
		bool raw_mode{false};
		Jungfrau::ImgSrc jungfrau_img_src{Jungfrau::Raw};

		Pars();
	};

	TestApp(int argc, char *argv[]) : HwTestApp(argc, argv) {}

 protected:
	virtual HwTestApp::Pars *getPars();
	virtual HwInterface *getHwInterface();

	AutoPtr<Pars> m_pars;
	AutoPtr<Camera> m_cam;
	AutoPtr<Eiger> m_eiger;
	AutoPtr<Jungfrau> m_jungfrau;
	AutoPtr<Interface> m_interface;
};


TestApp::Pars::Pars()
{
	DEB_CONSTRUCTOR();
	const char *env_config = getenv("SLSDETECTOR_CONFIG");
	if (env_config)
		config_fname = env_config;
	exp_time = 2.0e-3;
	double frame_period = 2.5e-3;
	latency_time = frame_period - exp_time;

#define AddOpt(var, sopt, lopt, par)	\
	m_opt_list.insert(MakeOpt(var, sopt, lopt, par))

	AddOpt(det_id, "-d", "--det-id", "multi-detector ID");

	AddOpt(config_fname, "-c", "--config-fname", "detector config file");

	AddOpt(raw_mode, "-r", "--raw-mode", "raw mode");

	AddOpt(jungfrau_img_src, "-i", "--jungfrau-img-src",
	       "Jungfrau image source");
}

HwTestApp::Pars *TestApp::getPars()
{
	m_pars = new Pars();
	return m_pars;
}

HwInterface *TestApp::getHwInterface()
{
	DEB_MEMBER_FUNCT();

	if (m_pars->config_fname.empty())
		THROW_HW_ERROR(Error) << "Missing config file";

	m_cam = new Camera(m_pars->config_fname, m_pars->det_id);
	m_interface = new Interface(*m_cam);

	Type det_type = m_cam->getType();
	if (det_type == EigerDet) {
		m_eiger = new Eiger(m_cam);
	} else if (det_type == JungfrauDet) {
		m_jungfrau = new Jungfrau(m_cam);
	} else
		THROW_HW_ERROR(Error) << "Unknown detector: " << det_type;

	m_cam->setRawMode(m_pars->raw_mode);

	if (m_jungfrau) {
		Jungfrau::ImgSrc img_src = m_pars->jungfrau_img_src;
		DEB_ALWAYS() << "Junfrau: ImgSrc=" << img_src;
		m_jungfrau->setImgSrc(img_src);
	}

	return m_interface;
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

