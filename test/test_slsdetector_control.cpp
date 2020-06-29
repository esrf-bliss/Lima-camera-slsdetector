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
#include "SlsDetectorInterface.h"
#include "SlsDetectorEiger.h"
#include "lima/CtControl.h"
#include "lima/CtAcquisition.h"
#include "lima/CtImage.h"
#include "lima/CtSaving.h"
#include "lima/CtSpsImage.h"
#include "lima/AcqState.h"

using namespace lima;
using namespace std;
using namespace SlsDetector;

DEB_GLOBAL(DebModTest);

//*********************************************************************
//* ImageStatusCallback
//*********************************************************************

class ImageStatusCallback : public CtControl::ImageStatusCallback
{
	DEB_CLASS(DebModTest, "ImageStatusCallback");

public:
	ImageStatusCallback(CtControl& ct, AcqState& acq_state);
	virtual ~ImageStatusCallback();

protected:
	virtual void imageStatusChanged(
				const CtControl::ImageStatus& img_status);

private:
	CtControl& m_ct;
	AcqState& m_acq_state;

	int m_nb_frames;
};

ImageStatusCallback::ImageStatusCallback(CtControl& ct, AcqState& acq_state)
	: m_ct(ct), m_acq_state(acq_state), m_nb_frames(0)
{
	DEB_CONSTRUCTOR();
}

ImageStatusCallback::~ImageStatusCallback()
{
	DEB_DESTRUCTOR();
}

void ImageStatusCallback::imageStatusChanged(
				const CtControl::ImageStatus& img_status)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR1(img_status);

	int last_acq_frame_nb = img_status.LastImageAcquired;
	int last_saved_frame_nb = img_status.LastImageSaved;
	DEB_ALWAYS() << DEB_VAR1(img_status);

	if (last_acq_frame_nb == 0) {
		CtAcquisition *ct_acq = m_ct.acquisition();
		ct_acq->getAcqNbFrames(m_nb_frames);
		DEB_ALWAYS() << DEB_VAR1(m_nb_frames);
	}

	if ((last_acq_frame_nb == m_nb_frames - 1) &&
	    (m_acq_state.get() == AcqState::Acquiring)) {
		DEB_ALWAYS() << "All frames acquired!";
		m_acq_state.set(AcqState::Saving);
	}

	if (last_saved_frame_nb == m_nb_frames - 1) {
		DEB_ALWAYS() << "All frames saved!";
		m_acq_state.set(AcqState::Finished);
	}
}


//*********************************************************************
//* SlsDetectorAcq
//*********************************************************************

class SlsDetectorAcq
{
	DEB_CLASS(DebModTest, "SlsDetectorAcq");
public:
	SlsDetectorAcq(std::string config_fname);
	~SlsDetectorAcq();

	void initSaving(string dir, string prefix, string suffix, int idx, 
			CtSaving::FileFormat fmt, CtSaving::SavingMode mode,
			int frames_per_file, bool overwrite=false);

	void setExpTime(double exp_time);
	void setNbAcqFrames(int nb_acq_frames);

	void start();
	void wait();
	void run();

	void setBin(Bin& bin);
	void setRoi(Roi& roi);

private:
	void printDefaults();

	Camera			m_cam;
	Interface		m_hw_inter;
	AcqState		m_acq_state;

	AutoPtr<Eiger>		m_eiger;
	AutoPtr<Eiger::Correction> m_corr;

	AutoPtr<ImageStatusCallback> m_img_status_cb;

	AutoPtr<CtControl>	m_ct;
	CtAcquisition		*m_ct_acq;
	CtSaving		*m_ct_saving;
	CtImage			*m_ct_image;
	CtBuffer		*m_ct_buffer;
#ifdef WITH_SPS_IMAGE
	CtSpsImage		*m_ct_display;
#endif
};

SlsDetectorAcq::SlsDetectorAcq(string config_fname)
	: m_cam(config_fname), m_hw_inter(m_cam)
{
	DEB_CONSTRUCTOR();

	switch (m_cam.getType()) {
	case EigerDet:
		m_eiger = new Eiger(&m_cam);
		m_corr = new Eiger::Correction(m_eiger);
		break;
	default:
		DEB_WARNING() << "Non-supported type: " << m_cam.getType();
	}

	m_ct = new CtControl(&m_hw_inter);

	m_ct_acq     = m_ct->acquisition();
	m_ct_saving  = m_ct->saving();
	m_ct_image   = m_ct->image();
	m_ct_buffer  = m_ct->buffer();
#ifdef WITH_SPS_IMAGE
	m_ct_display = m_ct->display();
#endif

	printDefaults();

	if (m_corr)
		m_ct->setReconstructionTask(m_corr);

	m_img_status_cb = new ImageStatusCallback(*m_ct, m_acq_state);
	m_ct->registerImageStatusCallback(*m_img_status_cb);
#ifdef WITH_SPS_IMAGE
	m_ct_display->setNames("_ccd_ds_", "slsdetector_live");
	m_ct_display->setActive(true);
#endif
	DEB_TRACE() << "All is OK!";
}

SlsDetectorAcq::~SlsDetectorAcq()
{
	DEB_DESTRUCTOR();
}

void SlsDetectorAcq::printDefaults()
{
	DEB_MEMBER_FUNCT();

	AcqMode acq_mode;
	m_ct_acq->getAcqMode(acq_mode);
	DEB_TRACE() << "Default " << DEB_VAR1(acq_mode);

	ImageType image_type;
	m_ct_image->getImageType(image_type);
	DEB_TRACE() << "Default " << DEB_VAR1(image_type);

	Size max_size;
	m_ct_image->getMaxImageSize(max_size);
	DEB_TRACE() << "Default " << DEB_VAR1(max_size);

	CtImage::ImageOpMode img_op_mode;
	m_ct_image->getMode(img_op_mode);
	DEB_TRACE() << "Default " << DEB_VAR1(img_op_mode);

	Bin bin;
	m_ct_image->getBin(bin);
	DEB_TRACE() << "Default " << DEB_VAR1(bin);

	Roi roi;
	m_ct_image->getRoi(roi);
	DEB_TRACE() << "Default " << DEB_VAR1(roi);
	
	double exp_time;
	m_ct_acq->getAcqExpoTime(exp_time);
	DEB_TRACE() << "Default " << DEB_VAR1(exp_time);

	int nb_frames;
	m_ct_acq->getAcqNbFrames(nb_frames);
	DEB_TRACE() << "Default " << DEB_VAR1(nb_frames);

}

void SlsDetectorAcq::start()
{
	DEB_MEMBER_FUNCT();

	m_ct->prepareAcq();
	m_acq_state.set(AcqState::Acquiring);
	DEB_TRACE() << "Starting acquisition";
   	m_ct->startAcq();
}

void SlsDetectorAcq::wait()
{
	DEB_MEMBER_FUNCT();
	m_acq_state.waitNot(AcqState::Acquiring | AcqState::Saving);
	DEB_TRACE() << "Acquisition finished";
	m_cam.waitState(Idle);
	DEB_TRACE() << "Camera finished";

}

void SlsDetectorAcq::run()
{
	DEB_MEMBER_FUNCT();
	start();
	wait();
}

void SlsDetectorAcq::initSaving(string dir, string prefix, string suffix, 
				int idx, CtSaving::FileFormat fmt, 
				CtSaving::SavingMode mode, int frames_per_file,
				bool overwrite)
{
	DEB_MEMBER_FUNCT();

	m_ct_saving->setDirectory(dir);
 	m_ct_saving->setPrefix(prefix);
	m_ct_saving->setSuffix(suffix);
	m_ct_saving->setNextNumber(idx);
	m_ct_saving->setFormat(fmt);
	m_ct_saving->setSavingMode(mode);
	m_ct_saving->setFramesPerFile(frames_per_file);
	m_ct_saving->setOverwritePolicy(overwrite ? CtSaving::Overwrite : 
						    CtSaving::Append);
}

void SlsDetectorAcq::setExpTime(double exp_time)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR1(exp_time);

	double lat_time = 1e-3;
	m_ct_acq->setAcqExpoTime(exp_time - lat_time);
	m_ct_acq->setLatencyTime(lat_time);
}

void SlsDetectorAcq::setNbAcqFrames(int nb_acq_frames)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR1(nb_acq_frames);

	m_ct_acq->setAcqNbFrames(nb_acq_frames);
}

void SlsDetectorAcq::setBin(Bin& bin)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR1(bin);

	m_ct_image->setBin(bin);
}

void SlsDetectorAcq::setRoi(Roi& roi)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR1(roi);

	Size max_size;
	m_ct_image->getMaxImageSize(max_size);
	Bin bin;
	m_ct_image->getBin(bin);
	max_size /= bin;
	Roi max_roi(Point(0, 0), max_size);
	DEB_TRACE() << DEB_VAR2(roi, max_roi);
	if (!max_roi.containsRoi(roi)) {
		Point tl = roi.getTopLeft();
		if (!max_roi.containsPoint(tl))
			THROW_HW_ERROR(InvalidValue) << DEB_VAR2(roi, max_roi);
		Point br = max_roi.getBottomRight();
		roi.setCorners(tl, br);
		DEB_WARNING() << "Restricting roi: " << DEB_VAR2(roi, max_roi);
	}

	m_ct_image->setRoi(roi);

}

//*********************************************************************
//* test_slsdetector_control
//*********************************************************************

void test_slsdetector_control(string config_fname, bool enable_debug = false)
{
	DEB_GLOBAL_FUNCT();

	if (enable_debug) {
		DebParams::enableTypeFlags(DebParams::AllFlags);
	}

	DEB_ALWAYS() << "Creating SlsDetectorAcq";
	SlsDetectorAcq acq(config_fname);
	DEB_ALWAYS() << "Done!";

	system("mkdir -p data");

	acq.initSaving("data", "img", ".edf", 0, CtSaving::EDF, 
		       CtSaving::AutoFrame, 1, true);

	DEB_ALWAYS() << "First run with default pars";
	acq.run();
	DEB_ALWAYS() << "Done!";

	double exp_time = 100e-3;
	acq.setExpTime(exp_time);

	int nb_acq_frames = 50;
	acq.setNbAcqFrames(nb_acq_frames);

	DEB_ALWAYS() << "Run " << DEB_VAR2(exp_time, nb_acq_frames);
	acq.run();
	DEB_ALWAYS() << "Done!";

	Bin bin(2);
	acq.setBin(bin);

	nb_acq_frames = 5;
	acq.setNbAcqFrames(nb_acq_frames);

	DEB_ALWAYS() << "Run " << DEB_VAR2(bin, nb_acq_frames);
	acq.run();
	DEB_ALWAYS() << "Done!";

	Roi roi = Roi(Point(256, 128), Size(256, 512));
	acq.setRoi(roi);

	DEB_ALWAYS() << "Run " << DEB_VAR1(roi);
	acq.run();
	DEB_ALWAYS() << "Done!";
}


//*********************************************************************
//* main
//*********************************************************************

int main(int argc, char *argv[])
{
	DEB_GLOBAL_FUNCT();

	string config_fname = getenv("EIGER_CONFIG");
	if (config_fname.empty()) {
		if (argc < 2) {
			DEB_ERROR() << "Must provide configuration file";
			exit(1);
		}
		config_fname = argv[1];
	}

	string par;
	if (argc > 2)
		par = argv[2];
	bool enable_debug = (par == "debug");

	try {
		test_slsdetector_control(config_fname, enable_debug);
	} catch (Exception& e) {
		DEB_ERROR() << "LIMA Exception: " << e;
	}
	return 0;
}
