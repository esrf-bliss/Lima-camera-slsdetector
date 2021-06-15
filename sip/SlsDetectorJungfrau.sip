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

// namespace SlsDetector
// {
// typedef std::array<Data, 3> JungfrauGainDataSet;
// };

%MappedType SlsDetector::JungfrauGainDataSet
{
%TypeHeaderCode
#include "SlsDetectorJungfrau.h"
#include <array>
using namespace lima;
%End

%ConvertToTypeCode
    constexpr int S = 3;
    using cppType = std::array<Data, S>;
    auto sipElemType = sipFindType("Data");
    if (sipIsErr == NULL) {
        bool is_ok = (PySequence_Check(sipPy) && (PySequence_Size(sipPy) == S));
        for (int i = 0; is_ok && i < S; ++i) {
            PyObject *p = PySequence_Fast_GET_ITEM(sipPy, i);
            is_ok = sipCanConvertToType(p, sipElemType, SIP_NOT_NONE);
        }
        return is_ok;
    }

    std::unique_ptr<cppType> a(new cppType);

    for (int i = 0; i < S; ++i) {
        PyObject *py_d = PySequence_Fast_GET_ITEM(sipPy, i);
        int state;
        void *p = sipConvertToType(py_d, sipElemType, 0, SIP_NOT_NONE, &state,
	                           sipIsErr);
        auto deleter = [=](Data *d) { sipReleaseType(d, sipElemType, state); };
        using Ptr = std::unique_ptr<Data, std::function<void(Data *)>>;
        Ptr d(reinterpret_cast<Data *>(p), deleter);
        if (*sipIsErr)
	    return 0;
        (*a)[i] = *d;
    }

    *sipCppPtr = a.release();
    return sipGetState(sipTransferObj);
%End

%ConvertFromTypeCode
    constexpr int S = 3;
    auto deleter = [](PyObject *o) { Py_DECREF(o); };
    using Ptr = std::unique_ptr<PyObject, std::function<void(PyObject *)>>;
    auto sipElemType = sipFindType("Data");

    // Create the Python list of the correct length.
    Ptr l(PyList_New(S), deleter);
    if (!l)
       return NULL;

    for (int i = 0; i < S; ++i) {
       auto cppData = std::make_unique<Data>((*sipCpp)[i]);
       PyObject *d = sipConvertFromNewType(cppData.get(), sipElemType, NULL);
       if (!d)
           return NULL;
       cppData.release();
       PyList_SET_ITEM(l.get(), i, d);
    }

    // Return the Python list.
    return l.release();
%End
};


namespace SlsDetector
{

%TypeHeaderCode
#include "SlsDetectorJungfrau.h"
%End


class Jungfrau : public SlsDetector::Model
{

%TypeHeaderCode
#include "SlsDetectorJungfrau.h"
%End

 public:
	class GainPed
	{
	public:
		struct Calib {
			SlsDetector::JungfrauGainDataSet gain_map;
			SlsDetector::JungfrauGainDataSet ped_map;
			void clear();
		};

		enum MapType { Map16, Map32 };

		GainPed(SlsDetector::Jungfrau *jungfrau);

		void setMapType(
			SlsDetector::Jungfrau::GainPed::MapType  map_type);
		void getMapType(
			SlsDetector::Jungfrau::GainPed::MapType& map_type);

		void updateImageSize(Size size, bool raw);

		void getDefaultCalib(
			SlsDetector::Jungfrau::GainPed::Calib& calib /Out/);

		void setCalib(
			const SlsDetector::Jungfrau::GainPed::Calib& calib);
		void getCalib(
			SlsDetector::Jungfrau::GainPed::Calib& calib /Out/);

		void processFrame(Data& data, Data& proc /In,Out/);
	};

	enum ImgSrc { Raw, GainPedCorr };

	Jungfrau(SlsDetector::Camera *cam);

	virtual void getFrameDim(FrameDim& frame_dim /Out/, bool raw = false);
	virtual void getAcqFrameDim(FrameDim& frame_dim /Out/,
				    bool raw = false);

	virtual void getDetMap(Data& det_map /Out/);

	SinkTaskBase *createImgProcTask() /Factory/;

	virtual std::string getName();
	virtual void getPixelSize(double& x_size /Out/, double& y_size /Out/);

	virtual void getDACInfo(std::vector<std::string>& name_list /Out/,
				std::vector<int>& idx_list /Out/, 
				std::vector<int>& milli_volt_list /Out/);
	virtual void getADCInfo(std::vector<std::string>& name_list /Out/,
				std::vector<int>& idx_list /Out/,
				std::vector<double>& factor_list /Out/,
				std::vector<double>& min_val_list /Out/);

	virtual void getTimeRanges(SlsDetector::TimeRanges& time_ranges /Out/);

	void setHighVoltage(int  hvolt);
	void getHighVoltage(int& hvolt /Out/);

	void setThresholdEnergy(int  thres);
	void getThresholdEnergy(int& thres /Out/);

	void setImgProcConfig(std::string  config);
	void getImgProcConfig(std::string &config /Out/);

	void readGainADCMaps(Data& gain_map /Out/, Data& adc_map /Out/,
			     unsigned long& frame /In,Out/);

	void readGainPedProcMap(Data& proc_map /Out/,
	     			unsigned long& frame /In,Out/);
	void readAveMap(Data& proc_map /Out/, unsigned long& nb_frames /Out/,
			unsigned long& frame /In,Out/);

	void setGainPedMapType(Jungfrau::GainPed::MapType  map_type);
	void getGainPedMapType(Jungfrau::GainPed::MapType& map_type /Out/);

	void setGainPedCalib(const SlsDetector::Jungfrau::GainPed::Calib& calib);
	void getGainPedCalib(SlsDetector::Jungfrau::GainPed::Calib& calib /Out/);

	void setImgSrc(Jungfrau::ImgSrc  img_src);
	void getImgSrc(Jungfrau::ImgSrc& img_src /Out/);

	virtual bool isXferActive();

	virtual SlsDetector::Reconstruction *getReconstruction();

 protected:
	virtual void updateImageSize();

	virtual bool checkSettings(SlsDetector::Defs::Settings settings);

	virtual void prepareAcq();
	virtual void startAcq();
	virtual void stopAcq();
};

}; // namespace SlsDetector
