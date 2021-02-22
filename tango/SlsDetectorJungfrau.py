############################################################################
# This file is part of LImA, a Library for Image Acquisition
#
# Copyright (C) : 2009-2011
# European Synchrotron Radiation Facility
# BP 220, Grenoble 38043
# FRANCE
#
# This is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 3 of the License, or
# (at your option) any later version.
#
# This software is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, see <http://www.gnu.org/licenses/>.
############################################################################
#=============================================================================
#
# file :        SlsDetectorJungfrau.py
#
# description : Python source for the PSI/SlsDetector and its commands.
#                The class is derived from Device. It represents the
#                CORBA servant object which will be accessed from the
#                network. All commands which can be executed on the
#                SlsDetector are implemented in this file.
#
# project :     TANGO Device Server
#
# copyleft :    European Synchrotron Radiation Facility
#               BP 220, Grenoble 38043
#               FRANCE
#
#=============================================================================
#         (c) - Bliss - ESRF
#=============================================================================
#

from .SlsDetector import *

#------------------------------------------------------------------
#    SlsDetectorJungfrau device class
#------------------------------------------------------------------

class SlsDetectorJungfrau(SlsDetector):

    Core.DEB_CLASS(Core.DebModApplication, 'LimaSlsDetectorJungfrau')


#------------------------------------------------------------------
#    Device constructor
#------------------------------------------------------------------

    ModelAttrs = ['img_proc_config',
                  'img_src',
                  'gain_ped_map_type',
    ]

    GainPedCalibAttrRe = re.compile('((?P<action>read|write)_)?'
                                    '(?P<select>gain|ped)_'
                                    '(?P<gain>[0-2])_calib_map')

    GainPedProcAttrRe = re.compile('((?P<action>read|write)_)?'
                                   'gain_ped_proc_map(?P<map_type>16|32)')

    def __init__(self,*args) :
        SlsDetector.__init__(self,*args)

#------------------------------------------------------------------
#    Device destructor
#------------------------------------------------------------------
    def delete_device(self):
        pass

#------------------------------------------------------------------
#    Device initialization
#------------------------------------------------------------------
    @Core.DEB_MEMBER_FUNCT
    def init_device(self):
        SlsDetector.init_device(self)

        self.gain_ped_bckgnd = None

    def init_list_attr(self):
        SlsDetector.init_list_attr(self)

        nl = ['Map16', 'Map32']
        klass = SlsDetectorHw.Jungfrau.GainPed
        self.__GainPedMapType = ConstListAttr(nl, namespc=klass)

        nl = ['Raw', 'GainPedCorr']
        klass = SlsDetectorHw.Jungfrau
        self.__ImgSrc = ConstListAttr(nl, namespc=klass)

    def __getattr__(self, name):
        m = self.GainPedCalibAttrRe.match(name)
        if m:
            name = f'{m.group("action")}_gain_ped_calib_map'
            return partial(getattr(self, name), map_select=m.group("select"),
                           gain=int(m.group("gain")))
        m = self.GainPedProcAttrRe.match(name)
        if m:
            name = f'{m.group("action")}_gain_ped_proc_map'
            return partial(getattr(self, name), map_type=m.group("map_type"))
        obj = None
        for attr in self.ModelAttrs:
            if attr in name:
                obj = self.model
        return self.getDevAttr(name, obj)

    @Core.DEB_MEMBER_FUNCT
    def read_det_map(self, attr):
        det_map = self.model.getDetMap();
        attr.set_value(det_map.buffer)

    @Core.DEB_MEMBER_FUNCT
    def read_gain_map(self, attr):
        jungfrau = _SlsDetectorJungfrau
        gain_data, adc_data, frame = jungfrau.readGainADCMaps(-1)
        deb.Return("frame=%s, gain_data=%s" % (frame, gain_data))
        attr.set_value(gain_data.buffer)

    @Core.DEB_MEMBER_FUNCT
    def read_adc_map(self, attr):
        jungfrau = _SlsDetectorJungfrau
        gain_data, adc_data, frame = jungfrau.readGainADCMaps(-1)
        deb.Return("frame=%s, adc_data=%s" % (frame, adc_data))
        attr.set_value(adc_data.buffer)

    @Core.DEB_MEMBER_FUNCT
    def getGainPedProcMap(self, map_type):
        jungfrau = _SlsDetectorJungfrau
        map_type_nb = self.__GainPedMapType[f'MAP{map_type}']
        if jungfrau.getGainPedMapType() != map_type_nb:
            raise ValueError('Invalid map_type %s' % map_type);
        return jungfrau.readGainPedProcMap(-1)

    @Core.DEB_MEMBER_FUNCT
    def read_gain_ped_proc_map(self, attr, map_type):
        proc_data, frame = self.getGainPedProcMap(map_type)
        data = proc_data.buffer
        if self.gain_ped_bckgnd is not None:
            data = data - self.gain_ped_bckgnd
        deb.Return("frame=%s, data=%s" % (frame, data))
        attr.set_value(data)

    @Core.DEB_MEMBER_FUNCT
    def setGainPedBackground(self, level=0):
        proc_data, frame = self.getGainPedProcMap('32')
        self.gain_ped_bckgnd = proc_data.buffer.copy() - level

    @Core.DEB_MEMBER_FUNCT
    def clearGainPedBackground(self):
        self.gain_ped_bckgnd = None

    @Core.DEB_MEMBER_FUNCT
    def read_gain_ped_calib_map(self, attr, map_select, gain):
        c = _SlsDetectorJungfrau.getGainPedCalib()
        d = c.gain_map[gain] if map_select == 'gain' else c.ped_map[gain]
        deb.Return("%s_%d=%s" % (map_select, gain, d))
        attr.set_value(d.buffer)


#------------------------------------------------------------------
#    SlsDetectorJungfrau class
#------------------------------------------------------------------

class SlsDetectorJungfrauClass(SlsDetectorClass):

    class_property_list = {}
    class_property_list.update(SlsDetectorClass.class_property_list)

    device_property_list = {
        }
    device_property_list.update(SlsDetectorClass.device_property_list)

    cmd_list = {
        'setGainPedBackground':
        [[PyTango.DevULong, "level"],
         [PyTango.DevVoid, ""]],
        'clearGainPedBackground':
        [[PyTango.DevVoid, ""],
         [PyTango.DevVoid, ""]],
        }
    cmd_list.update(SlsDetectorClass.cmd_list)

    attr_list = {
        'img_proc_config':
        [[PyTango.DevString,
          PyTango.SCALAR,
          PyTango.READ_WRITE]],
        'img_src':
        [[PyTango.DevString,
          PyTango.SCALAR,
          PyTango.READ_WRITE]],
        'det_map':
        [[PyTango.DevULong,
          PyTango.IMAGE,
          PyTango.READ, 8192, 8192]],
        'gain_map':
        [[PyTango.DevUChar,
          PyTango.IMAGE,
          PyTango.READ, 8192, 8192]],
        'adc_map':
        [[PyTango.DevUShort,
          PyTango.IMAGE,
          PyTango.READ, 8192, 8192]],
        'gain_ped_map_type':
        [[PyTango.DevString,
          PyTango.SCALAR,
          PyTango.READ_WRITE]],
        'gain_ped_proc_map16':
        [[PyTango.DevUShort,
          PyTango.IMAGE,
          PyTango.READ, 8192, 8192]],
        'gain_ped_proc_map32':
        [[PyTango.DevULong,
          PyTango.IMAGE,
          PyTango.READ, 8192, 8192]],
        'gain_0_calib_map':
        [[PyTango.DevDouble,
          PyTango.IMAGE,
          PyTango.READ, 8192, 8192]],
        'gain_1_calib_map':
        [[PyTango.DevDouble,
          PyTango.IMAGE,
          PyTango.READ, 8192, 8192]],
        'gain_2_calib_map':
        [[PyTango.DevDouble,
          PyTango.IMAGE,
          PyTango.READ, 8192, 8192]],
        'ped_0_calib_map':
        [[PyTango.DevDouble,
          PyTango.IMAGE,
          PyTango.READ, 8192, 8192]],
        'ped_1_calib_map':
        [[PyTango.DevDouble,
          PyTango.IMAGE,
          PyTango.READ, 8192, 8192]],
        'ped_2_calib_map':
        [[PyTango.DevDouble,
          PyTango.IMAGE,
          PyTango.READ, 8192, 8192]],
        }
    attr_list.update(SlsDetectorClass.attr_list)

    def __init__(self,name) :
        SlsDetectorClass.__init__(self,name)


#----------------------------------------------------------------------------
# Plugin
#----------------------------------------------------------------------------
_SlsDetectorJungfrau = None

def get_control(config_fname, **keys) :
    global _SlsDetectorJungfrau

    _Cam, _HwInter, _Control = get_slsdetector_objs()
    if _Control is not None:
        return _Control 

    det_id = 0
    _Cam = SlsDetectorHw.Camera(config_fname, det_id)
    det_type = _Cam.getType()

    _HwInter = SlsDetectorHw.Interface(_Cam)

    if det_type != SlsDetectorHw.JungfrauDet:
        raise ValueError("Bad detector type: %s" % det_type)

    _SlsDetectorJungfrau = SlsDetectorHw.Jungfrau(_Cam)
    _Reconstruction = _SlsDetectorJungfrau.getReconstruction();
    _HwInter.setReconstruction(_Reconstruction)

    _Control = Core.CtControl(_HwInter)

    _ImgProc = _SlsDetectorJungfrau.createImgProcTask()
    if _ImgProc:
        ext_op_mgr = _Control.externalOperation()
        alias = 'SlsDetectorImgProc'
        stage_level = 10
        soft_op = ext_op_mgr.addOp(Core.USER_SINK_TASK, alias, stage_level)
        soft_op.setSinkTask(_ImgProc)

    set_slsdetector_objs(_Cam, _HwInter, _Control)
    return _Control 

def get_tango_specific_class_n_device():
    return SlsDetectorJungfrauClass, SlsDetectorJungfrau

