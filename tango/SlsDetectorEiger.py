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
# file :        SlsDetectorEiger.py
#
# description : Python source for the PSI/SlsDetector Eiger and its commands.
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
#    SlsDetectorEiger device class
#------------------------------------------------------------------

class SlsDetectorEiger(SlsDetector):

    Core.DEB_CLASS(Core.DebModApplication, 'LimaSlsDetectorEiger')


#------------------------------------------------------------------
#    Device constructor
#------------------------------------------------------------------

    ModelAttrs = ['apply_corrections',
                  'signed_image_mode',
                  'parallel_mode',
                  'high_voltage',
                  'clock_div',
                  'fixed_clock_div',
                  'threshold_energy',
                  'tx_frame_delay',
                  'fpga_frame_ptr_diff',
    ]

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

        if self.high_voltage > 0:
            self.model.setHighVoltage(self.high_voltage)
        if self.fixed_clock_div > 0:
            self.model.setFixedClockDiv(self.fixed_clock_div)
        if self.threshold_energy > 0:
            self.model.setThresholdEnergy(self.threshold_energy)
        sys.stderr.flush()

    def init_list_attr(self):
        SlsDetector.init_list_attr(self)

        nl = ['FullSpeed', 'HalfSpeed', 'QuarterSpeed']
        self.__ClockDiv = ConstListAttr(nl)

        nl = ['NonParallel', 'Parallel']
        self.__ParallelMode = ConstListAttr(nl, namespc=SlsDetectorHw.Eiger)

    def __getattr__(self, name):
        obj = None
        for attr in self.ModelAttrs:
            if attr in name:
                obj = self.model
        return self.getDevAttr(name, obj)

    @Core.DEB_MEMBER_FUNCT
    def read_all_trim_bits(self, attr):
        val_list = self.model.getAllTrimBitsList()
        deb.Return("val_list=%s" % val_list)
        attr.set_value(val_list)

    @Core.DEB_MEMBER_FUNCT
    def write_all_trim_bits(self, attr):
        for i, val in self.get_write_mod_idx_val_list(attr):
            self.model.setAllTrimBits(i, val)


#------------------------------------------------------------------
#    SlsDetectorEiger class
#------------------------------------------------------------------

class SlsDetectorEigerClass(SlsDetectorClass):

    class_property_list = {}
    class_property_list.update(SlsDetectorClass.class_property_list)

    device_property_list = {
        'high_voltage':
        [PyTango.DevShort,
         "Initial detector high voltage (V) "
         "(set to 150 if already tested)", 0],
        'fixed_clock_div':
        [PyTango.DevShort,
         "Initial detector fixed-clock-div (0, 1)", 0],
        'threshold_energy':
        [PyTango.DevLong,
         "Initial detector threshold energy (eV)", 0],
        'apply_corrections':
        [PyTango.DevBoolean,
         "Apply frame corrections", True],
        }
    device_property_list.update(SlsDetectorClass.device_property_list)

    cmd_list = {
        }
    cmd_list.update(SlsDetectorClass.cmd_list)

    attr_list = {
        'apply_corrections':
        [[PyTango.DevBoolean,
          PyTango.SCALAR,
          PyTango.READ_WRITE]],
        'signed_image_mode':
        [[PyTango.DevBoolean,
          PyTango.SCALAR,
          PyTango.READ_WRITE]],
        'threshold_energy':
        [[PyTango.DevLong,
          PyTango.SCALAR,
          PyTango.READ_WRITE]],
        'all_trim_bits':
        [[PyTango.DevLong,
          PyTango.SPECTRUM,
          PyTango.READ_WRITE, 64]],
        'high_voltage':
        [[PyTango.DevLong,
          PyTango.SCALAR,
          PyTango.READ_WRITE]],
        'clock_div':
        [[PyTango.DevString,
          PyTango.SCALAR,
          PyTango.READ_WRITE]],
        'fixed_clock_div':
        [[PyTango.DevBoolean,
          PyTango.SCALAR,
          PyTango.READ_WRITE]],
        'parallel_mode':
        [[PyTango.DevString,
          PyTango.SCALAR,
          PyTango.READ_WRITE]],
        'skip_frame_freq':
        [[PyTango.DevLong,
          PyTango.SCALAR,
          PyTango.READ_WRITE]],
        'tx_frame_delay':
        [[PyTango.DevLong,
          PyTango.SCALAR,
          PyTango.READ_WRITE]],
        'fpga_frame_ptr_diff':
        [[PyTango.DevULong,
          PyTango.SPECTRUM,
          PyTango.READ, 64]],
        }
    attr_list.update(SlsDetectorClass.attr_list)

    def __init__(self,name) :
        SlsDetectorClass.__init__(self,name)


#----------------------------------------------------------------------------
# Plugin
#----------------------------------------------------------------------------
_SlsDetectorEiger = None

def get_control(config_fname, full_config_fname=None, apply_corrections=None,
                detector_id=0, **keys) :
    global _SlsDetectorEiger

    _Cam, _HwInter, _Control = get_slsdetector_objs()
    if _Control is not None:
        return _Control 
        check_partial_config(config_name, full_config_name)

    _Cam = SlsDetectorHw.Camera(config_fname, int(detector_id))
    det_type = _Cam.getType()
    if det_type == SlsDetectorHw.EigerDet:
        for i, n in enumerate(_Cam.getHostnameList()):
            print('Enabling: %s (%d)' % (n, i))
            _Cam.setModuleActive(i, True)

    _HwInter = SlsDetectorHw.Interface(_Cam)
    if det_type != SlsDetectorHw.EigerDet:
        raise ValueError("Bad detector type: %s" % det_type)

    apply_corrections = to_bool(apply_corrections, True)

    _SlsDetectorEiger = SlsDetectorHw.Eiger(_Cam)
    _SlsDetectorEiger.setApplyCorrections(apply_corrections);
    _Reconstruction = _SlsDetectorEiger.getReconstruction();
    _HwInter.setReconstruction(_Reconstruction)

    _Control = Core.CtControl(_HwInter)

    set_slsdetector_objs(_Cam, _HwInter, _Control)
    return _Control 

def get_tango_specific_class_n_device():
    return SlsDetectorEigerClass, SlsDetectorEiger


