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
# file :        SlsDetector.py
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
import time, string
import numpy as np
import PyTango
from collections import OrderedDict
from functools import partial
from itertools import chain
from multiprocessing import Process

from Lima import Core
from Lima import SlsDetector as SlsDetectorHw
from Lima.Server.AttrHelper import get_attr_4u, get_attr_string_value_list

def ConstListAttr(nl, vl=None, namespc=SlsDetectorHw.Defs):
    def g(x):
        n = ''
        was_cap = True
        for c in x:
            cap = c.isupper()
            sep = '_' if cap and not was_cap else ''
            n += sep + c.upper()
            was_cap = cap or not c.isalpha()
        return n

    if vl is None:
        vl = [getattr(namespc, n) for n in nl]
    return OrderedDict([(g(n), v) for n, v in zip(nl, vl)])


class SlsDetector(PyTango.Device_4Impl):

    Core.DEB_CLASS(Core.DebModApplication, 'LimaSlsDetector')


#------------------------------------------------------------------
#    Device constructor
#------------------------------------------------------------------

    MilliVoltSuffix = '_mv'

    ModelAttrs = ['parallel_mode',
                  'high_voltage',
                  'clock_div',
                  'fixed_clock_div',
                  'threshold_energy',
    ]

    def __init__(self,*args) :
        PyTango.Device_4Impl.__init__(self,*args)
        self.init_device()

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
        self.set_state(PyTango.DevState.ON)
        self.get_device_properties(self.get_device_class())

        self.cam = _SlsDetectorCam
        self.model = self.cam.getModel()

        self.init_list_attr()
        self.init_dac_adc_attr()

        self.proc_finished = self.cam.getProcessingFinishedEvent()
        self.proc_finished.registerStatusCallback(_SlsDetectorControl)

        if self.high_voltage > 0:
            self.model.setHighVoltage(self.high_voltage)
        if self.fixed_clock_div > 0:
            self.model.setFixedClockDiv(self.fixed_clock_div)
        if self.threshold_energy > 0:
            self.model.setThresholdEnergy(self.threshold_energy)

        self.cam.setTolerateLostPackets(self.tolerate_lost_packets)
        self.netdev_groups = [g.split(',') for g in self.netdev_groups]
        aff_array = self.pixel_depth_cpu_affinity_map
        if aff_array:
            flat_array = ','.join(aff_array).split(',')
            aff_array = np.array(map(partial(int, base=0), flat_array))
            aff_map = self.getPixelDepthCPUAffinityMapFromArray(aff_array)
            self.cam.setPixelDepthCPUAffinityMap(aff_map)

    def init_list_attr(self):
        nl = ['FullSpeed', 'HalfSpeed', 'QuarterSpeed', 'SuperSlowSpeed']
        self.__ClockDiv = ConstListAttr(nl)

        nl = ['Parallel', 'NonParallel', 'Safe']
        self.__ParallelMode = ConstListAttr(nl, namespc=SlsDetectorHw.Eiger)

        nl = ['PixelDepth4', 'PixelDepth8', 'PixelDepth16', 'PixelDepth32']
        bdl = map(lambda x: getattr(SlsDetectorHw, x), nl)
        self.__PixelDepth = OrderedDict([(str(bd), int(bd)) for bd in bdl])

    @Core.DEB_MEMBER_FUNCT
    def init_dac_adc_attr(self):
        nb_modules = self.cam.getNbDetSubModules()
        name_list, idx_list, milli_volt_list = self.model.getDACInfo()
        attr_name_list = map(lambda n: 'dac_' + n, name_list)
        data_list = zip(idx_list, milli_volt_list)
        self.dac_attr_idx_list = zip(attr_name_list, data_list)
        for name, data in self.dac_attr_idx_list:
            attr_data_dict = {
                'name': name, 
                'dtype': PyTango.DevLong,
                'dformat': PyTango.SPECTRUM,
                'max_dim_x': nb_modules,
                'fget': self.read_dac,
                'fset': self.write_dac,
            }
            attr_data = PyTango.AttrData.from_dict(attr_data_dict)
            self.add_attribute(attr_data)

            idx, has_mv = data
            if has_mv:
                attr_data_dict['name'] = name + self.MilliVoltSuffix
                attr_data = PyTango.AttrData.from_dict(attr_data_dict)
                self.add_attribute(attr_data)

        name_list, idx_list, factor_list, min_val_list = self.model.getADCInfo()
        attr_name_list = map(lambda n: 'adc_' + n, name_list)
        data_list = zip(idx_list, factor_list, min_val_list)
        self.adc_attr_idx_list = zip(attr_name_list, data_list)
        for name, data in self.adc_attr_idx_list:
            attr_data_dict = {
                'name': name, 
                'dtype': PyTango.DevDouble,
                'dformat': PyTango.SPECTRUM,
                'max_dim_x': nb_modules,
                'fget': self.read_adc,
            }
            attr_data = PyTango.AttrData.from_dict(attr_data_dict)
            self.add_attribute(attr_data)

    @Core.DEB_MEMBER_FUNCT
    def getAttrStringValueList(self, attr_name):
        return get_attr_string_value_list(self, attr_name)

    def __getattr__(self, name):
        if '_stats_' in name:
            stats_tok = name.split('_stats_')
            if stats_tok[1] in ['do_hist']:
                stats_name = '_'.join(stats_tok)
                return get_attr_4u(self, stats_name, SlsDetectorHw.SimpleStat)
        obj = self.cam
        for attr in self.ModelAttrs:
            if attr in name:
                obj = self.model
        return get_attr_4u(self, name, obj)

    @Core.DEB_MEMBER_FUNCT
    def read_config_fname(self, attr):
        deb.Return("config_fname=%s" % self.config_fname)
        attr.set_value(self.config_fname)

    @Core.DEB_MEMBER_FUNCT
    def putCmd(self, cmd):
        deb.Param("cmd=%s" % cmd)
        self.cam.putCmd(cmd);

    @Core.DEB_MEMBER_FUNCT
    def getCmd(self, cmd):
        deb.Param("cmd=%s" % cmd)
        resp = self.cam.getCmd(cmd);
        deb.Return("resp=%s" % resp)
        return resp

    @Core.DEB_MEMBER_FUNCT
    def read_dac_name_list(self, attr):
        dac_name_list = [n for n, (i, m) in self.dac_attr_idx_list]
        deb.Return("dac_name_list=%s" % (dac_name_list,))
        attr.set_value(dac_name_list)

    @Core.DEB_MEMBER_FUNCT
    def read_dac_name_list_mv(self, attr):
        mv = self.MilliVoltSuffix
        dac_name_list = [n + mv for n, (i, m) in self.dac_attr_idx_list if m]
        deb.Return("dac_name_list=%s" % (dac_name_list,))
        attr.set_value(dac_name_list)

    @Core.DEB_MEMBER_FUNCT
    def get_dac_name_mv(self, dac_name):
        mv = self.MilliVoltSuffix
        is_mv = dac_name.endswith(mv)
        if is_mv:
            dac_name = dac_name[:-len(mv)]
        return dac_name, is_mv

    @Core.DEB_MEMBER_FUNCT
    def read_dac(self, attr):
        dac_name, milli_volt = self.get_dac_name_mv(attr.get_name())
        deb.Param("dac_name=%s, milli_volt=%s" % (dac_name, milli_volt))
        idx, has_mv = dict(self.dac_attr_idx_list)[dac_name]
        val_list = self.cam.getDACList(idx, milli_volt)
        deb.Return("val_list=%s" % val_list)
        attr.set_value(val_list)

    @Core.DEB_MEMBER_FUNCT
    def write_dac(self, attr):
        dac_name, milli_volt = self.get_dac_name_mv(attr.get_name())
        idx, has_mv = dict(self.dac_attr_idx_list)[dac_name]
        deb.Param("dac_name=%s, milli_volt=%s" % (dac_name, milli_volt))
        for i, val in self.get_write_mod_idx_val_list(attr):
            self.cam.setDAC(i, idx, val, milli_volt)

    @Core.DEB_MEMBER_FUNCT
    def read_all_trim_bits(self, attr):
        val_list = self.model.getAllTrimBitsList()
        deb.Return("val_list=%s" % val_list)
        attr.set_value(val_list)

    @Core.DEB_MEMBER_FUNCT
    def write_all_trim_bits(self, attr):
        for i, val in self.get_write_mod_idx_val_list(attr):
            self.model.setAllTrimBits(i, val)

    @Core.DEB_MEMBER_FUNCT
    def get_write_mod_idx_val_list(self, attr):
        attr_name = attr.get_name()
        val_list = attr.get_write_value()
        deb.Param("attr_name=%s, val_list=%s" % (attr_name, val_list))
        msg = None
        nb_val = len(val_list)
        if (val_list < 0).sum() == nb_val:
            msg = 'Invalid %s: %s' % (attr_name, val_list)
        elif nb_val == 1:
            mod_idx_list = [-1]
        elif nb_val == self.cam.getNbDetSubModules():
            mod_idx_list = range(nb_val)
        else:
            msg = 'Invalid %s length: %s' % (att_name, val_list)
        if msg:
            deb.Error(msg)
            raise ValueError(msg)
        return [(i, val) for i, val in zip(mod_idx_list, val_list) if val >= 0]

    @Core.DEB_MEMBER_FUNCT
    def read_adc_name_list(self, attr):
        adc_name_list = zip(*self.adc_attr_idx_list)[0]
        deb.Return("adc_name_list=%s" % (adc_name_list,))
        attr.set_value(adc_name_list)

    @Core.DEB_MEMBER_FUNCT
    def read_adc(self, attr):
        idx, factor, min_val = dict(self.adc_attr_idx_list)[attr.get_name()]
        val_list = self.cam.getADCList(idx)
        out_arr = np.array(val_list, 'float64') * factor + min_val
        deb.Return("out_arr=%s" % out_arr)
        attr.set_value(out_arr)

    @Core.DEB_MEMBER_FUNCT
    def read_max_frame_rate(self, attr):
        time_ranges = self.model.getTimeRanges()
        max_frame_rate = 1 / time_ranges.min_frame_period / 1e3;
        deb.Return("max_frame_rate=%s" % max_frame_rate)
        attr.set_value(max_frame_rate)

    @Core.DEB_MEMBER_FUNCT
    def getNbBadFrames(self, port_idx):
        nb_bad_frames = self.cam.getNbBadFrames(port_idx);
        deb.Return("nb_bad_frames=%s" % nb_bad_frames)
        return nb_bad_frames

    @Core.DEB_MEMBER_FUNCT
    def getBadFrameList(self, port_idx):
        bad_frame_list = self.cam.getBadFrameList(port_idx);
        deb.Return("bad_frame_list=%s" % bad_frame_list)
        return bad_frame_list

    @Core.DEB_MEMBER_FUNCT
    def getStats(self, port_idx_stats_name):
        port_idx_str, stats_name = port_idx_stats_name.split(':')
        port_idx = int(port_idx_str)
        deb.Param("port_idx=%s, stats_name=%s");
        stats = self.cam.getStats(port_idx)
        stat = getattr(stats, stats_name)
        stat_data = [stat.min(), stat.max(), stat.ave(), stat.std(), stat.n()]
        deb.Return("stat_data=%s" % stat_data)
        return stat_data

    @Core.DEB_MEMBER_FUNCT
    def getStatsHistogram(self, port_idx_stats_name):
        port_idx_str, stats_name = port_idx_stats_name.split(':')
        port_idx = int(port_idx_str)
        deb.Param("port_idx=%s, stats_name=%s");
        stats = self.cam.getStats(port_idx)
        stat = getattr(stats, stats_name)
        stat_data = np.array(stat.hist).flatten()
        deb.Return("stat_data=%s" % stat_data)
        return stat_data

    def getCPUAffinityLen(self):
        return 5 + len(self.netdev_groups);

    @Core.DEB_MEMBER_FUNCT
    def getArrayFromPixelDepthCPUAffinityMap(self, aff_map):
        nb_pixel_depth = len(aff_map)
        aff_len = self.getCPUAffinityLen()
        aff_array = np.zeros((nb_pixel_depth, aff_len), 'int')
        for i, (pixel_depth, global_affinity) in enumerate(aff_map.items()):
            aff_array[i][:5] = (pixel_depth, 
                                global_affinity.recv.listeners, 
                                global_affinity.recv.writers, 
                                global_affinity.lima, 
                                global_affinity.other)
            for ng_aff in global_affinity.netdev:
                j = self.netdev_groups.index(ng_aff.name_list)
                aff_array[i][5 + j] = ng_aff.processing
        return aff_array

    @Core.DEB_MEMBER_FUNCT
    def getPixelDepthCPUAffinityMapFromArray(self, aff_array):
        aff_len = self.getCPUAffinityLen()
        err = ValueError("Invalid pixel_depth_cpu_affinity_map: "
                         "must be a list of %d-value tuples" % aff_len)
        if len(aff_array.shape) == 1:
            if len(aff_array) % aff_len != 0:
                raise err
            aff_array.resize((len(aff_array) / aff_len, aff_len))
        if aff_array.shape[1] != aff_len:
            raise err
        aff_map = {}
        CPUAffinity = SlsDetectorHw.CPUAffinity
        RecvCPUAffinity = SlsDetectorHw.RecvCPUAffinity
        NetDevRxQueueCPUAffinity = SlsDetectorHw.NetDevRxQueueCPUAffinity
        NetDevGroupCPUAffinity = SlsDetectorHw.NetDevGroupCPUAffinity
        GlobalCPUAffinity = SlsDetectorHw.GlobalCPUAffinity
        for aff_data in aff_array:
            aff_data = map(int, aff_data)
            pixel_depth, recv_l, recv_w, lima, other = aff_data[:5]
            netdev_aff = aff_data[5:]
            all_cpus = range(CPUAffinity.getNbSystemCPUs())
            recv_lw = [[6, 7], [9, 10]]
            indep_lw = True
            if indep_lw:
                recv_l = recv_lw
                recv_w = [map(lambda x: x + 12, l) for l in recv_l]
            else:
                recv_l = [[(x, x + 12) for x in r] for r in recv_lw]
                recv_w = recv_l
            recv_pt = [(8, 20), (11, 23)]
            recv_pt = zip(*([recv_pt] * 2))
            lima = list(range(6))
            lima += map(lambda x: x + 12, lima)
            lima.remove(0)
            other = [0]
            irq_aff = [0, (8, 20), (11, 23)]
            proc_aff = irq_aff
            def Affinity(*x):
                if type(x[0]) in [tuple, list]:
                    x = list(chain(*x))
                m = reduce(lambda a, b: a | b, map(lambda a: 1 << a, x))
                return CPUAffinity(m)
            global_affinity = GlobalCPUAffinity()
            recv_list = []
            for l, w, pt in zip(recv_l, recv_w, recv_pt):
                recv = RecvCPUAffinity()
                recv.listeners = map(Affinity, l)
                recv.writers = map(Affinity, w)
                recv.port_threads = map(Affinity, pt)
                recv_list.append(recv)
            global_affinity.recv = recv_list
            for i, r in enumerate(global_affinity.recv):
                s = "Recv[%d]:" % i
                def A(x):
                    return hex(long(x))
                s += " listeners=%s," % [A(x) for x in r.listeners]
                s += " writers=%s," % [A(x) for x in r.writers]
                s += " port_threads=%s" % [A(x) for x in r.port_threads]
                print(s)
            global_affinity.lima = Affinity(*lima)
            global_affinity.other = Affinity(*other)
            ng_aff_list = []
            for name_list, (irq, proc) in zip(self.netdev_groups, 
                                              zip(irq_aff, proc_aff)):
                ng_aff = NetDevGroupCPUAffinity()
                ng_aff.name_list = name_list
                ng_aff_queue = NetDevRxQueueCPUAffinity()
                ng_aff_queue.irq = Affinity(irq)
                ng_aff_queue.processing = Affinity(proc)
                ng_aff.queue_affinity = {-1: ng_aff_queue}
                ng_aff_list.append(ng_aff)
            global_affinity.netdev = ng_aff_list
            aff_map[pixel_depth] = global_affinity
        return aff_map

    @Core.DEB_MEMBER_FUNCT
    def read_netdev_groups(self, attr):
        netdev_groups = [','.join(g) for g in self.netdev_groups]
        deb.Return("aff_array=%s" % netdev_groups)
        attr.set_value(netdev_groups)

    @Core.DEB_MEMBER_FUNCT
    def write_netdev_groups(self, attr):
        netdev_groups = [g.split(',') for g in attr.get_write_value()]
        deb.Param("aff_array=%s" % netdev_groups)
        self.netdev_groups = netdev_groups

    @Core.DEB_MEMBER_FUNCT
    def read_pixel_depth_cpu_affinity_map(self, attr):
        aff_map = self.cam.getPixelDepthCPUAffinityMap()
        aff_array = self.getArrayFromPixelDepthCPUAffinityMap(aff_map)
        deb.Return("aff_array=%s" % aff_array)
        attr.set_value(aff_array)

    @Core.DEB_MEMBER_FUNCT
    def write_pixel_depth_cpu_affinity_map(self, attr):
        aff_array = attr.get_write_value()
        deb.Param("aff_array=%s" % aff_array)
        aff_map = self.getPixelDepthCPUAffinityMapFromArray(aff_array)
        self.cam.setPixelDepthCPUAffinityMap(aff_map)


class SlsDetectorClass(PyTango.DeviceClass):

    class_property_list = {}

    device_property_list = {
        'config_fname':
        [PyTango.DevString,
         "Path to the SlsDetector config file",[]],
        'full_config_fname':
        [PyTango.DevString,
         "In case of partial configuration, path to the full config file",[]],
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
        'tolerate_lost_packets':
        [PyTango.DevBoolean,
         "Initial tolerance to lost packets", True],
        'netdev_groups':
        [PyTango.DevVarStringArray,
         "List of network device groups, each group is a list of "
         "comma-separated interface names: [\"ethX,ethY\", \"ethZ,...\"]", []],
        'pixel_depth_cpu_affinity_map':
        [PyTango.DevVarStringArray,
         "Default PixelDepthCPUAffinityMap as a list of 5+n-value tuple "
         "strings (n is nb of netdev_groups): "
         "[\"<pixel_depth>,<recv_l>,<recv_w>,<lima>,<other>"
         "[,<netdev_grp1>,...]\", ...]", []],
        }

    cmd_list = {
        'getAttrStringValueList':
        [[PyTango.DevString, "Attribute name"],
         [PyTango.DevVarStringArray, "Authorized String value list"]],
        'putCmd':
        [[PyTango.DevString, "SlsDetector command"],
         [PyTango.DevVoid, ""]],
        'getCmd':
        [[PyTango.DevString, "SlsDetector command"],
         [PyTango.DevString, "SlsDetector response"]],
        'getNbBadFrames':
        [[PyTango.DevLong, "port_idx(-1=all)"],
         [PyTango.DevLong, "Number of bad frames"]],
        'getBadFrameList':
        [[PyTango.DevLong, "port_idx(-1=all)"],
         [PyTango.DevVarLongArray, "Bad frame list"]],
        'getStats':
        [[PyTango.DevString, "port_idx(-1=all):stats_name"],
         [PyTango.DevVarDoubleArray, "Statistics: min, max, ave, std, n"]],
        'getStatsHistogram':
        [[PyTango.DevString, "port_idx(-1=all):stats_name"],
         [PyTango.DevVarDoubleArray, "[[bin, count], ...]"]],
        }

    attr_list = {
        'config_fname':
        [[PyTango.DevString,
          PyTango.SCALAR,
          PyTango.READ]],
        'hostname_list':
        [[PyTango.DevString,
          PyTango.SPECTRUM,
          PyTango.READ, 64]],
        'dac_name_list':
        [[PyTango.DevString,
          PyTango.SPECTRUM,
          PyTango.READ, 64]],
        'dac_name_list_mv':
        [[PyTango.DevString,
          PyTango.SPECTRUM,
          PyTango.READ, 64]],
        'adc_name_list':
        [[PyTango.DevString,
          PyTango.SPECTRUM,
          PyTango.READ, 64]],
        'pixel_depth':
        [[PyTango.DevString,
          PyTango.SCALAR,
          PyTango.READ_WRITE]],
        'raw_mode':
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
        'max_frame_rate':
        [[PyTango.DevDouble,
          PyTango.SCALAR,
          PyTango.READ]],
        'tolerate_lost_packets':
        [[PyTango.DevBoolean,
          PyTango.SCALAR,
          PyTango.READ_WRITE]],
        'netdev_groups':
        [[PyTango.DevString,
          PyTango.SPECTRUM,
          PyTango.READ_WRITE, 64]],
        'pixel_depth_cpu_affinity_map':
        [[PyTango.DevLong,
          PyTango.IMAGE,
          PyTango.READ_WRITE, 64, 5]],
        'stats_do_hist':
        [[PyTango.DevBoolean,
          PyTango.SCALAR,
          PyTango.READ_WRITE]],
        'skip_frame_freq':
        [[PyTango.DevLong,
          PyTango.SCALAR,
          PyTango.READ_WRITE]],
        }

    def __init__(self,name) :
        PyTango.DeviceClass.__init__(self,name)
        self.set_type(name)


#----------------------------------------------------------------------------
# Plugin
#----------------------------------------------------------------------------
_SlsDetectorCam = None
_SlsDetectorHwInter = None
_SlsDetectorEiger = None
_SlsDetectorCorrection = None
_SlsDetectorControl = None

def get_control(config_fname, **keys) :
    global _SlsDetectorCam, _SlsDetectorHwInter, _SlsDetectorEiger
    global _SlsDetectorCorrection, _SlsDetectorControl
    if _SlsDetectorControl is None:
        full_config_fname = keys.pop('full_config_fname', None)
        if full_config_fname:
            p = Process(target=setup_partial_config,
                        args=(config_fname, full_config_fname))
            p.start()
            p.join()

        _SlsDetectorCam = SlsDetectorHw.Camera(config_fname)
        for i, n in enumerate(_SlsDetectorCam.getHostnameList()):
            print('Enabling: %s (%d)' % (n, i))
            _SlsDetectorCam.putCmd('activate 1', i)

        _SlsDetectorHwInter = SlsDetectorHw.Interface(_SlsDetectorCam)
        if _SlsDetectorCam.getType() == SlsDetectorHw.EigerDet:
            _SlsDetectorEiger = SlsDetectorHw.Eiger(_SlsDetectorCam)
            _SlsDetectorCorrection = _SlsDetectorEiger.createCorrectionTask()
        else:
            raise ValueError("Unknown detector type: %s" %
                             _SlsDetectorCam.getType())
        _SlsDetectorControl = Core.CtControl(_SlsDetectorHwInter)
        if _SlsDetectorCorrection:
            _SlsDetectorControl.setReconstructionTask(_SlsDetectorCorrection)

    return _SlsDetectorControl 

def get_tango_specific_class_n_device():
    return SlsDetectorClass, SlsDetector


#----------------------------------------------------------------------------
# Deactivate modules in partial config
#----------------------------------------------------------------------------

def setup_partial_config(config_fname, full_config_fname):
    import re

    cam = SlsDetectorHw.Camera(full_config_fname)
    full_hostname_list = cam.getHostnameList()
    print('Full config: %s' % ','.join(full_hostname_list))
    host_re_str = '([A-Za-z0-9]+)+?'
    host_re_obj = re.compile(host_re_str)
    re_obj = re.compile('^[ \\t]*hostname[ \\t]+(%s[^# ]+)' % host_re_str)
    partial_hostname_list = []
    with open(config_fname) as f:
        for l in f:
            m = re_obj.match(l)
            if m:
                s = m.groups()[0]
                partial_hostname_list = host_re_obj.findall(s)
                break
    print('Partial config: %s' % ','.join(partial_hostname_list))
    for i, n in enumerate(full_hostname_list):
        if n not in partial_hostname_list:
            print('Disabling: %s (%d)' % (n, i))
            cam.putCmd('activate 0', i)
    print('Partial config: Done!')
