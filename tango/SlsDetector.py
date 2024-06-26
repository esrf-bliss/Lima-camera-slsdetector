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
# description : Python common source for the PSI/SlsDetector and its commands.
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
import sys
import time, string
import numpy as np
import PyTango
from collections import OrderedDict
from functools import partial, reduce
from itertools import chain
from multiprocessing import Process
import re

from Lima import Core
from Lima import SlsDetector as SlsDetectorHw
from Lima.Server.AttrHelper import get_attr_4u, get_attr_string_value_list

NumAffinity = long if sys.version_info.major == 2 else int
NumAffinity = lambda a: NumAffinity(a.getMask()) \
			if type(a) is SlsDetectorHw.CPUAffinity else a

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


def to_bool(x, default_val=False):
    if x is None:
        return default_val
    elif type(x) is str:
        x = x.lower()
        if x == 'true':
            return True
        elif x == 'false':
            return False
        else:
            return bool(int(x))
    else:
        return bool(x)

#------------------------------------------------------------------
#    SlsDetector base device class
#------------------------------------------------------------------

class SlsDetector(PyTango.Device_4Impl):

    Core.DEB_CLASS(Core.DebModApplication, 'LimaSlsDetector')

    AttrNameRe = re.compile('_(?P<klass>[^_]+)__(?P<member>.+)')

#------------------------------------------------------------------
#    Device constructor
#------------------------------------------------------------------

    MilliVoltSuffix = '_mv'

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
        self.buffer = self.cam.getBuffer()

        self.__Prefix2Group = {'stats' : SlsDetectorHw.SimpleStat,
                               'buffer' : self.buffer}
        groups = '|'.join(self.__Prefix2Group.keys())
        self.attr_group_re = re.compile('(?P<action>(read|write))_' +
                                        '(?P<group>(%s))_' % groups +
                                        '(?P<attr>.+)')

        self.init_list_attr()
        self.init_dac_adc_attr()

        if self.initial_acq_params:
            self.perform_initial_acq(self.initial_acq_params)

        self.proc_finished = self.cam.getProcessingFinishedEvent()
        self.proc_finished.registerStatusCallback(_SlsDetectorControl)

        self.cam.setTolerateLostPackets(self.tolerate_lost_packets)
        aff_arr = self.pixel_depth_cpu_affinity_map
        if aff_arr:
            aff_str = ' '.join(aff_arr)
            aff_map = self.getPixelDepthCPUAffinityMapFromString(aff_str)
            self.printPixelDepthCPUAffinityMap(aff_map)
            self.cam.setPixelDepthCPUAffinityMap(aff_map)

        if self.buffer_max_memory:
            deb.Always("Setting buffer_max_memory: %s" % self.buffer_max_memory)
            self.buffer.setMaxMemory(int(self.buffer_max_memory))
        buffer_resize_policy = self.buffer_resize_policy
        packet_fifo_depth = self.buffer_packet_fifo_depth
        if packet_fifo_depth:
            if buffer_resize_policy and buffer_resize_policy.title() != "Manual":
                    raise ValueError("buffer_packet_fifo_depth requires "
                                     "Manual ResizePolicy")
            deb.Always("Setting Manual buffer_resize_policy")
            self.buffer.setResizePolicy(self.buffer.Manual)
            deb.Always("Setting buffer_packet_fifo_depth: %s" %
                       packet_fifo_depth)
            self.buffer.setPacketFifoDepth(int(packet_fifo_depth))
        elif buffer_resize_policy:
            deb.Always("Setting %s buffer_resize_policy" %
                       buffer_resize_policy.title())
            pol = self.__ResizePolicy[buffer_resize_policy.upper()]
            self.buffer.setResizePolicy(pol)

    def init_list_attr(self):
        nl = ['GenericDet', 'EigerDet', 'JungfrauDet']
        self.__Type = ConstListAttr(nl, namespc=SlsDetectorHw)

        nl = ['PixelDepth4', 'PixelDepth8', 'PixelDepth16', 'PixelDepth32']
        bdl = map(lambda x: getattr(SlsDetectorHw, x), nl)
        self.__PixelDepth = OrderedDict([(str(bd), int(bd)) for bd in bdl])

        nl = ['Auto', 'Manual', 'Max']
        BufferMgr = SlsDetectorHw.BufferMgr
        self.__ResizePolicy = ConstListAttr(nl, namespc=BufferMgr)

    @Core.DEB_MEMBER_FUNCT
    def init_dac_adc_attr(self):
        nb_modules = self.cam.getNbDetModules()
        name_list, idx_list, milli_volt_list = self.model.getDACInfo()
        attr_name_list = map(lambda n: 'dac_' + n, name_list)
        data_list = zip(idx_list, milli_volt_list)
        self.dac_attr_idx_list = list(zip(attr_name_list, data_list))
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
        self.adc_attr_idx_list = list(zip(attr_name_list, data_list))
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
    def perform_initial_acq(self, params):
        deb.Always("Performing initial setup acquisition: %s ..." % params)
        ct = _SlsDetectorControl
        acq = ct.acquisition()
        for x in params.split(','):
            n, v = x.split('=')
            attr_name = 'set' + ''.join([i.title() for i in n.split('_')])
            eval('acq.%s(%s)' % (attr_name, v))
        ct.prepareAcq()
        ct.startAcq()
        while ct.getStatus().AcquisitionStatus != Core.AcqReady:
            time.sleep(0.1)
        deb.Always("Done!")

    @Core.DEB_MEMBER_FUNCT
    def getAttrStringValueList(self, attr_name):
        return get_attr_string_value_list(self, attr_name)

    @Core.DEB_MEMBER_FUNCT
    def getDevAttr(self, name, obj=None):
        m = self.attr_group_re.match(name)
        if m:
            name = '%s_%s' % (m.group('action'), m.group('attr'))
            obj = self.__Prefix2Group[m.group('group')]
        m = self.AttrNameRe.match(name)
        if m and (m.group('klass') != 'SlsDetector'):
            member = m.group('member')
            if member in ['Type', 'PixelDepth', 'ResizePolicy']:
                return getattr(self, '_SlsDetector__' + member)
        obj = obj or self.cam
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
        elif nb_val == self.cam.getNbDetModules():
            mod_idx_list = range(nb_val)
        else:
            msg = 'Invalid %s length: %s' % (att_name, val_list)
        if msg:
            deb.Error(msg)
            raise ValueError(msg)
        return [(i, val) for i, val in zip(mod_idx_list, val_list) if val >= 0]

    @Core.DEB_MEMBER_FUNCT
    def read_adc_name_list(self, attr):
        adc_name_list = list(zip(*self.adc_attr_idx_list))[0]
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
    def getStats(self, recv_idx_stats_name):
        recv_idx_str, stats_name = recv_idx_stats_name.split(':')
        recv_idx = int(recv_idx_str)
        deb.Param("recv_idx=%s, stats_name=%s");
        stats = self.cam.getStats(recv_idx)
        stat = getattr(stats, stats_name)
        stat_data = [stat.min(), stat.max(), stat.ave(), stat.std(), stat.n()]
        deb.Return("stat_data=%s" % stat_data)
        return stat_data

    @Core.DEB_MEMBER_FUNCT
    def getStatsHistogram(self, recv_idx_stats_name):
        recv_idx_str, stats_name = recv_idx_stats_name.split(':')
        recv_idx = int(recv_idx_str)
        deb.Param("recv_idx=%s, stats_name=%s");
        stats = self.cam.getStats(recv_idx)
        stat = getattr(stats, stats_name)
        stat_data = np.array(stat.hist).flatten()
        deb.Return("stat_data=%s" % stat_data)
        return stat_data

    @Core.DEB_MEMBER_FUNCT
    def getPixelDepthCPUAffinityMapFromString(self, aff_str):
        CPUAffinity = SlsDetectorHw.CPUAffinity
        RecvCPUAffinity = SlsDetectorHw.RecvCPUAffinity
        NetDevRxQueueCPUAffinity = SlsDetectorHw.NetDevRxQueueCPUAffinity
        NetDevGroupCPUAffinity = SlsDetectorHw.NetDevGroupCPUAffinity
        GlobalCPUAffinity = SlsDetectorHw.GlobalCPUAffinity
        Mask = lambda x: CPUAffinity(hex(x).encode())
        def CPU(*x):
            if type(x[0]) in [tuple, list]:
                x = list(chain(*x))
            m = reduce(lambda a, b: a | b, map(lambda a: 1 << a, x))
            return Mask(m)
        aff_map_raw = eval(aff_str)
        self.expandPixelDepthRefs(aff_map_raw)
        aff_map = {}
        for pixel_depth, aff_data in aff_map_raw.items():
            recv_aff, acq, lima, other, netdev_aff = aff_data[:5]
            rx_netdev = list(aff_data[5]) if len(aff_data) > 5 else []
            global_aff = GlobalCPUAffinity()
            recv_list = []
            for listeners in recv_aff:
                recv = RecvCPUAffinity()
                recv.listeners = list(listeners)
                recv_list.append(recv)
            global_aff.recv = recv_list
            global_aff.acq = acq
            global_aff.lima = lima
            global_aff.other = other
            ng_aff_list = []
            for name_list, queue_data in netdev_aff:
                ng_aff = NetDevGroupCPUAffinity()
                ng_aff.name_list = name_list.split(',')
                queue_aff = {}
                for queue, (irq, proc) in queue_data.items():
                    ng_aff_queue = NetDevRxQueueCPUAffinity()
                    ng_aff_queue.irq = irq
                    ng_aff_queue.processing = proc
                    queue_aff[queue] = ng_aff_queue
                ng_aff.queue_affinity = queue_aff
                ng_aff_list.append(ng_aff)
            global_aff.netdev = ng_aff_list
            global_aff.rx_netdev = rx_netdev
            aff_map[pixel_depth] = global_aff
        return aff_map

    @Core.DEB_MEMBER_FUNCT
    def getStringFromPixelDepthCPUAffinityMap(self, aff_map, use_cpu=True):
        pixel_depth_aff_list = []
        def cpu_str(a):
            cpu_list = [str(i) for i in range(128) if NumAffinity(a) & (1 << i)]
            return 'CPU(%s)' % ', '.join(cpu_list)
        def mask_str(a):
            return 'Mask(0x%x)' % NumAffinity(a)
        def aff_2_str(a):
            if type(a) in [tuple, list]:
                str_list = list(map(aff_2_str, a))
                if len(str_list) == 1:
                    str_list.append('')
                return '(%s)' % ', '.join(str_list)
            f = cpu_str if use_cpu else mask_str
            return f(a)
        for pixel_depth, global_aff in sorted(aff_map.items()):
            recv_list = []
            for r in global_aff.recv:
                recv_list.append(r.listeners)
            recv_str = aff_2_str(recv_list)
            acq_str = aff_2_str(global_aff.acq)
            lima_str = aff_2_str(global_aff.lima)
            other_str = aff_2_str(global_aff.other)
            netdev_grp_list = []
            for netdev_grp in global_aff.netdev:
                name_str = ','.join(netdev_grp.name_list)
                queue_list = [('%d: %s)' % (q, aff_2_str((a.irq, 
                                                          a.processing))))
                              for q, a in netdev_grp.queue_affinity.items()]
                queue_str = ','.join(queue_list)
                netdev_str = '"%s": {%s}' % (name_str, queue_str)
                netdev_grp_list.append(netdev_str)
            netdev_grp_str = '(%s)' % ', '.join(netdev_grp_list)
            aff_data = [recv_str, acq_str, lima_str, other_str, netdev_grp_str]
            if global_aff.rx_netdev:
                aff_data.append(str(tuple(global_aff.rx_netdev)))
            aff_str = ', '.join(aff_data)
            pixel_depth_aff_list.append('%d: (%s)' % (pixel_depth, aff_str))
        return '{%s}' % ', '.join(pixel_depth_aff_list)

    @Core.DEB_MEMBER_FUNCT
    def expandPixelDepthRefs(self, aff_map):
        for k, v in aff_map.items():
            if type(v) != str:
                continue
            re_str = '^@(%s)$' % '|'.join(self.__PixelDepth.keys())
            re_obj = re.compile(re_str)
            m = re_obj.match(v)
            if not m:
                err = 'Invalid pixel_depth_cpu_affinity_map entry: %s' % v
                raise ValueError(err)
            o = int(m.group(1))
            v = aff_map[o]
            if type(v) == str:
                raise ValueError('Invalid reference to pixel depth %d' % o)
            aff_map[k] = v

    @Core.DEB_MEMBER_FUNCT
    def printPixelDepthCPUAffinityMap(self, aff_map):
        for pixel_depth, global_aff in sorted(aff_map.items()):
            deb.Always('PixelDepth: %d' % pixel_depth)
            self.printGlobalAffinity(global_aff)

    @Core.DEB_MEMBER_FUNCT
    def printGlobalAffinity(self, global_aff):
        def A(x):
            return hex(NumAffinity(x))
        for i, r in enumerate(global_aff.recv):
            s = "Recv[%d]:" % i
            s += " listeners=%s" % [A(x) for x in r.listeners]
            deb.Always('  ' + s)
        acq, lima, other = global_aff.acq, global_aff.lima, global_aff.other
        deb.Always('  Acq=%s, Lima=%s, Other=%s' % (A(acq), A(lima), A(other)))
        for netdev_grp in global_aff.netdev:
            s = "NetDevGroup[%s]: {" % ','.join(netdev_grp.name_list)
            l = []
            for queue, queue_aff in netdev_grp.queue_affinity.items():
                l.append('%d: (%s, %s)' % (queue, A(queue_aff.irq), 
                                           A(queue_aff.processing)))
            s += ','.join(l) + "}"
            deb.Always('  ' + s)
        if global_aff.rx_netdev:
            s = "RxNetDev=%s" % (global_aff.rx_netdev,)
            deb.Always('  ' + s)

    @Core.DEB_MEMBER_FUNCT
    def read_pixel_depth_cpu_affinity_map(self, attr):
        aff_map = self.cam.getPixelDepthCPUAffinityMap()
        aff_str = self.getStringFromPixelDepthCPUAffinityMap(aff_map)
        deb.Return("aff_str=%s" % aff_str)
        attr.set_value(aff_str)

    @Core.DEB_MEMBER_FUNCT
    def write_pixel_depth_cpu_affinity_map(self, attr):
        aff_str = attr.get_write_value()
        deb.Param("aff_str=%s" % aff_str)
        aff_map = self.getPixelDepthCPUAffinityMapFromString(aff_str)
        self.printPixelDepthCPUAffinityMap(aff_map)
        self.cam.setPixelDepthCPUAffinityMap(aff_map)

    @Core.DEB_MEMBER_FUNCT
    def clearAllBuffers(self):
        buffer = self.cam.getBuffer()
        buffer.clearAllBuffers()

    @Core.DEB_MEMBER_FUNCT
    def releaseBuffers(self):
        buffer = self.cam.getBuffer()
        buffer.releaseBuffers()


#------------------------------------------------------------------
#    SlsDetector base class
#------------------------------------------------------------------

class SlsDetectorClass(PyTango.DeviceClass):

    class_property_list = {}

    device_property_list = {
        'config_fname':
        [PyTango.DevString,
         "Path to the SlsDetector config file",[]],
        'full_config_fname':
        [PyTango.DevString,
         "In case of partial configuration, path to the full config file",[]],
        'detector_id':
        [PyTango.DevUShort,
         "Detector ID in multi-detector environment", 0],
        'initial_acq_params':
        [PyTango.DevString,
         "Initial acquisition parameters: "
         "acq_expo_time=0.01,acq_nb_frames=10,...", ""],
        'tolerate_lost_packets':
        [PyTango.DevBoolean,
         "Initial tolerance to lost packets", True],
        'pixel_depth_cpu_affinity_map':
        [PyTango.DevVarStringArray,
         "Default PixelDepthCPUAffinityMap as Python string(s) defining a dict: "
         "{<pixel_depth>: <global_affinity>}, being global_affinity a tuple: "
         "(<recv_list>, <acq>, <lima>, <other>, <netdev_grp_list>"
         "[, <rx_netdev>]), where recv_list is a list of tuples of "
         "listeners affinities, acq, lima and other are affinities, "
         "netdev_grp_list is a list of tuples in the form: "
         "(<comma_separated_netdev_name_list>, <rx_queue_affinity_map>), the "
         "latter in the form of: {<queue>: (<irq>, <processing>)}, and "
         "rx_netdev is an optional list of Rx netdev names. "
         "Each affinity can be expressed by one of the functions: Mask(mask) "
         "or CPU(<cpu1>[, ..., <cpuN>]) for independent CPU enumeration", []],
        'buffer_max_memory':
        [PyTango.DevString,
         "The maximum memory (percent) for image packet buffers (Fifo length), "
         "similar to LimaCCDs.BufferMaxMemory", []],
        'buffer_resize_policy':
        [PyTango.DevString,
         "The initial packet Fifo Resize Policy [Auto|Manual|Max]", []],
        'buffer_packet_fifo_depth':
        [PyTango.DevString,
         "The initial packet Fifo length (implies Manual ResizePolicy)", []],
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
        'getStats':
        [[PyTango.DevString, "recv_idx(-1=all):stats_name"],
         [PyTango.DevVarDoubleArray, "Statistics: min, max, ave, std, n"]],
        'getStatsHistogram':
        [[PyTango.DevString, "recv_idx(-1=all):stats_name"],
         [PyTango.DevVarDoubleArray, "[[bin, count], ...]"]],
        'clearAllBuffers':
        [[PyTango.DevVoid, ""],
         [PyTango.DevVoid, ""]],
        'releaseBuffers':
        [[PyTango.DevVoid, ""],
         [PyTango.DevVoid, ""]],
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
        'type':
        [[PyTango.DevString,
          PyTango.SCALAR,
          PyTango.READ]],
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
        'max_frame_rate':
        [[PyTango.DevDouble,
          PyTango.SCALAR,
          PyTango.READ]],
        'tolerate_lost_packets':
        [[PyTango.DevBoolean,
          PyTango.SCALAR,
          PyTango.READ_WRITE]],
        'pixel_depth_cpu_affinity_map':
        [[PyTango.DevString,
          PyTango.SCALAR,
          PyTango.READ_WRITE]],
        'stats_do_hist':
        [[PyTango.DevBoolean,
          PyTango.SCALAR,
          PyTango.READ_WRITE]],
        'buffer_max_memory':
        [[PyTango.DevShort,
          PyTango.SCALAR,
          PyTango.READ_WRITE]],
        'buffer_max_nb_buffers':
        [[PyTango.DevLong,
          PyTango.SCALAR,
          PyTango.READ]],
        'buffer_resize_policy':
        [[PyTango.DevString,
          PyTango.SCALAR,
          PyTango.READ_WRITE]],
        'buffer_packet_fifo_depth':
        [[PyTango.DevLong,
          PyTango.SCALAR,
          PyTango.READ_WRITE]],
        'last_frame_caught':
        [[PyTango.DevLong,
          PyTango.SCALAR,
          PyTango.READ]],
        }

    def __init__(self,name) :
        PyTango.DeviceClass.__init__(self,name)
        self.set_type(name)


#----------------------------------------------------------------------------
# Deactivate modules in partial config
#----------------------------------------------------------------------------

def check_partial_config(config_name, full_config_name):
    if full_config_fname:
        p = Process(target=setup_partial_config,
                    args=(config_fname, full_config_fname))
        p.start()
        p.join()
    
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
            cam.setModuleActive(i, False)
    print('Partial config: Done!')


#----------------------------------------------------------------------------
# Plugin
#----------------------------------------------------------------------------
_SlsDetectorCam = None
_SlsDetectorHwInter = None
_SlsDetectorControl = None
    
def set_slsdetector_objs(cam, hwinter, control):
    global _SlsDetectorCam, _SlsDetectorHwInter, _SlsDetectorControl
    _SlsDetectorCam = cam
    _SlsDetectorHwInter = hwinter
    _SlsDetectorControl = control
    
def get_slsdetector_objs():
    return _SlsDetectorCam, _SlsDetectorHwInter, _SlsDetectorControl
