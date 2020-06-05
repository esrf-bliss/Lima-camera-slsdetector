from Lima import Core, SlsDetector
import time
import os
import sys
import getopt
from collections import namedtuple
from contextlib import contextmanager

AcqPars = namedtuple('AcqPars',
                     ['trig_mode', 'nb_frames', 'expo_time', 'lat_time'])

class TestIntTrig:
  def __init__(self, hw_inter):
    self.hw_inter = hw_inter
    self.ct = Core.CtControl(self.hw_inter)
    self.acq = self.ct.acquisition()
    self.sync = self.hw_inter.getHwCtrlObj(Core.HwCap.Sync)

  def run(self, acq_pars):
    self.acq.setTriggerMode(acq_pars.trig_mode)
    self.acq.setAcqNbFrames(acq_pars.nb_frames)
    self.acq.setAcqExpoTime(acq_pars.expo_time)
    self.acq.setLatencyTime(acq_pars.lat_time)

    class Cb(Core.CtControl.ImageStatusCallback):
      def __init__(self, frame_ts):
        super().__init__()
        self.last_acquired = -1
        self.frame_ts = frame_ts
      def imageStatusChanged(self, status):
        if status.LastImageAcquired != self.last_acquired:
          self.frame_ts.append(time.time())
          self.last_acquired = status.LastImageAcquired

    self.frame_ts = []
    cb = Cb(self.frame_ts)
    self.ct.registerImageStatusCallback(cb)

    self.ct.prepareAcq()

    ranges = self.sync.getValidRanges()
    print("Valid ranges: ")
    print(f"  min_exp_time={ranges.min_exp_time:.7f},"
          f"  max_exp_time={ranges.max_exp_time:.7f}")
    print(f"  min_lat_time={ranges.min_lat_time:.7f},"
          f"  max_lat_time={ranges.max_lat_time:.7f}")

    print("Starting acquisition ...")
    if acq_pars.trig_mode == Core.IntTrig:
      self.runIntTrig(acq_pars)
    else:
      self.runIntTrigMult(acq_pars)

  def runIntTrig(self, acq_pars):
    t0 = time.time()
    self.ct.startAcq()
    while self.hw_inter.getStatus().det != Core.DetIdle:
      pass
    trigger_delay = time.time() - t0
    while self.ct.getStatus().AcquisitionStatus != Core.AcqReady:
      pass
    acq_delay = time.time() - t0

    print([(t - t0) for t in self.frame_ts])
    acq_time = acq_pars.nb_frames * (acq_pars.expo_time + acq_pars.lat_time)
    overhead = trigger_delay - acq_time
    print(f"Trigger delay: {trigger_delay:.3f} "
          f"({acq_time:.3f} + {overhead:.3f})")
    print(f"Acquisition finished after {acq_delay:.3f} sec")

  def runIntTrigMult(self, acq_pars):
    trigger_delay = []
    for i in range(nb_frames):
      print(f"Sending start #{i}")
      self.ct.startAcq()
      t0 = time.time()
      while self.hw_inter.getStatus().det != Core.DetIdle:
        pass
      trigger_delay.append(time.time() - t0)

    t0 = time.time()
    while self.ct.getStatus().AcquisitionStatus != Core.AcqReady:
      pass
    delay = time.time() - t0

    last_delay = trigger_delay.pop(-1)
    ave_trigger_delay = sum(trigger_delay) / len(trigger_delay)
    overhead = ave_trigger_delay - expo_time
    print(f"Average trigger_delay={ave_trigger_delay:.3f} "
          f"({expo_time:.3f} + {overhead:.3f})")

    delay += last_delay - ave_trigger_delay
    print(f"Acquisition finished {delay:.3f} sec after trigger sequence")


if __name__ == '__main__':
  verbose = False
  nb_frames = 10
  expo_time = 0.01
  lat_time = 2e-3
  trig_mode = 'IntTrigMult'

  opts, args = getopt.getopt(sys.argv[1:], 'vn:e:l:t:')
  for opt, val in opts:
    if opt == '-v':
      verbose = True
    if opt == '-n':
      nb_frames = int(val)
    if opt == '-e':
      expo_time = float(val)
    if opt == '-l':
      lat_time = float(val)
    if opt == '-t':
      trig_mode = val

  config_file = os.environ.get('EIGER_CONFIG', None)
  if args:
    config_file = args[0]

  acq_pars = AcqPars(getattr(Core, trig_mode), nb_frames, expo_time, lat_time)

  if verbose:
    db = Core.DebParams
    db.setTypeFlagsNameList(['Funct','Trace','Param','Return','Warning','Fatal'])
    #db.setTypeFlagsNameList(['Funct', 'Trace','Fatal'])
    #db.setModuleFlagsNameList(['Camera'])

  cam = SlsDetector.Camera(config_file)
  model = SlsDetector.Eiger(cam)
  hw_inter = SlsDetector.Interface(cam)

  test = TestIntTrig(hw_inter)
  test.run(acq_pars)

  stream_info = hw_inter.getLastStreamInfo()
  print(f"Stream info: encoding={stream_info.encoding}, "
        f"packed_size={stream_info.packed_size}")
  stream_stats = hw_inter.latchStreamStatistics()
  print(f"Stream stats: ave_speed={stream_stats.ave_speed() / 1e6:.3f} MB/s")
