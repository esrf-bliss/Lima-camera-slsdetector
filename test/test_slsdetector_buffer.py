import os
import sys

slsdetector_dir = os.path.join(os.path.dirname(sys.argv[0]), os.pardir)
lima_dir = os.path.join(slsdetector_dir, os.pardir, os.pardir)
control_test_dir = os.path.join(lima_dir, 'control', 'test')
sys.path.append(control_test_dir)
from testbufferalloc import *

from Lima import SlsDetector

def get_slsdetector_hw_inter(config=None):
  config = config or os.environ['EIGER_CONFIG']
  cam = SlsDetector.Camera(config)
  eiger = SlsDetector.Eiger(cam)
  hw_inter = SlsDetector.Interface(cam)
  return hw_inter, (cam, eiger)

def main():
  frame_rate = 500
  readout_time = 100e-6
  config = TestBufferAlloc.get_default_config(frame_rate, readout_time)
  hw_inter, deps = get_slsdetector_hw_inter()
  test = TestBufferAlloc(hw_inter, deps, config)
  test.run()

if __name__ == '__main__':
  main()

