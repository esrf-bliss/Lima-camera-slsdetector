from Lima import Core, SlsDetector
import os
import time

class DebTypeFlags:
	def __init__(self, flags):
		self.flags = flags
		self.orig = Core.DebParams.getTypeFlags()
	def __enter__(self):
		Core.DebParams.setTypeFlags(self.flags)		
	def __exit__(self, *args):
		Core.DebParams.setTypeFlags(self.orig)

config_file = os.environ['EIGER_CONFIG']
cam = SlsDetector.Camera(config_file)
eiger = SlsDetector.Eiger(cam)
hw_inter = SlsDetector.Interface(cam)
ct = Core.CtControl(hw_inter)

def do_acq():
	acq = ct.acquisition()
	acq.setAcqExpoTime(0.1)
	ct.prepareAcq()
	ct.startAcq()
	while ct.getStatus().AcquisitionStatus == Core.AcqRunning:
		time.sleep(0.1)
	print "*** Acq finished ***"

def test(cam):
	print "** Calling getPixelDepthCPUAffinityMap"
	m = cam.getPixelDepthCPUAffinityMap()
	pixel_depth = 16
	print "** Calling m[pixel_depth]"
	sys_affinity = m[pixel_depth]
	print "** Calling setPixelDepthCPUAffinityMap"
	cam.setPixelDepthCPUAffinityMap({pixel_depth: sys_affinity})
	print "** Calling m.items()"
	print "m.items()[0][1]==sys_affinity=%s" % \
		(m.items()[0][1] == sys_affinity)
	print "** Calling m.items()"
	for i, (pixel_depth, sys_affinity) in enumerate(m.items()):
		d = map(long, (pixel_depth, 
			       sys_affinity.recv, 
			       sys_affinity.lima, 
			       sys_affinity.other))
		print "#%d: %s" % (i, d)
	print "** Calling setPixelDepthCPUAffinityMap"
	cam.setPixelDepthCPUAffinityMap({pixel_depth: sys_affinity})

do_acq()
with DebTypeFlags(Core.DebParams.AllFlags):
	for i in range(2):
		test(cam)
