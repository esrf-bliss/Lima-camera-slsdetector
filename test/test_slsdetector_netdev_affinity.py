from Lima import Core, SlsDetector
CPUAffinity = SlsDetector.CPUAffinity
NetDevGroupCPUAffinity = SlsDetector.NetDevGroupCPUAffinity
GlobalCPUAffinity = SlsDetector.GlobalCPUAffinity

import os
config_fname = os.environ['EIGER_CONFIG']
cam = SlsDetector.Camera(config_fname)
try:
	print cam.getNetworkParameter(SlsDetector.Defs.FlowCtrl10G)
	print cam.setNetworkParameter(SlsDetector.Defs.FlowCtrl10G, "0")
	print cam.setNetworkParameter(SlsDetector.Defs.FlowCtrl10G, "1")
except:
	print "Exception on NetworkParameter management"

global_aff_map = cam.getPixelDepthCPUAffinityMap()

if not global_aff_map:
	pixel_depth_list = [4, 8, 16, 32]
	for pixel_depth in pixel_depth_list:
		global_aff = GlobalCPUAffinity()
		global_aff.recv.listeners = CPUAffinity(0xf00)
		global_aff.recv.writers = CPUAffinity(0x0fc)
		global_aff.recv.lima = CPUAffinity(0x002)
		global_aff.recv.other = CPUAffinity(0x001)
		global_aff_map[pixel_depth] = global_aff

all_netdev = [('eth%d' % i) for i in range(10)]
data_netdev = ['eth3', 'eth5']
other_netdev = [i for i in all_netdev if i not in data_netdev]

netdev_groups = [(other_netdev, 0x001), (data_netdev, 0x002)]
ng_aff_list = []
for name_list, a in netdev_groups:
	ng_aff = NetDevGroupCPUAffinity()
	ng_aff.name_list = name_list
	ng_aff.processing = CPUAffinity(a)
	ng_aff_list.append(ng_aff)

for pixel_depth, global_aff in global_aff_map.items():
	global_aff.netdev = ng_aff_list

cam.setPixelDepthCPUAffinityMap(global_aff_map)
