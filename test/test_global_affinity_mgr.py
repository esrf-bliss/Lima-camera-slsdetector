import socket
from Lima import Core, SlsDetector

NetDevs = {
    'lid29pwr9': [f'mlx100c{cpu}p0' for cpu in range(2)],
    'lisgeiger1': [f'eth{2 + i * 2 + 1}' for i in range(2)],
    'ldegjfrau1': [f'eth{2 + i}' for i in range(2)],
}

hostname = socket.gethostname()
net_devs = NetDevs[hostname]

mgrs = {}
for net_dev in net_devs:
    mgr = SlsDetector.NetDevRxQueueMgr()
    mgr.setDev(net_dev)
    rq_mgrs = mgrs.setdefault("rx_queue", [])
    rq_mgrs.append(mgr)
    mgr = SlsDetector.IrqMgr(net_dev)
    irq_mgrs = mgrs.setdefault("irq", [])
    irq_mgrs.append(mgr)

print('RxQueues:', [mgr.getRxQueueList() for mgr in mgrs["rx_queue"]])
print('Irqs:', [mgr.getIrqList() for mgr in mgrs["irq"]])

if False:
    Core.DebParams.setTypeFlags(Core.DebParams.AllFlags)
    mgr.getIrqList()

if hostname != 'lid29pwr9':
    exit(0)
   
pixel_depth_cpu_affinity_map_str = """
{16: (((CPU(  2), CPU(  3)), (CPU(  4), CPU(  5)),
       (CPU( 34), CPU( 35)), (CPU( 36), CPU( 37)),
       (CPU( 66), CPU( 67)), (CPU( 68), CPU( 69)),
       (CPU( 98), CPU( 99)), (CPU(100), CPU(101))),
      CPU(  6,  38,  70, 102),
      CPU(*chain(range(  7,  32), range( 39,  64),
                 range( 71,  96), range(103, 128))),
      CPU(0, 64),
      (('enP5p1s0f0,enP5p1s0f1,enP48p1s0f0,enP48p1s0f1,mlx100c0p1,mlx100c1p1',
         {-1: (CPU( 32), CPU( 96))}),
       ('mlx100c0p0', {-1: (CPU(  1), CPU( 33))}),
       ('mlx100c1p0', {-1: (CPU( 65), CPU( 97))}))),
}
"""
