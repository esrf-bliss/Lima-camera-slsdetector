import sys
import time
import contextlib
import numpy as np
from PyTango import DeviceProxy

dev_name = 'id00/slsdetector/eiger500k'

if len(sys.argv) > 1:
    dev_name = sys.argv[1]

eiger_dev = DeviceProxy(dev_name)
data = []

ref = eiger_dev.fpga_frame_ptr_diff
nb_hosts = len(ref)
zeros = np.zeros((nb_hosts,), 'uint32')
if (ref != zeros).any():
    raise RuntimeError(f'Invalid reference: {ref}')

max_val = np.array(zeros)
max_i = np.array(zeros)
t0 = None
print('Ready')
while True:
    i = len(data)
    d = eiger_dev.fpga_frame_ptr_diff
    d > max_val
    if (d != ref).all():
        t = time.time()
        if t0 is None:
            t0 = t
            print('Starting')
        data.append(((t - t0), d))
    elif i > 0:
        break

elapsed = time.time() - t0
print(f'Finished: {elapsed:.3f} sec, {len(data)} points')

ofname = '/tmp/fpga_frame_ptr_diff.dat'
with contextlib.closing(open(ofname, 'wt')) as f:
    for t0, d in data:
        f.write('\t'.join([str(x) for x in [t0] + list(d)]) + '\n')
print(f'Saved {ofname}')
