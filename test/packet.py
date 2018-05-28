import sys
import os
import struct
import glob
from scipy import stats
from scapy.all import rdpcap, IP, UDP

HEADER_PACKET_SIZE = 40
DATA_PACKET_HEADER_SIZE = 48
DATA_PACKET_DATA_SIZE = 4096
DATA_PACKET_SIZE = DATA_PACKET_HEADER_SIZE + DATA_PACKET_DATA_SIZE

TSTAMP_CLOCK_FREQ = 10e6
NOMINAL_PACKET_XFER_TIME = 7.646e-6

pixel_depth = 16
frame_packets = 4 * pixel_depth

addr_map = {}
for d in glob.glob('/sys/class/net/*'):
    name = os.path.basename(d)
    addrfname = os.path.join(d, 'address')
    addr = open(addrfname).readline().strip()
    if addr != ':'.join(['00'] * 6):
        addr_map[addr] = name

fname = sys.argv[1]
capfile = rdpcap(fname)

ifaces = {}
start_ts = None
for i, x in enumerate(capfile):
    if not x.haslayer(IP):
        continue
    if start_ts is None:
        start_ts = x.time
    if not x.haslayer(UDP):
        continue
    u = x.getlayer(UDP)

    addr = x.dst
    iface = addr_map[addr]

    if iface not in ifaces:
        ifaces[iface] = {'ports': {}, 'start_dtime': None}
    idata = ifaces[iface]
    ports = idata['ports']

    port = u.dport
    if port not in ports:
        ports[port] = {'packets': []}

    p = u.payload
    if len(p) == HEADER_PACKET_SIZE:
        f = '>' + 'L' * 10
        d = struct.unpack(f, str(p))
        fnum = d[5]
        continue
    elif len(p) != DATA_PACKET_SIZE:
        raise ValueError('Invalid packet')

    h = str(p)[:DATA_PACKET_HEADER_SIZE]
    f = '<' + 'QLLQQHHHHLHBB'
    d = struct.unpack(f, h)
    fnum, explen, pnum, bunchid, dtime, modid, cx, cy, cz = d[0:9]
    debug, rrnum, dtype, ver = d[9:13]

    dtime /= TSTAMP_CLOCK_FREQ

    if idata['start_dtime'] is None:
        idata['start_dtime'] = dtime

    t = x.time - start_ts
    dt = dtime - idata['start_dtime']

    ports[port]['packets'].append([fnum, pnum, t, dt])

for iface, idata in ifaces.items():
    for port, d in idata['ports'].items():
        fnum, pnum, t, dt = d['packets'][-1]
        x_0 = int(0.4 * fnum * frame_packets)

        xfer_times = []
        xfer_delays = []

        def calc_xfer_time(data):
            if len(data[0]) < 2:
                return None
            slope, intercept, r_value, p_value, std_err = stats.linregress(*data)
            xfer_times.append(slope)
            xfer_delays.append(intercept)

        data = [[], []]
        for fnum, pnum, t, dt in d['packets']:
            x = fnum * frame_packets
            if (x < x_0) or (data[0] and (data[0][-1] == x)):
                continue
            if data[0]:
                xfer_time = (dt - data[1][-1]) / (x - data[0][-1])
                err = abs((xfer_time - NOMINAL_PACKET_XFER_TIME) / xfer_time)
                if err > 0.05:
                    calc_xfer_time(data)
                    data = [[], []]

            data[0].append(x)
            data[1].append(dt)
        calc_xfer_time(data)
        nb_xfer_times = len(xfer_times)
        if nb_xfer_times > 0:
            d['packet_xfer_time'] = sum(xfer_times) / nb_xfer_times
        else:
            d['packet_xfer_time'] = NOMINAL_PACKET_XFER_TIME
        delay_str = ''
        if nb_xfer_times > 1:
             x = stats.linregress(range(nb_xfer_times), xfer_delays)
             delay_str = ':%.5f' % x[0]
        print "packet_xfer_time[%d:%d%s]=%s" % (port, nb_xfer_times, delay_str,
                                                d['packet_xfer_time'])
        ofname = '.'.join(fname.split('.')[:-1]) + '_%s_%d.dat' % (iface, port)
        d['out_fname'] = ofname
        d['out_file'] = open(d['out_fname'], 'wt')

for iface, idata in ifaces.items():
    for port, d in idata['ports'].items():
        for fnum, pnum, t, dt in d['packets']:
            xt = fnum * frame_packets * d['packet_xfer_time']
            fnum += float(pnum) / frame_packets - 1
            ofile = d['out_file']
            ofile.write('%6.2f\t%13.9f\t%11.7f\t%11.7f\t%11.7f\n' % 
                        (fnum, t, dt, t - dt, dt - xt))

    
