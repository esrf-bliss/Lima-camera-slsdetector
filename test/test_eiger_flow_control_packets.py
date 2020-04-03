import sys
import struct
from getopt import getopt

from Lima.SlsDetector import BebShell, BebFpgaMem

class Eiger10GFifo:

    ControlReg = 0xC4200000
    StatusReg = 0xC4200004
    DataReg = 0xC4200008

    StatusFields = (
        ("TxLocalFault", (1, 0)),
        ("RxLocalFault", (1, 1)),
        ("Synchronization", (0xf, 2)),
        ("Alignment", (1, 6)),
        ("RxLinkStatus", (1, 7)),
        ("MgtTxReady", (1, 12)),
        ("AlignStatus", (1, 13)),

        ("RxFifoAlmostEmpty", (1, 26)),
        ("RxFifoEmpty", (1, 27)),
        ("RxStartOfFrame", (1, 28)),
        ("RxEndOfFrame", (1, 29)),
    )

    ControlFields = (
        ("LoopBack", (1, 8)),
        ("PowerDown", (1, 9)),
        ("ResetLocalFault", (1, 10)),
        ("ResetRxLinkStatus", (1, 11)),
        ("TestEnable", (1, 12)),
        ("TestSelect", (3, 13)),
    )

    def __init__(self, name):
        self.name = name
        self.sh = BebShell(self.name)
        self.mem = BebFpgaMem(self.sh)

        self.sync_packet()

    def __del__(self):
        del self.mem
        del self.sh

    def read_status(self):
        return self.mem.read(self.StatusReg)

    def read_status_field(self, f):
        mask, shift = dict(self.StatusFields)[f]
        return (self.read_status() >> shift) & mask

    def read_data(self):
        return self.mem.read(self.DataReg)

    def sync_packet(self):
        while not (self.read_status_field('RxFifoEmpty') or
                   self.read_status_field('RxStartOfFrame')):
            self.read_data()

    def read_packet(self):
        packet = b''
        while not self.read_status_field('RxFifoEmpty'):
            is_sof = self.read_status_field('RxStartOfFrame')
            if not packet and not is_sof:
                raise RuntimeError('First data is not start of frame')
            is_eof = self.read_status_field('RxEndOfFrame')
            data = self.read_data()
            packet += struct.pack('>I', data)
            if is_eof:
                return packet
        if packet:
            raise RuntimeError('Did not find end of frame')

    def write_control(self, data):
        val = self.mem.read(self.ControlReg)
        print("Writing control: val=0x%08x, val=0x%08x" % (val, val | data))
        self.mem.write(self.ControlReg, val | data)
        self.mem.write(self.ControlReg, val)

    def write_control_fields(self, **kws):
        d = dict(self.ControlFields)
        data = sum([((x & d[n][0]) << d[n][1]) for n, x in kws.items()])
        self.write_control(data)

    def reset_status(self):
        self.write_control_fields(ResetLocalFault=1, ResetRxLinkStatus=1)


def mac_str(addr):
    return ':'.join(map(lambda x: '%02x' % x,  addr))


class EthernetPacket:

    MAC_CONTROL_TYPE = 0x8808

    def __init__(self, data):
        self.data = data

        struct_format = '>' + 'BBBBBB' + 'BBBBBB' + 'H'
        self.unpacked = list(struct.unpack(struct_format, data[:14]))
        self.dst = self.unpacked[0:6]
        self.src = self.unpacked[6:12]
        self.eth_type = self.unpacked[12]

        self.pause = None
        self.quanta = None
        if self.is_mac_control():
            self.unpacked += list(struct.unpack('HH', data[14:18]))
            self.pause = self.unpacked[13]
            self.quanta = self.unpacked[14]

    def is_mac_control(self):
        return self.eth_type == self.MAC_CONTROL_TYPE

    def __str__(self):
        blist = list(map(lambda x: '%02x' % x,  self.data))
        blen = 2 * 8
        s = ['len=%s' % len(self.data)]
        for x in range(0, len(blist), blen):
            s += [' '.join(blist[x:x+blen])]
        s += ['dst=%s, src=%s, type=%04x' % 
              (mac_str(self.dst), mac_str(self.src), self.eth_type)]
        if self.is_mac_control():
            s += ['pause=0x%04x, quanta=0x%04x' % (self.pause, self.quanta)]
        return '\n'.join(s)
        
def main():
    forever = False
    print_all = False
    reset_status = False
    read_status = False
    write_fields = None

    opts, args = getopt(sys.argv[1:], 'afrsw:')
    for opt, val in opts:
        if opt == '-a':
            print_all = True
        if opt == '-f':
            forever = True
        if opt == '-r':
            reset_status = True
        if opt == '-s':
            read_status = True
        if opt == '-w':
            write_fields = eval('dict(%s)' % val)

    name = args[0]

    fifo = Eiger10GFifo(name)

    if read_status:
        status = fifo.read_status()
        r = bin(status)[2:]
        s = '0' * (32 - len(r)) + r
        status_str = ' '.join([s[x:x+8] for x in range(0, 32, 8)])
        print('status=%s' % status_str)
        for n, (m, s) in fifo.StatusFields:
            print("%-20s = %s" % (n, fifo.read_status_field(n)))
        exit(0)
    elif reset_status:
        fifo.reset_status()
        exit(0)
    elif write_fields:
        fifo.write_control_fields(**write_fields)
        exit(0)

    while True:
        data = fifo.read_packet()
        if data is None:
            if forever:
                continue
            print("Fifo is empty")
            exit(0)

        packet = EthernetPacket(data)
        if packet.is_mac_control() or print_all:
            print("------------")
            print(packet)



if __name__ == '__main__':
    main()

