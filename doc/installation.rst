Installation of Eiger computer at ESRF
======================================

BIOS
----

Disable CPU power management
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

'lid10eiger1' - Detector PC Type B
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

`High Performance Detector PC Type B <http://wikiserv.esrf.fr/bliss/index.php/High_Performance_Detector_PC#Fourth_Generation_-_E4.2FSupermicro_-_CCTF_Detector_PC_Type-B>`_

In the 'Advanced -> CPU Configuration' menu

+-----------------+---------+
| Hyper-Threading | Disable |
+-----------------+---------+

In the 'Advanced -> CPU Configuration -> CPU Power Management
Configuration' menu

+--------------------+-------------+
| Power Technology   | Disable     |
+--------------------+-------------+
| Energy Performance | Performance |
+--------------------+-------------+

Disable boot on Intel 10 Gigabit Converged Network adapter
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

In the 'Boot -> Set Boot Priority' menu select:

+-----------------+--------------------------+
| 1st Boot Device | Hard Disk:P0: SATADOM-SL |
+-----------------+--------------------------+

In the 'Advanced -> Intel(R) Ethernet Controller 10 Gigabit X540-AT2'
(first) menu note the field:

+-------------+----------+
| PCI Address | 07:00:00 |
+-------------+----------+

It is referred to in other menu as Slot 0700. Then in the 'Boot ->
Network Device BBS Priorities' menu disable all the Intel X520 ports,
and leave only the Intel X540-AT2 ports:

+------------+------------------------+
| 1st Device | IBA XE Slot 0700 v2304 |
+------------+------------------------+
| 2nd Device | Disabled               |
+------------+------------------------+
| 3rd Device | Disabled               |
+------------+------------------------+
| 4th Device | Disabled               |
+------------+------------------------+
| 5th Device | Disabled               |
+------------+------------------------+

Debian 7
--------

*libc*
~~~~~~

The Debian 7 *libc* is 2.13-38+deb7u11, built from *eglibc-2.13*. This
version has a bug in *malloc*, corrupting memory in heavyly
multi-threaded applications. A recompiled version of *libc*, modified
with selected upstream patches, must be installed so the SlsDetector
software can run properly:

::

    # as root
    lid10eiger1:~ # mkdir -p ~/libc && cd ~/libc
    lid10eiger1:~/libc # scp lid01eiger1:/nobackup/lid01eiger12/devel/libc/apt-get-source/eglibc-2.13/modified-02/*.deb .
    ...
    lid10eiger1:~/libc # dpkg --install libc6_2.13-38+deb7u10_amd64.deb \
      libc6-dbg_2.13-38+deb7u10_amd64.deb libc6-dev_2.13-38+deb7u10_amd64.deb \
      libc6-dev-i386_2.13-38+deb7u10_amd64.deb \
      libc6-i386_2.13-38+deb7u10_amd64.deb \
      libc-bin_2.13-38+deb7u10_amd64.deb  libc-dev-bin_2.13-38+deb7u10_amd64.deb \
      locales_2.13-38+deb7u10_all.deb locales-all_2.13-38+deb7u10_amd64.deb \
      multiarch-support_2.13-38+deb7u10_amd64.deb libc6_2.13-38+deb7u10_i386.deb \
      libc-bin_2.13-38+deb7u10_i386.deb libc6-i686_2.13-38+deb7u10_i386.deb
    ...
    lid10eiger1:~/libc # dpkg --list libc\* | grep 2.13-38
    ii  libc-bin         2.13-38+deb7u10  i386   Embedded GNU C Library: Binaries
    ii  libc-dev-bin     2.13-38+deb7u10  amd64  Embedded GNU C Library: Development binaries
    ii  libc6:amd64      2.13-38+deb7u10  amd64  Embedded GNU C Library: Shared libraries
    ii  libc6:i386       2.13-38+deb7u10  i386   Embedded GNU C Library: Shared libraries
    ii  libc6-dbg:amd64  2.13-38+deb7u10  amd64  Embedded GNU C Library: detached debugging symbols
    ii  libc6-dev:amd64  2.13-38+deb7u10  amd64  Embedded GNU C Library: Development Libraries and Header Files
    ii  libc6-dev-i386   2.13-38+deb7u10  amd64  Embedded GNU C Library: 32-bit development libraries for AMD64
    ii  libc6-i386       2.13-38+deb7u10  amd64  Embedded GNU C Library: 32-bit shared libraries for AMD64
    ii  libc6-i686:i386  2.13-38+deb7u10  i386   Embedded GNU C Library: Shared libraries [i686 optimized]

*cpufrequtils*
~~~~~~~~~~~~~~

The previous settings disable the ACPI CPU power management interface,
so the *loadcpufreq* INIT service will not be able to load
*acpi-cpufreq* kernel module. In case this is enabled in BIOS in the
future, force the 'performance' governor in *cpufrequtils* INIT service:

::

    # as root
    lid10eiger1:~ # cat /etc/default/cpufrequtils
    # valid values: userspace conservative powersave ondemand performance
    # get them from cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_available_governors
    GOVERNOR="performance"

*irqbalance*
~~~~~~~~~~~~

The *irqbalance* must be installed in order to distribute hardware IRQs
to different CPU cores:

::

    # as root
    lid10eiger1:~ # apt-get install irqbalance
    ...

Disk configuration
~~~~~~~~~~~~~~~~~~

Find the 4 TByte RAID-0 array in Debian (**/dev/sda**):

::

    # as root
    lid10eiger1:~ # for d in /sys/block/sd?; do \
        echo "$(basename ${d}): model=$(cat ${d}/device/model)" \
             "size=$(python -c "print \"%.1f MB\" % ($(cat ${d}/size) / (2*1024.0**2))")"; \
    done
    sda: model=LSI2208          size=3811.0 MB
    sdb: model=SATADOM-SL 3ME   size=59.6 MB

Install *parted* and use it to create a *GPT* and a RAID partition on
the entire disk:

::

    lid10eiger1:~ # apt-get install parted
    ...
    lid10eiger1:~ # parted /dev/sda
    GNU Parted 2.3
    Using /dev/sda
    Welcome to GNU Parted! Type 'help' to view a list of commands.

    (parted) mklabel gpt

    (parted) unit s

    (parted) print free
    Model: SMC LSI2208 (scsi)
    Model: SMC LSI2208 (scsi)
    Disk /dev/sda: 7992180736s
    Sector size (logical/physical): 512B/512B
    Partition Table: gpt

    Number  Start  End          Size         File system  Name  Flags
            34s    7992180702s  7992180669s  Free Space

    (parted) mkpart logical 2048s 7992178687s

    (parted) set 1 raid on

    (parted) print free
    Model: SMC LSI2208 (scsi)
    Disk /dev/sda: 7992180736s
    Sector size (logical/physical): 512B/512B
    Partition Table: gpt

    Number  Start        End          Size         File system  Name     Flags
            34s          2047s        2014s        Free Space
     1      2048s        7992178687s  7992176640s               logical  raid
            7992178688s  7992180702s  2015s        Free Space

    (parted) quit
    Information: You may need to update /etc/fstab.

.. note:: the partition is aligned to 2048 sectors (1 MByte). The end sector
   is obtained by:

::

    7992180702 - 7992180702 % 2048 - 1 = 7992178687

Create the filesystem and mount it:

::

    lid10eiger1:~ # mkfs.ext4 /dev/sda1
    mke2fs 1.42.5 (29-Jul-2012)
    ...

    lid10eiger1:~ # blkid /dev/sda1
    /dev/sda1: UUID="aff827d8-a744-470d-a753-998919f36d77" TYPE="ext4"

    lid10eiger1:~ # mkdir -p /nobackup/lid10eiger12

    lid10eiger1:~ # cat /etc/fstab
    ...
    UUID="aff827d8-a744-470d-a753-998919f36d77" /nobackup/lid10eiger12        ext4    relatime,nodev,nosuid 0       2

    lid10eiger1:~ # mount /nobackup/lid10eiger12

    lid10eiger1:~ # df -h /nobackup/lid10eiger12
    Filesystem      Size  Used Avail Use% Mounted on
    /dev/sda1       3.7T  196M  3.5T   1% /nobackup/lid10eiger12

    lid10eiger1:~ # mkdir /nobackup/lid10eiger12/data
    lid10eiger1:~ # chmod a+w /nobackup/lid10eiger12/data

Test the effective write speed:

::

    lid10eiger1:~ # mkdir /nobackup/lid10eiger12/data/eiger
    lid10eiger1:~ # chmod a+w /nobackup/lid10eiger12/data/eiger
    lid10eiger1:~ # dd if=/dev/zero bs=8M count=4096 of=/nobackup/lid10eiger12/data/eiger/test.raw
    4096+0 records in
    4096+0 records out
    34359738368 bytes (34 GB) copied, 32.3067 s, 1.1 GB/s

Network performance
~~~~~~~~~~~~~~~~~~~

Add *opid00* user:

::

    # as root
    lid10eiger1:~ # mkuser opid00
    ...

Create *netperf* group and add affected users to it:

::

    lid10eiger1:~ # groupadd netperf

    lid10eiger1:~ # for u in ahoms opid00 opid10; do \
        usermod -a -G netperf ${u}; \
    done

Allow *netperf* users to set real-time (SCHED_RR) scheduling policy with
the highest priority:

::

    lid10eiger1:~ # cat /etc/security/limits.d/net-performance.conf
    @netperf         -       rtprio 99

Compile the *netdev_set_queue_rps_cpus* util, used by the *SlsDetector* plugin
to change the network packet dispatching tasks' CPU affinity, and install it 
in */usr/local/bin*:

::

    lid10eiger1:~ # cat /tmp/netdev_set_queue_rps_cpus.c
    #include <stdio.h>
    #include <stdlib.h>
    #include <string.h>
    #include <errno.h>
    #include <unistd.h>
    #include <sys/types.h>
    #include <sys/stat.h>
    #include <fcntl.h>

    int main(int argc, char *argv[])
    {
            char *dev, *queue, *p, fname[256], buffer[128];
            int fd, len, ret;
            long aff;

            if (argc != 4)
                    exit(1);
            if (!strlen(argv[1]) || !strlen(argv[2]) || !strlen(argv[3]))
                    exit(2);

            dev = argv[1];
            queue = argv[2];

            errno = 0;
            aff = strtol(argv[3], &p, 0);
            if (errno || *p)
                    exit(3);

            len = sizeof(fname);
            ret = snprintf(fname, len, "/sys/class/net/%s/queues/%s/rps_cpus",
                           dev, queue);
            if ((ret < 0) || (ret == len))
                    exit(4);

            len = sizeof(buffer);
            ret = snprintf(buffer, len, "%016lx", aff);
            if ((ret < 0) || (ret == len))
                    exit(5);

            fd = open(fname, O_WRONLY);
            if (fd < 0)
                    exit(6);

            for (p = buffer; *p; p += ret)
                    if ((ret = write(fd, p, strlen(p))) < 0)
                            exit(7);

            if (close(fd) < 0)
                    exit(8);
            return 0;
    }

    lid10eiger1:~ # gcc -Wall -o /tmp/netdev_set_queue_rps_cpus /tmp/netdev_set_queue_rps_cpus.c
    lid10eiger1:~ # cp /tmp/netdev_set_queue_rps_cpus /usr/local/bin

Allow *netperf* users to execute *sudo* in order to change other tasks' CPU affinity
(*taskset* and *netdev_set_queue_rps_cpus*) and to configure the network devices (*ethtool* and
*ifconfig*):

::

    lid10eiger1:~ # cat /etc/sudoers.d/netperf
    %netperf        ALL=(root) NOPASSWD: /usr/bin/taskset, /sbin/ethtool, \
                                         /sbin/ifconfig, \
                                         /usr/local/bin/netdev_set_queue_rps_cpus

Tune the OS network buffer sizes:

::

    lid10eiger1:~ # cat /etc/sysctl.d/net-performance.conf
    # Tune network buffers for UDP RX performance

    # Original values: sysctl -a | grep net
    #...
    #net.core.wmem_max = 131071
    #net.core.rmem_max = 131071
    #net.core.wmem_default = 229376
    #net.core.rmem_default = 229376
    #...
    #net.core.netdev_max_backlog = 1000
    #...
    #net.ipv4.tcp_mem = 1549845 2066462 3099690
    #net.ipv4.tcp_wmem = 4096   16384   4194304
    #net.ipv4.tcp_rmem = 4096   87380   6291456
    #...
    #net.ipv4.udp_mem = 1549845 2066462 3099690
    #net.ipv4.udp_rmem_min = 4096
    #net.ipv4.udp_wmem_min = 4096

    # Max OS socket receive buffer size (in bytes) for all types
    net.core.rmem_max = 134217728

    # Size of per-device buffer (in packets) before Linux kernel dispatching
    net.core.netdev_max_backlog = 262144

Linux *maxcpus* option
~~~~~~~~~~~~~~~~~~~~~~

The *Supermicro* BIOS does not always acknowledge the *Disable Hyper-Threading* option,
so after some reboot conditions the Hyper-Threading is activated and Linux sees twice the
number of CPUs. The only way to control this is to limit the number of CPUs used by 
Linux in the kernel command line.

Edit the */etc/default/grub* file and add *maxcpus=12* to the *GRUB_CMDLINE_LINUX_DEFAULT*
variable and force the update *GRUB* configuration file:

::

    # as root
    lid10eiger1:~ # cat /etc/default/grub
    # If you change this file, run 'update-grub' afterwards to update
    # /boot/grub/grub.cfg.
    # For full documentation of the options in this file, see:
    #   info -f grub -n 'Simple configuration'

    GRUB_DEFAULT=0
    GRUB_TIMEOUT=5
    GRUB_DISTRIBUTOR=`lsb_release -i -s 2> /dev/null || echo Debian`
    GRUB_CMDLINE_LINUX_DEFAULT="ipv6.disable=1 quiet maxcpus=12"
    GRUB_CMDLINE_LINUX=""
    ...    

    lid10eiger1:~ # update-grub
    ...

*cmake*
~~~~~~~

A recent version of *cmake* (> 3.0) is needed to compile Lima. Debian 7 package is 
cmake-2.8.9-1, so it must be compiled from the sources. First un-install the Debian package:

::

    # as root
    lid10eiger1:~ # p=$(dpkg --list cmake\* | grep '^ii' | awk '{print $2}'); \
        [ -n "${p}" ] && dpkg --purge ${p}
    ...

and then copy and compile the sources as *opid00* and install as *root*:

::

    # as opid00
    lid10eiger1:~ % mkdir -p ~/Downloads/cmake && cd ~/Downloads/cmake
    lid10eiger1:~/Downloads/cmake % scp lisgeiger1:Downloads/cmake/cmake-3.8.0.tar.gz .
    ...
    lid10eiger1:~/Downloads/cmake % tar -xzf cmake-3.8.0.tar.gz 
    lid10eiger1:~/Downloads/cmake % cd cmake-3.8.0
    lid10eiger1:~/Downloads/cmake/cmake-3.8.0 % ./bootstrap --parallel=12 --qt-gui
    ...
    lid10eiger1:~/Downloads/cmake/cmake-3.8.0 % make -j12
    ...
    lid10eiger1:~/Downloads/cmake/cmake-3.8.0 % su
    Password: 
    lid10eiger1:Downloads/cmake/cmake-3.8.0 # make install
    ...

Network configuration
---------------------

Intel 10 Gigabit Converged Adapter
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Interface association
^^^^^^^^^^^^^^^^^^^^^

Force *eth2* and *eth3* to be in PCI-Express slot #2 ports and *eth4*
and *eth5* to be in slot #1.

First locate the Intel 10 Gigabit X520 Ethernet adapters (reported as
*Intel Corporation 82599EB 10-Gigabit SFI/SFP+ Network Connection*):

::

    lid10eiger1:~ # lspci | grep Ethernet
    02:00.0 Ethernet controller: Intel Corporation 82599EB 10-Gigabit SFI/SFP+ Network Connection (rev 01)
    02:00.1 Ethernet controller: Intel Corporation 82599EB 10-Gigabit SFI/SFP+ Network Connection (rev 01)
    05:00.0 Ethernet controller: Intel Corporation 82599EB 10-Gigabit SFI/SFP+ Network Connection (rev 01)
    05:00.1 Ethernet controller: Intel Corporation 82599EB 10-Gigabit SFI/SFP+ Network Connection (rev 01)
    07:00.0 Ethernet controller: Intel Corporation Ethernet Controller 10-Gigabit X540-AT2 (rev 01)
    07:00.1 Ethernet controller: Intel Corporation Ethernet Controller 10-Gigabit X540-AT2 (rev 01)

Then check the PCI tree:

::

    lid10eiger1:~ # lspci -t
    -+-[0000:ff]-+-08.0
    ...
     +-[0000:80]-+-01.0-[81]--
    ...
     +-[0000:7f]-+-08.0
    ...
     \-[0000:00]-+-00.0
    ...
                 +-02.0-[02-03]--+-00.0
                 |               \-00.1
                 +-02.2-[04]--
                 +-03.0-[05-06]--+-00.0
                 |               \-00.1
                 +-03.2-[07-08]--+-00.0
                 |               \-00.1
    ...

From the tree we identify the parent root device of each *Intel X520
Ethernet adapter*:

+--------------+--------------+
| Node         | Parent       |
+==============+==============+
| 0000:02:00.x | 0000:00:02.0 |
+--------------+--------------+
| 0000:05:00.x | 0000:00:03.0 |
+--------------+--------------+

Find the PCI-e slot from the parent root port in the CPU:

::

    lid10eiger1:~ # lspci -s 0:02.0 -vvv | grep Slot
        Capabilities: [90] Express (v2) Root Port (Slot+), MSI 00
            LnkSta: Speed 5GT/s, Width x8, TrErr- Train- SlotClk+ DLActive+ BWMgmt+ ABWMgmt-
                Slot #2, PowerLimit 25.000W; Interlock- NoCompl-
    lid10eiger1:~ # lspci -s 0:03.0 -vvv | grep Slot
        Capabilities: [90] Express (v2) Root Port (Slot+), MSI 00
            LnkSta: Speed 5GT/s, Width x8, TrErr- Train- SlotClk+ DLActive+ BWMgmt+ ABWMgmt-
                Slot #1, PowerLimit 25.000W; Interlock- NoCompl-

This means that:

+--------------+------+
| Adapter      | Slot |
+==============+======+
| 0000:02:00.x | 2    |
+--------------+------+
| 0000:05:00.x | 1    |
+--------------+------+

So we must force the following association:

+--------------+----------------+
| PCI-e Device | Network Device |
+==============+================+
| 0000:02:00.0 | eth2           |
+--------------+----------------+
| 0000:02:00.1 | eth3           |
+--------------+----------------+
| 0000:05:00.0 | eth4           |
+--------------+----------------+
| 0000:05:00.1 | eth5           |
+--------------+----------------+

This is obtained by the following *udev* network rules configuration:

::

    lid10eiger1:~ # cat /etc/udev/rules.d/70-persistent-net.rules
    # This file was automatically generated by the /lib/udev/write_net_rules
    # program, run by the persistent-net-generator.rules rules file.
    #
    # You can modify it, as long as you keep each rule on a single
    # line, and change only the value of the NAME= key.

    # PCI device 0x8086:/sys/devices/pci0000:00/0000:00:03.2/0000:07:00.1 (ixgbe)
    SUBSYSTEM=="net", ACTION=="add", DRIVERS=="?*", ATTR{address}=="0c:c4:7a:bc:d0:35", ATTR{dev_id}=="0x0", ATTR{type}=="1", KERNEL=="eth*", NAME="eth1"

    # PCI device 0x8086:/sys/devices/pci0000:00/0000:00:03.2/0000:07:00.0 (ixgbe)
    SUBSYSTEM=="net", ACTION=="add", DRIVERS=="?*", ATTR{address}=="0c:c4:7a:bc:d0:34", ATTR{dev_id}=="0x0", ATTR{type}=="1", KERNEL=="eth*", NAME="eth0"

    # PCI device 0x8086:/sys/devices/pci0000:00/0000:00:02.0/0000:02:00.1 (ixgbe)
    SUBSYSTEM=="net", ACTION=="add", DRIVERS=="?*", ATTR{address}=="90:e2:ba:86:28:65", ATTR{dev_id}=="0x0", ATTR{type}=="1", KERNEL=="eth*", NAME="eth3"

    # PCI device 0x8086:/sys/devices/pci0000:00/0000:00:02.0/0000:02:00.0 (ixgbe)
    SUBSYSTEM=="net", ACTION=="add", DRIVERS=="?*", ATTR{address}=="90:e2:ba:86:28:64", ATTR{dev_id}=="0x0", ATTR{type}=="1", KERNEL=="eth*", NAME="eth2"

    # PCI device 0x8086:/sys/devices/pci0000:00/0000:00:03.0/0000:05:00.1 (ixgbe)
    SUBSYSTEM=="net", ACTION=="add", DRIVERS=="?*", ATTR{address}=="90:e2:ba:86:2e:15", ATTR{dev_id}=="0x0", ATTR{type}=="1", KERNEL=="eth*", NAME="eth5"

    # PCI device 0x8086:/sys/devices/pci0000:00/0000:00:03.0/0000:05:00.0 (ixgbe)
    SUBSYSTEM=="net", ACTION=="add", DRIVERS=="?*", ATTR{address}=="90:e2:ba:86:2e:14", ATTR{dev_id}=="0x0", ATTR{type}=="1", KERNEL=="eth*", NAME="eth4"

Reboot and verify that the association is OK:

::

    lid10eiger1:~ # ls -l /sys/class/net/eth?/device
    lrwxrwxrwx 1 root root 0 Sep  7 21:05 /sys/class/net/eth0/device -> ../../../0000:07:00.0
    lrwxrwxrwx 1 root root 0 Sep  7 21:05 /sys/class/net/eth1/device -> ../../../0000:07:00.1
    lrwxrwxrwx 1 root root 0 Sep  7 21:05 /sys/class/net/eth2/device -> ../../../0000:02:00.0
    lrwxrwxrwx 1 root root 0 Sep  7 21:05 /sys/class/net/eth3/device -> ../../../0000:02:00.1
    lrwxrwxrwx 1 root root 0 Sep  7 21:05 /sys/class/net/eth4/device -> ../../../0000:05:00.0
    lrwxrwxrwx 1 root root 0 Sep  7 21:05 /sys/class/net/eth5/device -> ../../../0000:05:00.1

IP assignments
^^^^^^^^^^^^^^

Assign the following addresses to the Eiger interfaces:

+----------------+---------------------+--------------+
| Network Device | Function            | IP Address   |
+================+=====================+==============+
| eth2           | Top-Half Control    | 192.168.11.1 |
+----------------+---------------------+--------------+
| eth3           | Top-Half Data       | 192.168.12.1 |
+----------------+---------------------+--------------+
| eth4           | Bottom-Half Control | 192.168.13.1 |
+----------------+---------------------+--------------+
| eth5           | Bottom-Half Data    | 192.168.12.1 |
+----------------+---------------------+--------------+

For the 10 Gigabit data interfaces, we force:

-  MTU 9000
-  Rx adaptive interrupt moderation cycle of 100 usecs: */sbin/ethtool
   -C ethX rx-usecs 100*
-  Rx ring buffer size of 4096 entries: */sbin/ethtool -G ethX rx 4096*

The resulting */etc/network/interfaces* file is:

::

    lid10eiger1:~ # cat /etc/network/interfaces
    # This file describes the network interfaces available on your system
    # and how to activate them. For more information, see interfaces(5).

    # The loopback network interface
    auto lo
    iface lo inet loopback

    # The primary network interface
    auto eth0
    allow-hotplug eth0
    iface eth0 inet dhcp
    #   up sleep 5; /sbin/ethtool -s eth0 autoneg off speed 100 duplex full
        up sleep 5; /sbin/ethtool -s eth0 autoneg on speed 1000 duplex full

    # The secondary network interface
    auto eth1
    allow-hotplug eth1
    iface eth1 inet static
            address 192.168.1.1
            netmask 255.255.255.0

    # The 10 Gbps FO network interfaces - Top half
    auto eth2
    allow-hotplug eth2
    iface eth2 inet static
            address 192.168.11.1
            netmask 255.255.255.0
    auto eth3
    allow-hotplug eth3
    iface eth3 inet static
            address 192.168.12.1
            netmask 255.255.255.0
            mtu 9000
            up while /sbin/ethtool eth3 | grep 'Link detected' | grep -q no; do sleep 1; done; /sbin/ethtool -C eth3 rx-usecs 100; /sbin/ethtool -G eth3 rx 4096

    # The 10 Gbps FO network interfaces - Bottom half
    auto eth4
    allow-hotplug eth4
    iface eth4 inet static
            address 192.168.13.1
            netmask 255.255.255.0
    auto eth5
    allow-hotplug eth5
    iface eth5 inet static
            address 192.168.14.1
            netmask 255.255.255.0
            mtu 9000
            up while /sbin/ethtool eth5 | grep 'Link detected' | grep -q no; do sleep 1; done; /sbin/ethtool -C eth5 rx-usecs 100; /sbin/ethtool -G eth5 rx 4096

PSI/Eiger modules
~~~~~~~~~~~~~~~~~

Define the PSI/Eiger module IPs (data interfaces are not actually
needed):

::

    lid10eiger1:~ # cat /etc/hosts
    #============= OS ====================
    127.0.0.1   localhost
    127.0.1.1   lid10eiger1.esrf.fr lid10eiger1

    #============= Eiger ====================
    # Direct Connection - Top half
    192.168.11.10   beb021.esrf.fr  beb021
    #192.168.12.20  beb02110ge1.esrf.fr     beb02110ge1

    # Direct Connection - Bottom half
    192.168.13.11   beb020.esrf.fr  beb020
    #192.168.14.21  beb02010ge1.esrf.fr     beb02010ge1

    #============= OS ====================
    # The following lines are desirable for IPv6 capable hosts
    ::1     localhost ip6-localhost ip6-loopback
    ff02::1 ip6-allnodes
    ff02::2 ip6-allrouters

Modify *nsswitch.conf* to first look at */etc/hosts* when resolving
names:

::

    lid10eiger1:~ # cat /etc/nsswitch.conf
    # /etc/nsswitch.conf
    ...
    hosts:          files dns [NOTFOUND=return] mdns4_minimal mdns4
    ...

DHCP configuration
~~~~~~~~~~~~~~~~~~

Install *DHCP server* software:

::

    # as root
    lid10eiger1:~ # apt-get install isc-dhcp-server
    ...
    [FAIL] Starting ISC DHCP server: dhcpd[....] check syslog for diagnostics. ... failed!
     failed!
    invoke-rc.d: initscript isc-dhcp-server, action "start" failed.
    ldegjfrau1:~ # apt-get install isc-dhcp-server
    ...

Define the dynamic addresses and Eiger MAC/name relations:

::

    lid10eiger1:~ # cat /etc/dhcp/dhcpd.conf
    ...
    option domain-name "esrf.fr";
    option domain-name-servers dns1.esrf.fr, dns2.esrf.fr;
    ...
    # This is a very basic subnet declaration.

    subnet 192.168.1.0 netmask 255.255.255.0 {
      range 192.168.1.128 192.168.1.191;
    }

    subnet 192.168.11.0 netmask 255.255.255.0 {
      range 192.168.11.128 192.168.11.191;
    }

    subnet 192.168.12.0 netmask 255.255.255.0 {
      range 192.168.12.128 192.168.12.191;
    }

    subnet 192.168.13.0 netmask 255.255.255.0 {
      range 192.168.13.128 192.168.13.191;
    }

    subnet 192.168.14.0 netmask 255.255.255.0 {
      range 192.168.14.128 192.168.14.191;
    }

    # PSI Eiger 500k detectors

    host beb021 {
      hardware ethernet 00:50:c2:46:d9:2a;
      fixed-address beb021.esrf.fr;
    }

    host beb020 {
      hardware ethernet 00:50:c2:46:d9:28;
      fixed-address beb020.esrf.fr;
    }
    ...

Specify the interfaces *DHCP server* will listen on:

::

    lid10eiger1:~ # cat /etc/default/isc-dhcp-server
    ...
    INTERFACES="eth2 eth4"

Reboot the computer for the changes to be applied. Verify that the
*dhcp* server is running on the given interfaces:

::

    lid10eiger1:~ # ps -ef | grep dhcpd | grep -v grep
    root      3923     1  0 21:48 ?        00:00:00 /usr/sbin/dhcpd -q -cf /etc/dhcp/dhcpd.conf -pf /var/run/dhcpd.pid eth2 eth4

Restart the detector, wait for 20 sec and check that the links are OK:

::

    lid10eiger1:~ # for i in $(seq 2 5); do n="eth${i}"; ifconfig ${n} | grep UP; done
              UP BROADCAST RUNNING MULTICAST  MTU:1500  Metric:1
              UP BROADCAST RUNNING MULTICAST  MTU:9000  Metric:1
              UP BROADCAST RUNNING MULTICAST  MTU:1500  Metric:1
              UP BROADCAST RUNNING MULTICAST  MTU:9000  Metric:1

    lid10eiger1:~ # ping -c 1 beb021; ping -c 1 beb020
    PING beb021.esrf.fr (192.168.11.10) 56(84) bytes of data.
    64 bytes from beb021.esrf.fr (192.168.11.10): icmp_req=1 ttl=64 time=0.357 ms

    --- beb021.esrf.fr ping statistics ---
    1 packets transmitted, 1 received, 0% packet loss, time 0ms
    rtt min/avg/max/mdev = 0.357/0.357/0.357/0.000 ms
    PING beb020.esrf.fr (192.168.13.11) 56(84) bytes of data.
    64 bytes from beb020.esrf.fr (192.168.13.11): icmp_req=1 ttl=64 time=0.399 ms

    --- beb020.esrf.fr ping statistics ---
    1 packets transmitted, 1 received, 0% packet loss, time 0ms
    rtt min/avg/max/mdev = 0.399/0.399/0.399/0.000 ms


SlsDetectors Software
---------------------

ROOT installation
~~~~~~~~~~~~~~~~~

Install development packages necessary as root:

::

    lid10eiger1:~ # apt-get install libxpm-dev libldap2-dev libmysqlclient-dev \
                                   libavahi-client-dev libavahi-compat-libdnssd-dev \
                                   libfftw3-dev graphviz-dev libxml2-dev libcfitsio3-dev
    ...

Unpack the ROOT sources on a user's directory (*~opid00*), build it (12
parallel jobs: one per core), and install on /opt/root:

::

    lid10eiger1:~/Downloads % mkdir -p ~/Downloads
    lid10eiger1:~/Downloads % cd ~/Downloads
    lid10eiger1:~/Downloads % scp lisgeiger1:Downloads/root_v5.34.34.source.tar.gz .
    root_v5.34.34.source.tar.gz                   100%   72MB  71.7MB/s   00:01
    lid10eiger1:~/Downloads % tar -xzf root_v5.34.34.source.tar.gz
    lid10eiger1:~/Downloads % mkdir rootbuild
    lid10eiger1:~/Downloads % cd rootbuild
    lid10eiger1:~/Downloads/rootbuild % cmake ~/Downloads/root
    ...
    lid10eiger1:~/Downloads/rootbuild % cmake --build . -- -j12
    ...
    lid10eiger1:~/Downloads/rootbuild % su
    Password:
    lid10eiger1:/users/opid00/Downloads/rootbuild # cmake -DCMAKE_INSTALL_PREFIX=/opt/root -P cmake_install.cmake
    ...

Include ROOT initialisation script in the global /etc/profile.d chain:

::

    lid10eiger1:~ # echo ". /opt/root/bin/thisroot.sh" > /etc/profile.d/root.sh

Qt4 environment
~~~~~~~~~~~~~~~

Do the same for Qt4:

::

    lid10eiger1:~ # echo "QTDIR=/usr/share/qt4
    export QTDIR" > /etc/profile.d/qt4.sh

Qwt development package installation
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Qwt development package is needed by some applications in the
*SlsDetectorsSoftware*:

::

    lid10eiger1:~ # apt-get install libqwt-dev
    Reading package lists... Done
    Building dependency tree
    Reading state information... Done
    The following packages will be REMOVED:
      libqwt5-qt4-dev
    The following NEW packages will be installed:
      libqwt-dev
    0 upgraded, 1 newly installed, 1 to remove and 210 not upgraded.
    Need to get 111 kB of archives.
    After this operation, 35.8 kB of additional disk space will be used.
    Do you want to continue [Y/n]? y
    ...

    lid10eiger1:~ # dpkg --list libqwt\* | grep '^ii'
    ii  libqwt-dev          6.0.0-1.2       amd64  Qt widgets library for technical applications (development)
    ii  libqwt5-doc         5.2.2-3         all    Qt widgets library for technical applications (documentation)
    ii  libqwt5-qt4         5.2.2-3         amd64  Qt4 widgets library for technical applications (runtime)
    ii  libqwt6             6.0.0-1.2       amd64  Qt widgets library for technical applications (runtime)
    ii  libqwtplot3d-qt4-0  0.2.7+svn191-7  amd64  3D plotting library based on Qt4/OpenGL (runtime)

BLISS software installation
~~~~~~~~~~~~~~~~~~~~~~~~~~~

Install and execute the `GitLab
Admin/bliss_python_install <https://gitlab.esrf.fr/Admin/bliss_python_install>`__
script (*blissadm*).

First, install the Gitlab project deployment keys:

::

    # as blissadm
    lid10eiger1:~ % bliss_rpm dev-gitlab
    Installing package dev-gitlab-src-1.3-1.src.rpm
    ...

Then clone the project:

::

    # as blissadm
    lid10eiger1:~ % mkdir -p ~/src/install
    lid10eiger1:~ % cd ~/src/install
    lid10eiger1:~/src/install % git clone git@blissinstaller.gitlab.esrf.fr:Admin/bliss_python_install.git

And execute the script as *root*:

::

    # as blissadm
    lid10eiger1:~/src/install % ssh root@localhost
    The authenticity of host 'localhost (127.0.0.1)' can't be established.
    ECDSA key fingerprint is d7:da:38:9c:c4:20:8f:87:66:73:5a:85:62:44:01:f8.
    Are you sure you want to continue connecting (yes/no)? yes
    ...

    lid10eiger1:~ # /users/blissadm/src/install/bliss_python_install/install_python_debian
    Logging to file: /users/blissadm/admin/log/install_python_debian.log
    Running on debian7 lid10eiger1 [Fri Sep  8 16:19:52 CEST 2017]
    6576d6b78ac7469a254d310e6136931c  install_python_debian
    98c591cbf712ac69e6963058c2c9474c  install_python_debian.blissadm
    ...

Install *PyTango*, needed by *Lima*:

::

    # as blissadm
    lid10eiger1:~/src/install % bliss_rpm six
    Installing package six-src-1.0-1.src.rpm
    ...

    lid10eiger1:~/src/install % bliss_rpm tango_lib
    Installing package tango_lib-debian7-9.25-1.src.rpm
    ...

    lid10eiger1:~/src/install % bliss_rpm PyTango
    Installing package PyTango-debian7-9.5-1.src.rpm
    ...

Install the Python modules needed for building the HTML documentation
with Doxygen, Sphinx and Read-the-Docs:

::

    # as blissadm
    lid10eiger1:~ . blissrc
    (bliss) lid10eiger1:~ % pip install sphinx_rtd_theme breathe
    ...

Eiger calibration development: *Seaborn* and *Spyder*
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The *seaborn* Python module and the *spyder* IDE for are used by Eiger
calibration development (Erik Frojdh). First *seaborn*:

::

    # as blissadm
    lid10eiger1:~ % (. blissrc && pip install seaborn)
    ...

Then install *spyder*:

::

    # as root
    lid10eiger1:/users/blissadm # apt-get install spyder
    ...

Configure *spyder* to use the BLISS python:

::

    # as opid00
    lid10eiger1:~ % (. blissrc && spyder)

and go to 'Tools -> Preferences -> Console -> Advanced Settings ->
Python executable' and set:

+----------------------------------------------+--------------------------------------------------+
| Path to Python interpreter executable binary | /users/blissadm/lib/virtualenvs/bliss/bin/python |
+----------------------------------------------+--------------------------------------------------+

Detector software and development account: *opid00*
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Define the Eiger software home
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Add the *eiger.sh* entry in the system-wide Bash login setup scripts:

::

    # as root
    lid10eiger1:~ # cat /etc/profile.d/eiger.sh
    EIGER_HOME=~opid00
    export EIGER_HOME

Eiger environment setup
^^^^^^^^^^^^^^^^^^^^^^^

Create *eiger_setup.sh*, oriented to prepare the Eiger environment. In
the beginning it just contains the BLISS environment:

::

    # as opid00
    lid10eiger1:~ % cat eiger_setup.sh
    # Setup the Eiger data acquisition environment

    # include the BLISS environment
    . blissrc

and include it in the *.bash_profile* so it is executed at every login
shell:

::

    lid10eiger1:~ % tail -n 3 .bash_profile

    # include the PSI/Eiger environment
    . ${EIGER_HOME}/eiger_setup.sh

*git-sig* Bash helper
^^^^^^^^^^^^^^^^^^^^^

Add the *git-sig* Bash helper for authoring future commits:

::

    lid10eiger1:~ % tail -n 22 .bashrc

    # Signature: from dev-gitlab dot_bashrc

    read_esrf_3612()
    {
        echo $1 | nc 160.103.180.14 3612
    }

    git-sig()
    {
        read_esrf=$(read_esrf_3612 $1)
        if [ "${read_esrf}" != "NO USER" ]; then
            GIT_AUTHOR_NAME=$(echo "${read_esrf}" | cut -d' ' -f4- | sed 's/"//g')
            GIT_AUTHOR_EMAIL="$1@esrf.fr"
        else
            GIT_AUTHOR_NAME="$1@esrf.fr"
            GIT_AUTHOR_EMAIL="$1@esrf.fr"
        fi

        export GIT_AUTHOR_NAME GIT_AUTHOR_EMAIL
        echo "Now git will use \"$GIT_AUTHOR_NAME\" to commits until SHELL ends"
    }

Logout from *opid00* and re-login so changes are taken into account for
next steps.

Eiger-500k configuration
~~~~~~~~~~~~~~~~~~~~~~~~

Copy the Eiger-500k detector configuration file and adapt to the new
computer directories:

::

    (bliss) lid10eiger1:~ % EIGER_DIR=${EIGER_HOME}/eiger/eiger_v3.1.1
    (bliss) lid10eiger1:~ % EIGER_CONFIG=${EIGER_DIR}/config/beb-021-020-direct-FO-10g.config
    (bliss) lid10eiger1:~ % mkdir -p $(dirname ${EIGER_CONFIG})
    (bliss) lid10eiger1:~ % scp lisgeiger1:${EIGER_CONFIG} $(dirname ${EIGER_CONFIG})
    beb-021-020-direct-FO-10g.config         100%  781     0.8KB/s   00:00
    (bliss) lid10eiger1:~ % sed -i 's:lisgeiger1:lid10eiger1:g' ${EIGER_CONFIG}

The resulting configuration file:

::

    (bliss) lid10eiger1:~ % cat ${EIGER_CONFIG}
    detsizechan 1024 512

    #type Eiger+
    #top+bottom+
    hostname beb021+beb020+

    rx_hostname lid10eiger1

    #port 1952
    #stopport 1953

    #top
    0:rx_tcpport 1961
    0:rx_udpport 50010
    0:rx_udpport2 50011
    0:rx_udpip 192.168.12.1
    0:detectorip 192.168.12.20
    0:detectormac 00:50:c2:46:d9:2b
    0:flippeddatax 0

    #bottom
    1:rx_tcpport 1962
    1:rx_udpport 50012
    1:rx_udpport2 50013
    1:rx_udpip 192.168.14.1
    1:detectorip 192.168.14.21
    1:detectormac 00:50:c2:46:d9:29
    1:flippeddatax 1

    settingsdir /users/opid00/eiger/eiger_v3.1.1/settingsdir/eiger
    lock 0
    #caldir /users/opid00/eiger/eiger_v3.1.1/settingsdir/eiger
    outdir /nobackup/lid10eiger12/data/eiger

    tengiga 1
    threaded 1
    flags parallel
    iodelay 651

    trimen 7 3000 3700 4500 5400 6400 8000 9900

    index 250

Copy the detector calibration data:

::

    (bliss) lid10eiger1:~ % SLS_DETECTOR_SETTINGS=$(grep ^settings ${EIGER_CONFIG} | awk '{print $2}')/standard
    (bliss) lid10eiger1:~ % mkdir -p $(dirname ${SLS_DETECTOR_SETTINGS})
    (bliss) lid10eiger1:~ % scp -r lisgeiger1:${SLS_DETECTOR_SETTINGS} $(dirname ${SLS_DETECTOR_SETTINGS})
    ...

Add the configuration file to *eiger_setup.sh* and decode the
*EIGER_MODULES*, together with the calibration directory:

::

    (bliss) lid10eiger1:~ % tail -n 8 eiger_setup.sh

    EIGER_DIR=${EIGER_HOME}/eiger/eiger_v3.1.1
    EIGER_CONFIG=${EIGER_DIR}/config/beb-021-020-direct-FO-10g.config
    EIGER_MODULES=$(grep "^hostname" ${EIGER_CONFIG} | cut -d" " -f2 | tr '+' ' ')
    export EIGER_DIR EIGER_CONFIG EIGER_MODULES

    SLS_DETECTOR_SETTINGS=$(grep ^settings ${EIGER_CONFIG} | awk '{print $2}')/standard
    export SLS_DETECTOR_SETTINGS

Logout from *opid00* and login again in order to apply the previous
changes.


ESRF package for SlsDetectors
-----------------------------

Install the [GitLab Hardware/sls_detectors
project\|\ https://gitlab.esrf.fr/Hardware/sls_detectors]:

::

    (bliss) lid10eiger1:~ % mkdir -p ~/esrf && cd ~/esrf
    (bliss) lid10eiger1:~/esrf % git clone -o gitlab git://gitlab.esrf.fr/Hardware/sls_detectors.git
    Cloning into 'sls_detectors'...
    ...

Add the *ESRF scripts* to *eiger_setup.sh*:

::

    (bliss) lid10eiger1:~ % tail -n 5 eiger_setup.sh

    SLS_DETECTORS=${EIGER_HOME}/esrf/sls_detectors
    export SLS_DETECTORS
    PATH=${SLS_DETECTORS}/eiger/scripts:${PATH}
    export PATH

Logout and re-login as *opid00* to have the previous environment set.

Lima installation in detector software account
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

First install *flex*, which might needed to compile some *Lima* subsystems:

::

    # as root
    lid10eiger1:~ # apt-get install flex
    ...

*Lima* is referenced as a submodule by the *sls_detectors* project installed before:

::

    # as opid00
    (bliss) lid10eiger1:~ % cd ${SLS_DETECTORS}
    (bliss) lid10eiger1:~/esrf/sls_detectors % git submodule init Lima
    Submodule 'Lima' (git://gitlab.esrf.fr/limagroup/lima.git) registered for path 'Lima'
    (bliss) lid10eiger1:~/esrf/sls_detectors % git submodule update
    ...
    (bliss) lid10eiger1:~/esrf/sls_detectors % LIMA_DIR=${SLS_DETECTORS}/Lima
    (bliss) lid10eiger1:~/esrf/sls_detectors % cd ${LIMA_DIR}
    (bliss) lid10eiger1:~/esrf/sls_detectors/Lima % submod="third-party/Processlib
        third-party/Sps
        third-party/gldisplay
        camera/slsdetector
        applications/spec
        applications/tango/python"
    (bliss) lid10eiger1:~/esrf/sls_detectors/Lima % github_submod_names="Sps"
    (bliss) lid10eiger1:~/esrf/sls_detectors/Lima % github_submod=$(for s in ${submod}; do \
        for m in ${github_submod_names}; do \
            echo ${s} | grep ${m}; \
        done; \
    done)
    (bliss) lid10eiger1:~/esrf/sls_detectors/Lima % re_pat="(${github_submod_names// /|})"
    (bliss) lid10eiger1:~/esrf/sls_detectors/Lima % gitlab_submod=$(echo "${submod}" | grep -Ev ${re_pat})
    (bliss) lid10eiger1:~/esrf/sls_detectors/Lima % git submodule init ${submod}
    ...
    (bliss) lid10eiger1:~/esrf/sls_detectors/Lima % git submodule update
    ...
    (bliss) lid10eiger1:~/esrf/sls_detectors/Lima % for s in ${github_submod}; do \
        (cd ${s} && \
             git remote rename origin github.bliss); \
    done
    (bliss) lid10eiger1:~/esrf/sls_detectors/Lima % for s in ${gitlab_submod}; do \
        (cd ${s} && \
             git remote rename origin gitlab && \
             git remote add github.bliss \
                 $(git config remote.gitlab.url | sed "s%git://gitlab.esrf.fr/limagroup%git://github.com/esrf-bliss%")); \
    done
    (bliss) lid10eiger1:~/esrf/sls_detectors/Lima % git remote rename origin gitlab
    (bliss) lid10eiger1:~/esrf/sls_detectors/Lima % git remote add github.bliss git://github.com/esrf-bliss/Lima.git
    (bliss) lid10eiger1:~/esrf/sls_detectors/Lima % git submodule foreach git fetch --all
    ...
    (bliss) lid10eiger1:~/esrf/sls_detectors/Lima % git fetch --all
    ...

Eiger software: slsDetectorPackage
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The *slsDetectorPackage* is in turn a submodule of the *Lima/camera/slsdetector*
plugin:

::

    # as opid00
    (bliss) lid10eiger1:~ % cd ${LIMA_DIR}/camera/slsdetector
    (bliss) lid10eiger1:Lima/camera/slsdetector % git submodule init
    Submodule 'slsDetectorPackage' (git://github.com/esrf-bliss/slsDetectorPackage.git) registered for path 'slsDetectorPackage'
    (bliss) lid10eiger1:Lima/camera/slsdetector % git submodule update
    ...
    (bliss) lid10eiger1:Lima/camera/slsdetector % cd slsDetectorPackage
    (bliss) lid10eiger1:camera/slsdetector/slsDetectorPackage % git remote rename origin github.bliss
    (bliss) lid10eiger1:camera/slsdetector/slsDetectorPackage % git remote add github.slsdetectorgroup \
        git://github.com/slsdetectorgroup/slsDetectorPackage.git
    (bliss) lid10eiger1:camera/slsdetector/slsDetectorPackage % git fetch --all
    ...

*Lima* compilation
~~~~~~~~~~~~~~~~~~

Compile *Lima*, including *slsDetectorPackage* using *CMake*:

::

    (bliss) lid10eiger1:~ % cd ${LIMA_DIR}
    (bliss) lid10eiger1:~/esrf/sls_detectors/Lima % cp scripts/config.txt_default scripts/config.txt
    (bliss) lid10eiger1:~/esrf/sls_detectors/Lima % echo "CMAKE_BUILD_TYPE=RelWithDebInfo" >> scripts/config.txt
    (bliss) lid10eiger1:~/esrf/sls_detectors/Lima % mkdir -p ${LIMA_DIR}/install/python
    (bliss) lid10eiger1:~/esrf/sls_detectors/Lima % ./install.sh \
        --install-prefix=${LIMA_DIR}/install \
        --install-python-prefix=${LIMA_DIR}/install/python \
        slsdetector sps-image gldisplay edfgz python pytango-server tests
    ...

Build the documentation:

::

    (bliss) lid10eiger1:~/esrf/sls_detectors/Lima % make -C docs html
    ...

Add *Lima* to the *PATH*, *LD_LIBRARY_PATH* and *PYTHONPATH* environment variables in
*eiger_setup.sh*:

::

    (bliss) lid10eiger1:~ % tail -n 6 eiger_setup.sh

    LIMA_DIR=${SLS_DETECTORS}/Lima
    PATH=${LIMA_DIR}/install/bin:${PATH}
    LD_LIBRARY_PATH=${LIMA_DIR}/install/lib:${LD_LIBRARY_PATH}
    PYTHONPATH=${LIMA_DIR}/install/python:${PYTHONPATH}
    export LIMA_DIR PATH LD_LIBRARY_PATH PYTHONPATH

*eigerDetectorServer* and detector firmwares
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

If necessary, the *eigerDetectorServer* corresponding to the installed *slsDetectorPackage* version
must be copied into the modules embedded Linux. Please refer to :doc:`installation_eiger_server_and_fw`


Test the *slsDetectorSoftware* and *Lima*
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Logout and re-login as *opid00*, so the previous changes can be tested. 
First, test the 'slsDetectorGui':

::

    (bliss) lid10eiger1:~ % start_eiger_gui
    ...

One *xterm* per Receiver (half-module) window should appear. Accept the
message box acknowleging the detector configuration parameters, and the
GUI will open. Wait for few seconds until a message box pops out asking
to activate the high voltage; answer *No*. In the GUI, disable the *File
Name* check box and press *Start* for a single acquisition. A frame
should be taken.

Finally, test the *Lima* plugin without and with *CtControl* instantiation:

::

    (bliss) lid10eiger1:~ % cd ${LIMA_DIR}
    (bliss) lid10eiger1:~/esrf/sls_detectors/Lima % rm -f /tmp/eiger.edf && \
                                                    build/camera/slsdetector/test/test_slsdetector -c ${EIGER_CONFIG}
    ...
    (bliss) lid10eiger1:~/esrf/sls_detectors/Lima % mkdir -p /nobackup/lid10eiger12/data/eiger/lima
    (bliss) lid10eiger1:~/esrf/sls_detectors/Lima % ln -s /nobackup/lid10eiger12/data/eiger/lima data
    (bliss) lid10eiger1:~/esrf/sls_detectors/Lima % rm -f data/img*.edf && \
                                                    python camera/slsdetector/test/test_slsdetector_control.py -c ${EIGER_CONFIG}
    ...

Clean the shared memory segments used by the SlsDetector library, so
thay can be re-created by *opid10*:

::

    # as opid00
    (bliss) lid10eiger1:~ % for m in $(ipcs -m | grep '^0x000016' | awk '{print $2}'); do \
                                ipcrm -m ${m}; \
                            done


Setup *opid10* account
~~~~~~~~~~~~~~~~~~~~~~

Include the Eiger environment at login:

::

    # as opid10
    lid10eiger1:~ % tail -n 3 .bash_profile

    # include the PSI/Eiger environment
    . ${EIGER_HOME}/eiger_setup.sh


Install Lima Python Tango software in *blissadm*
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Install the following packages with *Blissinstaller*:

-  Control/Driver/bliss_drivers: needed for *blisspipe*
-  Control/Taco/bliss_dserver
-  Tango/Server/LimaCCDs-Simulator:

   -  Python/Modules/PyLimaCore
   -  Python/MOdules/PyLimaSimulator
   -  Tango/Server/LimaCCDs-common

-  Control/Tango/Applications/Jive

Configure the driver infrastructure by calling *bliss_drivers config*:

::

    # as blissadm
    lid10eiger1:~ % bliss_drivers config
    Root Password:
    Copying /users/blissadm/applications/bliss_drivers/Esrfmap/60-esrf.rules to /etc/udev/rules.d/60-esrf.rules
    Starting blisspipe ...

Apply all the suggestions and save before quiting.

Include the *Lima* libraries and modules in the *BLISS_LIB_PATH* and *PYTHONPATH*, respectively:

::

    # as blissadm
    lid10eiger1:~ % . ${EIGER_HOME}/eiger_setup.sh
    (bliss) lid10eiger1:~ % blissrc -a BLISS_LIB_PATH ${LIMA_DIR}/install/lib
    (bliss) lid10eiger1:~ % blissrc -a PYTHONPATH ${LIMA_DIR}/install/python

Rename the Lima installed directories so they are no longer visible, and create the necessary
symbolic links:

::

    # as blissadm
    (bliss) lid10eiger1:~ % cd ~/python/bliss_modules
    (bliss) lid10eiger1:~/python/bliss_modules % mv Lima Lima-pack
    (bliss) lid10eiger1:~/python/bliss_modules % cd ~/applications
    (bliss) lid10eiger1:~/applications % mv LimaCCDs LimaCCDs-pack
    (bliss) lid10eiger1:~/python/bliss_modules % cd ~/server/src
    (bliss) lid10eiger1:~/server/src % mv LimaCCDs LimaCCDs-pack
    (bliss) lid10eiger1:~/server/src % ln -s ${LIMA_DIR}/install/bin/LimaCCDs


Lima Python Tango server configuration in *blissadm*
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Use *jive* server wizard to add the Lima Python Tango device server to
the Tango database:

::

    (bliss) lid10eiger1:~ % jive > /dev/null 2>&1 &

Define the server *LimaCCDs/eiger500k* and include it in the *dserver*
local database:

::

    # as blissadm
    lid10eiger1:~ % cat ~/local/daemon/config/device_servers
    [LimaCCDs]
    *eiger500k

::

    # as opid10
    (bliss) lid10eiger1:~ % bliss_dserver -fg start LimaCCDs
    Starting: LimaCCDs/eiger500k

Add LimaCCDs and SlsDetector class devices.

+----------------------------------------------------------+-------------------------------------------+
| LimaCCDs/eiger500k/DEVICE/LimaCCDs                       | id10/limaccds/eiger500k                   |
+----------------------------------------------------------+-------------------------------------------+
| id10/limaccds/eiger500k->LimaCameraType                  | SlsDetector                               |
+----------------------------------------------------------+-------------------------------------------+
| id10/limaccds/eiger500k->NbProcessingThread              | 11                                        |
+----------------------------------------------------------+-------------------------------------------+
| LimaCCDs/eiger500k/DEVICE/SlsDetector                    | id10/slsdetector/eiger500k                |
+----------------------------------------------------------+-------------------------------------------+
| id10/slsdetector/eiger500k->config_fname                 | /users/opid00/eiger/eiger_v3.1.1/config/  |
|                                                          | beb-021-020-direct-FO-10g.config          |
+----------------------------------------------------------+-------------------------------------------+
| id10/slsdetector/eiger500k->netdev_groups                | | eth0,eth1,eth2,eth4,eth6,eth7,eth8,eth9 |
|                                                          | | eth3,eth5                               |
+----------------------------------------------------------+-------------------------------------------+
| id10/slsdetector/eiger500k->pixel_depth_cpu_affinity_map | | 4,0xf00,0xfc,0x2,0x1,0x1,0x2            |
|                                                          | | 8,0xf00,0xfc,0x2,0x1,0x1,0x2            |
|                                                          | | 16,0xfff,0xfff,0xfff,0xfff,0xfff,0xfff  |
|                                                          | | 32,0xfff,0xfff,0xfff,0xfff,0xfff,0xfff  |
+----------------------------------------------------------+-------------------------------------------+

.. note:: in order to perform high frame rate acquisitions, the CPU affinity must be fixed for 
   the following tasks:

   * Receiver listeners
   * Receiver writers
   * Lima processing threads
   * OS processes
   * Net-dev group #1 packet dispatching
   * Net-dev group #2 packet dispatching
   * ...

   The previous example is based on a dual 6-core CPUs backend (12 cores). After the data acquisition finishes
   the Lima processing threads will run also on the CPUs assigned to listeners and writers (0xffe), that is
   11 cores in total, which is used for setting the NbProcessingThreads. Please note that there are two network
   groups and four pixel_depth->cpu_affinity settings (4-, 8-, 16- and 32-bit), each one represented by a line
   in a multi-line string array.
  
Finally, configure *opid10* as the default *DSERVER_USER*, which is used
by the *dserver_daemon*
   
::

    # as blissadm
    lid10eiger1:~ % grep DSERVER_USER local/BLISS_ENV_VAR || \
                         echo 'DSERVER_USER=opid10 export DSERVER_USER' >> local/BLISS_ENV_VAR


and restart the *blcontrol* subsystem:

::

    # as root
    lid10eiger1:~ # service blcontrol stop
     BL control ...
    ...
    lid10eiger1:~ # service blcontrol start
     BL control ...
    ...

.. note:: the latest version of the *daemon_adm* package allows the
   propagation of the real-time priority capabilities configured as
   resource limits, so **it is safe** to start the server through the
   *dserver* remote utility. **If the command *bliss_dserver start* is
   used, start the server in background and avoid *-fg* option**, so the
   *LimaCCDs* process is decoupled from the terminal, reducing the
   risks of CPU blocking.
   
SPEC
----

Install SPEC and CCD/Lima macros
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Install the following packages with *Blissinstaller*:

-  Control/Spec/Core/SPEC
-  Control/Spec/Macros/ccd
-  Control/Spec/Macros/ccdbpm
-  Control/Spec/Macros/lima.mac
-  Applications/Analysis/Oxidis

Add a symbolic link to the Lima SlsDetector macros in the development
version compiled on *opid00*:

::

    # as blissadm
    lid10eiger1:~ % LIMA_DIR=${EIGER_HOME}/esrf/sls_detectors/Lima
    lid10eiger1:~ % cd ~/spec/macros/lima
    lid10eiger1:~/spec/macros/lima % ln -s ${LIMA_DIR}/applications/spec/limaslsdetector.mac

SPEC configuration
~~~~~~~~~~~~~~~~~~

Include the *ccd & Lima* macros by default in SPEC:

::

    # as blissadm
    lid10eiger1:~ % cat ~/local/spec/macros/ID10setup.mac
    ...
    need ccd
    need lima/limacore
    need lima/limaacq
    need lima/limaroi
    need lima/limatools
    need lima/limasimulator
    need lima/limafrelon
    need lima/limaslsdetector
    ...

Create the *eiger* SPEC session:

::

    # as blissadm
    lid10eiger1:~/spec/macros/lima % spec_version add eiger
    ...

SPEC config file
^^^^^^^^^^^^^^^^

Configure the *LimaCCDs/eiger500k* Taco interface server.

::

    (bliss) lid10eiger1:~ % cat ~blissadm/local/spec/spec.d/eiger/config
    # ID @(#)getinfo.c  6.5  03/14/15 CSS
    # Device nodes
    PSE_MAC_MOT  = slsdetmot 32 eiger500k
    SW_SFTWARE   = 1 POLL
    VM_CCD_PC    = tango:id10/limatacoccds/eiger500k 2 TCP  @img_0
    # CAMAC Slot Assignments
    #  CA_name_unit = slot [crate_number]
    # Motor    cntrl steps sign slew base backl accel nada  flags   mne  name
    MOT000 = MAC_MOT:0/0   2000  1  2000  200   50  125    0 0x003    en_th  en_th
    MOTPAR:read_mode = 7
    MOTPAR:name = threshold_energy
    # Counter   ctrl unit chan scale flags    mne  name
    CNT000 =  SFTWARE  0  0 1000000 0x001      sec  Seconds
    CNT001 =     NONE  0  0      1 0x000   imgall  imgall

SPEC setup file
^^^^^^^^^^^^^^^

Configure the *LimaCCDs/eiger500k* server control and specific
interfaces.

::

    (bliss) lid10eiger1:~ % cat ~blissadm/local/spec/spec.d/eiger/setup
    #
    # Add or modify setup lines.
    # Comment out the lines you want to cancel temporarily.
    #

    def lima_ccd_resetup_all '{
        local ccd_u ccd_dev

        _ccd_globals
        limasetup

        for (ccd_u = 0; ccd_u < CCDS; ccd_u++) {
            ccd_dev = image_par(ccd_u, "device_id")
            if (!image_par(ccd_u,"responsive") || (ccd_dev == "?"))
                continue

            ccdresetup ccd_u

            ##########################
            #     eiger500k
            #########################
            if (index(ccd_dev, "eiger500k") > 0) {
                limaccdsetup eiger500k ccd_u id10/limaccds/eiger500k
                taco_io(ccd_dev, "timeout", 30)
                tango_io("id10/slsdetector/eiger500k", "timeout", 30)
            }
        }
    }'

    lima_ccd_resetup_all

.. note:: the 30 seconds timeout is necessary for large memory 
   allocations (long sequences)
