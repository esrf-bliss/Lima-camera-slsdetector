.. _camera-slsdetector:

SlsDetector camera
--------------

.. image:: PSI-Eiger-500k.jpg

Introduction
````````````
The PSI/SLS Detector Group has developed a family of X-ray detectors: Mythen, Pilatus, Gotthard,
Eiger, Moench, Jungrau, among others. Most of them are controlled through Ethernet interfaces,
with optional dedicated data link(s). A common protocol has been developed to control these detectors,
based on the *slsDetector* class. A separate software entity receives and dispatch the data: *slsReceiver*.
The SlsDetector LIMA plugin instantiates the necessary software objects to perform data aquisitions
with the detectors supported by the slsDetectorsPackage.

The current implementation only works with the PSI/Eiger detectors.

Prerequisite
````````````
The slsDetectorsPackage-v2.3.x is needed by the SlsDetector LIMA plugin. In the compilation phase,
the SLS_DETECTORS_DIR environment variable must point to the directory where the previous package
is extracted; header files and shared libraries are used during compilation and linkage. In particular,
the *libSlsDetector.so* and *libSlsReceiver.so* shared libraries must be reachable both by the compiler
and from the *LD_LIBRARY_PATH* during execution time, in order to run the SlsDetector plugin.

In addition to that, a *configuration file*, containing the commands necessary to initialise both
the *slsDetector" and *slsReceiver* instances, is required.

The library protocol uses Unix System-V IPC shared memory blocks to exchange information between processes.
The segments, referred to by keys matching hex *000016xx*, must be owned by the user running the plugin,
if it is not *root*. The following command, which removes the existing segments, must be run by the segments' owner (or *root*) so they
can be deleted/created by another user:

.. code-block:: sh

  ipcs -m | \
    grep -E '^0x000016[0-9a-z]{2}' | \
    awk '{print $2}' | while read m; do \
      ipcrm -m $m; \
  done

High-performance Acquisitions
.............................

Installation of lid10eiger1
===========================

BIOS
----

Disable CPU power management
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

'lid10eiger1' - Detector PC Type B
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

[http://wikiserv.esrf.fr/bliss/index.php/High_Performance_Detector_PC#Fourth_Generation_-_E4.2FSupermicro_-_CCTF_Detector_PC_Type-B\ \|
High Performance Detector PC Type B]

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
    lid10eiger1:~ # mkdir -p ~/libc
    lid10eiger1:~ # cd ~/libc
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

Note: the partition is aligned to 2048 sectors (1 MByte). The end sector
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

and also to execute *sudo* in order to change other tasks' CPU affinity
(*taskset*) and configure configure the network devices (*ethtool* and
*ifconfig*):

::

    lid10eiger1:~ # cat /etc/sudoers.d/netperf
    %netperf    ALL=(root) NOPASSWD: /usr/bin/taskset, /sbin/ethtool, \
                         /sbin/ifconfig

Tune the OS network buffer sizes:

::

    lid10eiger1:~ # cat /etc/sysctl.d/net-performance.conf
    # Receive buffers

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

    # Max & default OS receive buffer size (Bytes) for all types
    net.core.rmem_max = 134217728
    net.core.rmem_default = 134217728

    # Buffer before Linux Kernel processes them
    net.core.netdev_max_backlog = 262144

    # The minimum UDP receive buffer size (Bytes)
    net.ipv4.udp_rmem_min = 134217728

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

From the tree we identify the parent root device of each Intel X520
Ethernet adapter:

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

    lid10eiger1:~ % mkdir -p ~/src/install
    lid10eiger1:~ % cd ~/src/install
    lid10eiger1:~/src/install % git clone git@blissinstaller.gitlab.esrf.fr:Admin/bliss_python_install.git
    lid10eiger1:~/src/install % ssh root@localhost
    The authenticity of host 'localhost (127.0.0.1)' can't be established.
    ECDSA key fingerprint is d7:da:38:9c:c4:20:8f:87:66:73:5a:85:62:44:01:f8.
    Are you sure you want to continue connecting (yes/no)? yes
    Warning: Permanently added 'localhost' (ECDSA) to the list of known hosts.
    root@localhost's password:
    ...

And execute the script as *root*:

::

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
    (bliss) lid10eiger1:~ % spyder

and go to 'Tools -> Preferences -> Console -> Advanced Settings ->
Python executable' and set:

+-----------------------------------+-----------------------------------+
| Path to Python interpreter        | /users/blissadm/lib/virtualenvs/b |
| executable binary                 | liss/bin/python                   |
+-----------------------------------+-----------------------------------+

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

Eiger software: SlsDetectorsPackage
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Copy, extract and patch the latest version of the 'SlsDetectorsPackage':

::

    # as opid00
    (bliss) lid10eiger1:~ % EIGER_DIR=${EIGER_HOME}/eiger/eiger_v2.3.2
    (bliss) lid10eiger1:~ % mkdir -p ${EIGER_DIR}
    (bliss) lid10eiger1:~ % scp lisgeiger1:${EIGER_DIR}/slsDetectorsPackage*.tar.gz ${EIGER_DIR}
    slsDetectorsPackagev2.3.2.tar.gz              100%   43MB  43.1MB/s   00:00

    (bliss) lid10eiger1:~ % EIGER_PATCHES=${EIGER_HOME}/eiger/patches
    (bliss) lid10eiger1:~ % mkdir -p ${EIGER_PATCHES}
    (bliss) lid10eiger1:~ % scp -r lisgeiger1:${EIGER_PATCHES}/* ${EIGER_PATCHES}
    ...

    (bliss) lid10eiger1:~ % cd ${EIGER_DIR}
    (bliss) lid10eiger1:~/eiger/eiger_v2.3.2 % tar -xzf slsDetectorsPackagev2.3.2.tar.gz

    (bliss) lid10eiger1:~/eiger/eiger_v2.3.2 % cd slsDetectorsPackage
    (bliss) lid10eiger1:~/eiger/eiger_v2.3.2/slsDetectorsPackage % \
        for e in *; do \
            [ -d ${e} ] || continue; \
            for p in ${EIGER_PATCHES}/*/${e}*.patch; do \
                [ -e ${p} ] || continue; \
                echo "Applying ${p} ..." | sed "s:${EIGER_PATCHES}/::" ; \
                (cd ${e} && patch -p1 < ${p}); \
            done; \
        done
    Applying 2017-04-27/slsDetectorSoftware-multiSlsDetector-decode-mod.patch ...
    patching file multiSlsDetector/multiSlsDetector.cpp
    Applying 2017-05-12/slsDetectorSoftware-fix-energy-calibration.patch ...
    patching file slsDetector/slsDetector.cpp
    patching file slsDetectorAnalysis/energyConversion.cpp
    Applying 2017-04-22/slsReceiverSoftware-force-system-buffers-with-error-logs.patch ...
    patching file include/genericSocket.h
    patching file src/UDPStandardImplementation.cpp
    Applying 2017-08-30/slsReceiverSoftware-quiet-acquisition.patch ...
    patching file src/UDPStandardImplementation.cpp

And build it (12 parallel jobs: one per core):

::

    (bliss) lid10eiger1:~/eiger/eiger_v2.3.2/slsDetectorsPackage % make clean
    (bliss) lid10eiger1:~/eiger/eiger_v2.3.2/slsDetectorsPackage % make -j12 2>&1 | tee /tmp/slsDetectorPackage.make.log
    ...

Include the detector software environment in *eiger_setup.sh*:

::

    (bliss) lid10eiger1:~ % tail -n 9 eiger_setup.sh

    EIGER_DIR=${EIGER_HOME}/eiger/eiger_v2.3.2
    SLS_DETECTORS_DIR=${EIGER_DIR}/slsDetectorsPackage
    SLS_DETECTORS_BIN=${SLS_DETECTORS_DIR}/bin
    export EIGER_DIR SLS_DETECTORS_DIR

    PATH=${EIGER_DIR}/eiger_scripts:${SLS_DETECTORS_BIN}:${PATH}
    LD_LIBRARY_PATH=${SLS_DETECTORS_BIN}:${LD_LIBRARY_PATH}
    export PATH LD_LIBRARY_PATH

Eiger-500k configuration
~~~~~~~~~~~~~~~~~~~~~~~~

Copy the Eiger-500k detector configuration file and adapt to the new
computer directories:

::

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

    rx_hostname lid10eiger1

    settingsdir /users/opid00/eiger/eiger_v2.3.2/slsDetectorsPackage/settingsdir/eiger
    lock 0
    #caldir /users/opid00/eiger/eiger_v2.3.2/slsDetectorsPackage/settingsdir/eiger
    outdir /nobackup/lid10eiger12/data/eiger

    tengiga 1
    threaded 1
    clkdivider 0
    flags parallel
    iodelay 651

    trimen 7 3000 3700 4500 5400 6400 8000 9900

    index 250

Copy the detector calibration data:

::

    (bliss) lid10eiger1:~ % SLS_DETECTORS_SETTINGS=$(grep ^settings ${EIGER_CONFIG} | awk '{print $2}')/standard
    (bliss) lid10eiger1:~ % mkdir -p $(dirname ${SLS_DETECTORS_SETTINGS})
    (bliss) lid10eiger1:~ % scp -r lisgeiger1:${SLS_DETECTORS_SETTINGS} $(dirname ${SLS_DETECTORS_SETTINGS})
    ...

Add the configuration file to *eiger_setup.sh* and decode the
*EIGER_MODULES*, together with the calibration directory:

::

    (bliss) lid10eiger1:~ % tail -n 7 eiger_setup.sh

    EIGER_CONFIG=${EIGER_DIR}/config/beb-021-020-direct-FO-10g.config
    EIGER_MODULES=$(grep "^hostname" ${EIGER_CONFIG} | cut -d" " -f2 | tr '+' ' ')
    export EIGER_CONFIG EIGER_MODULES

    SLS_DETECTORS_SETTINGS=$(grep ^settings ${EIGER_CONFIG} | awk '{print $2}')/standard
    export SLS_DETECTORS_SETTINGS

ESRF scripts on top of SlsDetectors software
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Install the [GitLab Hardware/sls_detectors
project\|\ https://gitlab.esrf.fr/Hardware/sls_detectors] (requires
personal SSH key added to the active *ssh-agent*):

::

    (bliss) lid10eiger1:~ % mkdir -p ~/esrf && cd ~/esrf
    (bliss) lid10eiger1:~/esrf % git clone -o gitlab git@gitlab.esrf.fr:Hardware/sls_detectors.git
    Cloning into 'sls_detectors'...
    ...

and add the scripts to *eiger_setup.sh*:

::

    (bliss) lid10eiger1:~ % tail -n 5 eiger_setup.sh

    SLS_DETECTORS_SCRIPTS=${EIGER_HOME}/esrf/sls_detectors/eiger/scripts
    export SLS_DETECTORS_SCRIPTS
    PATH=${SLS_DETECTORS_SCRIPTS}:${PATH}
    export PATH

Logout from *opid00* and login again in order to apply the previous
changes.

Finally, test the 'slsDetectorGui':

::

    (bliss) lid10eiger1:~ % start_eiger_gui
    ...

One *xterm* per Receiver (half-module) window should appear. Accept the
message box acknowleging the detector configuration parameters, and the
GUI will open. Wait for few seconds until a message box pops out asking
to activate the high voltage; answer *No*. In the GUI, disable the *File
Name* check box and press *Start* for a single acquisition. A frame
should be taken.

Setup *opid10* account
~~~~~~~~~~~~~~~~~~~~~~

Include the Eiger environment at login:

::

    # as opid10
    lid10eiger1:~ % tail -n 3 .bash_profile

    # include the PSI/Eiger environment
    . ${EIGER_HOME}/eiger_setup.sh

Lima
----

Lima compilation in detector software account
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

First install *flex*, which is needed to compile
*Lima/third-party/libconfig*:

::

    # as root
    lid10eiger1:~ # apt-get install flex
    ...

*Lima* is refereced as a submodule by the *sls_detectors* project
already install it:

::

    # as opid00
    (bliss) lid10eiger1:~ % cd ~/esrf/sls_detectors
    (bliss) lid10eiger1:~/esrf/sls_detectors % git branch slsdetector gitlab/slsdetector
    Branch slsdetector set up to track remote branch slsdetector from gitlab.
    (bliss) lid10eiger1:~/esrf/sls_detectors % git checkout slsdetector
    Switched to branch 'slsdetector'
    (bliss) lid10eiger1:~/esrf/sls_detectors % git submodule init Lima
    Submodule 'Lima' (git@gitlab.esrf.fr:limagroup/lima.git) registered for path 'Lima'
    (bliss) lid10eiger1:~/esrf/sls_detectors % git submodule update
    ...
    (bliss) lid10eiger1:~/esrf/sls_detectors % cd Lima
    (bliss) lid10eiger1:~/esrf/sls_detectors/Lima % git remote rename origin gitlab
    (bliss) lid10eiger1:~/esrf/sls_detectors/Lima % git remote add github.bliss git@github.com:esrf-bliss/Lima.git
    (bliss) lid10eiger1:~/esrf/sls_detectors/Lima % git fetch github.bliss
    The authenticity of host 'github.com (192.30.253.112)' can't be established.
    RSA key fingerprint is 16:27:ac:a5:76:28:2d:36:63:1b:56:4d:eb:df:a6:48.
    Are you sure you want to continue connecting (yes/no)? yes
    ...
    (bliss) lid10eiger1:~/esrf/sls_detectors/Lima % git branch slsdetector gitlab/slsdetector
    Branch slsdetector set up to track remote branch slsdetector from gitlab.
    (bliss) lid10eiger1:~/esrf/sls_detectors/Lima % git checkout slsdetector
    Switched to branch 'slsdetector'
    (bliss) lid10eiger1:~/esrf/sls_detectors/Lima % git submodule init third-party/Processlib third-party/Sps \
                                                               third-party/gldisplay third-party/libconfig \
                                                               camera/frelon camera/common/espia \
                                                               camera/slsdetector \
                                                               applications/spec applications/tango/python \
                                                               documentation
    ...
    (bliss) lid10eiger1:~/esrf/sls_detectors/Lima % git submodule update
    ...
    (bliss) lid10eiger1:~/esrf/sls_detectors/Lima % git submodule foreach git remote rename origin github.bliss
    ...
    (bliss) lid10eiger1:~/esrf/sls_detectors/Lima % git submodule foreach 'bash -c "git remote add gitlab \$(git config remote.github.bliss.url | sed \"s%git://github.com/esrf-bliss%git@gitlab.esrf.fr:limagroup%\")"'
    ...
    (bliss) lid10eiger1:~/esrf/sls_detectors/Lima % (cd third-party/Sps && git remote rm gitlab)
    (bliss) lid10eiger1:~/esrf/sls_detectors/Lima % git submodule foreach git fetch --all
    ...
    (bliss) lid10eiger1:~/esrf/sls_detectors/Lima % sed -e 's/\(COMPILE_GLDISPLAY\|COMPILE_SLSDETECTOR\|COMPILE_EDFGZ_SAVING\)=0/\1=1/' \
                                                        config.inc_default > config.inc
    (bliss) lid10eiger1:~/esrf/sls_detectors/Lima % make config
    ...
    (bliss) lid10eiger1:~/esrf/sls_detectors/Lima % (make -j12 && make -j12 -C sip && make install) 2>&1
    ...

Add *Lima* to the *PATH* and *LD_LIBRARY_PATH* environment variables in
*eiger_setup.sh*:

::

    (bliss) lid10eiger1:~ % tail -n 5 eiger_setup.sh

    LIMA_DIR=${HOME}/esrf/sls_detectors/Lima
    LD_LIBRARY_PATH=${LIMA_DIR}/install/Lima/${BLOS}/lib:${LD_LIBRARY_PATH}
    PYTHONPATH=${LIMA_DIR}/install:${PYTHONPATH}
    export LD_LIBRARY_PATH PYTHONPATH

Logout and re-login as *opid00*, so the previous changes can be tested:

::

    (bliss) lid10eiger1:~ % cd ~/esrf/sls_detectors/Lima
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

Replace the Lima installed directories by symbolic links to the
development version compiled on *opid00*:

::

    # as blissadm
    lid10eiger1:~ % LIMA_DIR=${EIGER_HOME}/esrf/sls_detectors/Lima
    lid10eiger1:~ % cd ~/python/bliss_modules
    lid10eiger1:~/python/bliss_modules % mv Lima Lima-pack
    lid10eiger1:~/python/bliss_modules % ln -s ${LIMA_DIR}/install/Lima
    lid10eiger1:~/python/bliss_modules % cd ~/applications
    lid10eiger1:~/applications % mv LimaCCDs LimaCCDs-pack
    lid10eiger1:~/applications % ln -s ${LIMA_DIR}/applications/tango/python LimaCCDs

Include the *SlsDetector* libraries in the *BLISS_LIB_PATH*:

::

    # as blissadm
    lid10eiger1:~ % . ${EIGER_HOME}/eiger_setup.sh
    (bliss) lid10eiger1:~ % blissrc -a BLISS_LIB_PATH ${SLS_DETECTORS_BIN}

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

Add LimaCCDs and SlsDetector class devices:

+-----------------------------------+-----------------------------------+
| LimaCCDs/eiger500k/DEVICE/LimaCCD | id10/limaccds/eiger500k           |
| s                                 |                                   |
+-----------------------------------+-----------------------------------+
| id10/limaccds/eiger500k->LimaCame | SlsDetector                       |
| raType                            |                                   |
+-----------------------------------+-----------------------------------+
| id10/limaccds/eiger500k->NbProces | 10                                |
| singThread                        |                                   |
+-----------------------------------+-----------------------------------+
| LimaCCDs/eiger500k/DEVICE/SlsDete | id10/slsdetector/eiger500k        |
| ctor                              |                                   |
+-----------------------------------+-----------------------------------+
| id10/slsdetector/eiger500k->confi | /users/opid00/eiger/eiger_v2.3.2/ |
| g_fname                           | config/beb-021-020-direct-FO-10g. |
|                                   | config                            |
+-----------------------------------+-----------------------------------+

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

**Note**: when running the *LimaCCDs* server from the *dserver_daemon*,
the real-time priority capabilities are lost. **The server must be
started using the *bliss_dserver* script**. Moreover, **the command
*bliss_dserver start* must be used (start in background, avoid *-fg*
option)**, so the *LimaCCDs* process is decoupled from the terminal,
reducing the risks of CPU blocking.

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
            }
        }
    }'

    lima_ccd_resetup_all

***Note:*** The 30 seconds timeout is necessary for large memory
allocations (long sequences)





Installation & Module configuration
````````````````````````````````````

- follow first the steps for the linux installation :ref:`linux_installation`

The minimum configuration file is *config.inc* :

.. code-block:: sh

 COMPILE_CORE=1
 COMPILE_SIMULATOR=0
 COMPILE_SPS_IMAGE=1
 COMPILE_ESPIA=0
 COMPILE_FRELON=0
 COMPILE_MAXIPIX=0
 COMPILE_PILATUS=0
 COMPILE_BASLER=0
 COMPILE_PROSILICA=0
 COMPILE_ROPERSCIENTIFIC=0
 COMPILE_MYTHEN=0
 COMPILE_MYTHEN3=0
 COMPILE_ADSC=0
 COMPILE_UEYE=0
 COMPILE_XH=0
 COMPILE_XSPRESS3=0
 COMPILE_ULTRA=0
 COMPILE_XPAD=0
 COMPILE_PERKINELMER=0
 COMPILE_ANDOR=0
 COMPILE_ANDOR3=0
 COMPILE_PHOTONICSCIENCE=0
 COMPILE_PCO=0
 COMPILE_MARCCD=0
 COMPILE_POINTGREY=0
 COMPILE_IMXPAD=0
 COMPILE_DEXELA=0
 COMPILE_RAYONIXHS=0
 COMPILE_AVIEX=0
 COMPILE_META=0
 COMPILE_MERLIN=0
 COMPILE_V4l2=0
 COMPILE_EIGER=0
 COMPILE_PIXIRAD=0
 COMPILE_HEXITEC=0
 COMPILE_SLSDETECTOR=1
 COMPILE_CBF_SAVING=0
 COMPILE_NXS_SAVING=0
 COMPILE_FITS_SAVING=0
 COMPILE_EDFGZ_SAVING=1
 COMPILE_EDFLZ4_SAVING=0
 COMPILE_TIFF_SAVING=0
 COMPILE_HDF5_SAVING=0
 COMPILE_CONFIG=1
 COMPILE_GLDISPLAY=0
 LINK_STRICT_VERSION=0
 export COMPILE_CORE COMPILE_SPS_IMAGE COMPILE_SIMULATOR \
        COMPILE_ESPIA COMPILE_FRELON COMPILE_MAXIPIX COMPILE_PILATUS \
        COMPILE_BASLER COMPILE_PROSILICA COMPILE_ROPERSCIENTIFIC COMPILE_ADSC \
        COMPILE_UEYE COMPILE_XH COMPILE_XSPRESS3 COMPILE_ULTRA COMPILE_XPAD COMPILE_PERKINELMER \
        COMPILE_MYTHEN COMPILE_MYTHEN3 COMPILE_HEXITEC \
        COMPILE_ANDOR COMPILE_ANDOR3 COMPILE_PHOTONICSCIENCE COMPILE_PCO COMPILE_MARCCD COMPILE_DEXELA \
        COMPILE_POINTGREY COMPILE_IMXPAD COMPILE_RAYONIXHS COMPILE_AVIEX COMPILE_META COMPILE_MERLIN COMPILE_V4l2 COMPILE_EIGER COMPILE_PIXIRAD \
        COMPILE_SLSDETECTOR \
        COMPILE_CBF_SAVING COMPILE_NXS_SAVING COMPILE_FITS_SAVING COMPILE_EDFGZ_SAVING COMPILE_EDFLZ4_SAVING COMPILE_TIFF_SAVING \
        COMPILE_HDF5_SAVING COMPILE_CONFIG COMPILE_GLDISPLAY \
        LINK_STRICT_VERSION

-  start the linux compilation :ref:`linux_compilation`

-  finally with the Tango server installation :ref:`tango_installation`

Initialisation and Capabilities
````````````````````````````````
In order to help people to understand how the camera plugin has been implemented in LImA this section
provides some important information about the developer's choices.

Camera initialisation
......................
The SlsDetector plugin exports to kind classes: one generic *SlsDetector::Camera* class, with the common
interface to *slsDetector* and *slsReceiver* classes, and detector-specific classes, like *SlsDetector::Eiger*
which manage the particularities of each model.

First, the *SlsDetector::Camera* must be instantiated with the configuration file, and once the connection to
the detector is established, a specific class is created depending on the detected type:

.. code-block:: python

    cam = SlsDetector.Camera(config_fname)
    if cam.getType() == SlsDetector.Camera.EigerDet:
        eiger = SlsDetector.Eiger(cam)
    else:
        raise RuntimeError("Non-supported type: %s" % cam.getType())

    hw_inter = SlsDetector.Interface(cam)
    ct = Core.CtControl(hw_inter)

The raw images returned by the *slsReceiver* class might need to be reconstructed, like in the case of
the PSI/Eiger detector. A LImA software reconstruction task must be then created from the LImA plugin and registered
to the *Core::CtControl* layer:

    if cam.getType() == SlsDetector.Camera.EigerDet:
        corr = eiger.createCorrectionTask()
        ct.setReconstructionTask(corr)

Std capabilites
................

This plugin has been implemented in respect of the mandatory capabilites but with limitations according
due to the detector specific features and with some programmer's  choices.  We do not explain here the
standard Lima capabilites but you can find in this section the useful information on the SlsDetector specfic features.

* HwDetInfo

TODO

* HwSync

The following trigger modes are currently implemented:

  + IntTrig
  + ExtTrigSingle
  + ExtTrigMult
  + ExtGate

The minimum *latency_time* and the *max_frame_rate* are automatically updated depending on
the *PixelDepth* (4, 8, 16, 32), the *ClockDiv* (Full-, Half-, Quarter-, SuperSlow-Speed),
and the *ReadoutFlags* (Parallel, Non-Parallel).

Optional capabilites
........................
In addition to the standard capabilities, we make the choice to implement some optional capabilities in order to
have an improved simulation.

* HwShutter

*Not implemented*

* HwRoi

*Not implemented*

* HwBin

*Not implemented*

Configuration
`````````````

The main configuration will consist in providing the correct *config file* file to the *slsDetector API*.
As mentioned before, the file is a list of commands accepted by *sls_detector_put*, and it should also
work with the *slsDetectorGui* application.

Two important parameters define the image frame dimension:

* PixelDepth:

  + 4 bit (not implemented yet)
  + 8 bit
  + 16 bit
  + 32 bit

* RawMode:

If set to *True*, the image is exported to LiMA as given from the Receiver(s), without any software reconstruction.

How to use
````````````
The LimaCCDs Tango server provides a complete interface to the SlsDetector plugin so feel free to test.

For a quick test one can use Python, this a short code example to work with the PSI/Eiger detector:

.. code-block:: python

  from Lima import SlsDetector
  from Lima import Core
  import time
  import sys

  config_fname = sys.argv[1]

  cam = SlsDetector.Camera(config_fname)
  if cam.getType() != SlsDetector.Camera.EigerDet:
    raise RuntimeError("Non-supported type: %s" % cam.getType())

  eiger = SlsDetector.Eiger(cam)
  hw_inter = SlsDetector.Interface(cam)
  ct = Core.CtControl(hw_inter)
  corr = eiger.createCorrectionTask()
  ct.setReconstructionTask(corr)

  acq = ct.acquisition()

  # setting new file parameters and autosaving mode
  saving = ct.saving()

  pars = saving.getParameters()
  pars.directory = '/tmp'
  pars.prefix = 'test_slsdetector_'
  pars.suffix = '.edf'
  pars.fileFormat = Core.CtSaving.EDF
  pars.savingMode = Core.CtSaving.AutoFrame
  saving.setParameters(pars)

  # now ask for 0.2 sec. exposure and 10 frames
  acq.setAcqExpoTime(0.2)
  acq.setAcqNbFrames(10)

  ct.prepareAcq()
  ct.startAcq()

  # wait for last image (#9) ready
  lastimg = ct.getStatus().ImageCounters.LastImageReady
  while lastimg != 9:
    time.sleep(0.1)
    lastimg = ct.getStatus().ImageCounters.LastImageReady

  # read the first image
  im0 = ct.ReadImage(0)

  # cleanup in good order
  import gc
  del acq; gc.collect()
  del ct; gc.collect()
  del corr; gc.collect()
  del eiger; gc.collect()
  del hw_inter; gc.collect()
  del cam; gc.collect()

A more complete **test_slsdetector_control.py** Python script can be found under the *camera/slsdetector/test* directory.
