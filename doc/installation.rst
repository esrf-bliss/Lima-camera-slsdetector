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
| Hyper-Threading | Enable  |
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

*Linux kernel*
~~~~~~~~~~~~~~

Update the *Linux kernel* from Debian repository in order to improve system reliability:

::

    # as root
    lid10eiger1:~ # linux_ver=$(dpkg --list linux-image-3.2\* | grep '^ii' | awk '{print $3}')
    lid10eiger1:~ # echo ${linux_ver}
    3.2.82-1

    lid10eiger1:~ # linux_pkgs=$(dpkg --list linux\* | grep ${linux_ver} | awk '{print $2}')
    lid10eiger1:~ # echo "${linux_pkgs}"
    linux-doc-3.2
    linux-headers-3.2.0-4-all-amd64
    linux-headers-3.2.0-4-amd64
    linux-headers-3.2.0-4-common
    linux-headers-3.2.0-4-common-rt
    linux-headers-3.2.0-4-rt-amd64
    linux-image-3.2.0-4-amd64
    linux-libc-dev:amd64
    linux-manual-3.2

    lid10eiger1:~ # apt-get install ${linux_pkgs}
    ...
    Get:1 http://lin-repo-master.esrf.fr/debian/stable/debian-security/ wheezy/updates/main linux-image-3.2.0-4-amd64 amd64 3.2.96-2 [23.5 MB]
    Get:2 http://lin-repo-master.esrf.fr/debian/stable/debian-security/ wheezy/updates/main linux-doc-3.2 all 3.2.102-1 [6,501 kB]
    Get:3 http://lin-repo-master.esrf.fr/debian/stable/debian-security/ wheezy/updates/main linux-headers-3.2.0-4-all-amd64 amd64 3.2.96-2 [270 kB]
    Get:4 http://lin-repo-master.esrf.fr/debian/stable/debian-security/ wheezy/updates/main linux-headers-3.2.0-4-amd64 amd64 3.2.96-2 [671 kB]
    Get:5 http://lin-repo-master.esrf.fr/debian/stable/debian-security/ wheezy/updates/main linux-headers-3.2.0-4-common amd64 3.2.96-2 [3,641 kB]
    Get:6 http://lin-repo-master.esrf.fr/debian/stable/debian-security/ wheezy/updates/main linux-headers-3.2.0-4-rt-amd64 amd64 3.2.96-2 [671 kB]
    Get:7 http://lin-repo-master.esrf.fr/debian/stable/debian-security/ wheezy/updates/main linux-headers-3.2.0-4-common-rt amd64 3.2.96-2 [3,646 kB]
    Get:8 http://lin-repo-master.esrf.fr/debian/stable/debian-security/ wheezy/updates/main linux-libc-dev amd64 3.2.102-1 [890 kB]
    Get:9 http://lin-repo-master.esrf.fr/debian/stable/debian-security/ wheezy/updates/main linux-manual-3.2 all 3.2.102-1 [3,051 kB]
    ...
    
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

Copy the good *getconf* binary (64-bit):

::

    lid10eiger1:~ # cd /usr/bin
    lid10eiger1:/usr/bin # mv getconf getconf.32
    lid10eiger1:~ # scp lid01eiger1:/usr/bin/getconf .
    ...

*cpufrequtils*
~~~~~~~~~~~~~~

The previous settings disable the ACPI CPU power management interface,
so the *loadcpufreq* INIT service will not be able to load
*acpi-cpufreq* kernel module. In case this is enabled in BIOS in the
future, force the 'performance' governor in *cpufrequtils* INIT service:

::

    # as root
    lid10eiger1:~ # cat > /etc/default/cpufrequtils <<'EOF'
    # valid values: userspace conservative powersave ondemand performance
    # get them from cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_available_governors
    GOVERNOR="performance"
    EOF

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

Conda volume
~~~~~~~~~~~~

Reduce the *data* volume in order to create a *conda* volume
and mount it on */opt/bliss/conda*:

::

    # as root
    
    lid10eiger1:~ # (
        # create conda volume
        vg="vg"
        data_name="data"
        conda_name="conda"
        conda_size="20"
        conda_size_unit="GiB"
        conda_size_suffix=${conda_size_unit:0:1}
        part="/dev/mapper/${vg}-${data_name}"
        umount ${part}
        e2fsck -f ${part}
        curr_size_full=$(lvdisplay ${part} | grep 'LV Size' | awk '{print $3 " " $4}')
        curr_size=$(echo ${curr_size_full} | cut -d" " -f1)
        curr_size_unit=$(echo ${curr_size_full} | cut -d" " -f2)
        [ ${curr_size_unit} == ${conda_size_unit} ] || exit ]
        new_size=$(expr $(printf "%.0f" ${curr_size}) - ${conda_size})
        resize2fs ${part} ${new_size}${conda_size_unit:0:1}
        lvresize -L${new_size}${conda_size_suffix} ${part}
        e2fsck -f ${part}
        mount ${part}
      
        free_size_full=$(vgdisplay ${vg} | 
                             grep 'Free \+PE / Size' | awk '{print $7 " " $8}')
        free_size=$(echo ${free_size_full} | cut -d" " -f1)
        free_size_unit=$(echo ${free_size_full} | cut -d" " -f2)
        [ ${free_size_unit} == ${conda_size_unit} ] || exit ]
        size_opt="-L ${conda_size}${conda_size_suffix}"
        [ $(printf "%.0f" ${free_size}) -eq ${conda_size} ] && \
          size_opt="-l 100%FREE"
        lvcreate -n ${conda_name} ${size_opt} ${vg}
        part="/dev/mapper/${vg}-${conda_name}"
        mkfs.ext4 ${part}
        part_dir="/opt/bliss/conda"
        mkdir -p ${part_dir}
        echo "${part} ${part_dir} ext4 relatime,nodev,nosuid 0 2" >> /etc/fstab
        mount ${part_dir}
        chown blissadm:bliss ${part_dir} 
    )

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

    lid10eiger1:~ # cat > /etc/security/limits.d/net-performance.conf <<'EOF'
    @netperf         -       rtprio 99
    EOF

Compile the *netdev_set_queue_cpu_affinity* util, used by the *SlsDetector* plugin
to change the network packet dispatching tasks' CPU affinity, and install it 
in */usr/local/bin*:

::

    lid10eiger1:~ # (
        cd /tmp
        prog_name="netdev_set_queue_cpu_affinity"
        cat > ${prog_name}.c <<'EOF'
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
            char *dev, *irq_queue, *p, fname[256], buffer[128];
            int irq, rps, fd, len, ret;
            long aff;
    
            if (argc != 5)
                    exit(1);
            irq = (strcmp(argv[1], "-i") == 0);
            rps = (strcmp(argv[1], "-r") == 0);
            if (!irq && !rps)
                    exit(2);
            if (!strlen(argv[2]) || !strlen(argv[3]) || !strlen(argv[4]))
                    exit(2);
    
            dev = argv[2];
            irq_queue = argv[3];
    
            errno = 0;
            aff = strtol(argv[4], &p, 0);
            if (errno || *p)
                    exit(3);
    
            len = sizeof(fname);
            if (irq)
                    ret = snprintf(fname, len, "/proc/irq/%s/smp_affinity",
                                   irq_queue);
            else
                    ret = snprintf(fname, len, "/sys/class/net/%s/queues/%s/rps_cpus",
                                   dev, irq_queue);
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
    EOF

        gcc -Wall -o ${prog_name} ${prog_name}.c
        rm -f ${prog_name}.c
        mv ${prog_name} /usr/local/bin
    )

Allow *netperf* users to execute *sudo* in order to change other tasks' CPU affinity
(*taskset* and *netdev_set_queue_cpu_affinity*), to configure the network devices (*ethtool* and
*ifconfig*) and start/stop system services like *irqbalance* (*service*):

::

    lid10eiger1:~ # cat > /etc/sudoers.d/netperf <<'EOF'
    %netperf        ALL=(root) NOPASSWD: /usr/bin/taskset, /sbin/ethtool, \
                                         /sbin/ifconfig, \
                                         /usr/local/bin/netdev_set_queue_cpu_affinity, \
                                         /usr/sbin/service
    EOF

Tune the OS network buffer sizes:

::

    lid10eiger1:~ # cat > /etc/sysctl.d/net-performance.conf <<'EOF'
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
    EOF

*cmake* & *libnuma-dev*
~~~~~~~~~~~~~~~~~~~~~~~

*Conda* includes *cmake*, needed to compile Lima. De-install the Debian 7 package,
if present, or the manually installed *cmake-3.8.0*:

::

    # as root
    lid10eiger1:~ # p=$(dpkg --list cmake\* | grep '^ii' | awk '{print $2}'); \
        [ -n "${p}" ] && dpkg --purge ${p}
    ...

    lid10eiger1:~ # \
        curr_cmake=$(which cmake)
        if [ -n "${curr_cmake}" ] && [ $(dirname ${curr_cmake}) == "/usr/local/bin" ]; then
            cmake_src=$(find ~ ~opid00 -type d -name cmake-3.8.0)
            [ -n "${cmake_src}" ] && cd ${cmake_src} && su -c "make uninstall"
        fi
    ...

The same applies to *libnuma-dev*:

::

    # as root
    lid10eiger1:~ # p=$(dpkg --list libnuma-dev | grep '^ii' | awk '{print $2}'); \
        [ -n "${p}" ] && dpkg --purge ${p}
    ...

*Xsession*
~~~~~~~~~~

*Xsession* executes *ssh-agent*, which has the *setgid* bit set. This forces
*Linux* to clear its *LD_LIBRARY_PATH*, and hence that of its descendant processes -
the full *X11* session. The following patch propagates the *LD_LIBRARY_PATH*
configured at login (*.bash_profile*) to the *X11* session:

::

    # as root
    lid10eiger1:~ # (
        Xsession_patch="/etc/X11/Xsession.d/80ld_library_path"
        [ -f ${Xsession_patch} ] || cat > ${Xsession_patch} <<'EOF'
    # This file is sourced by Xsession(5), not executed.
    
    # ensure LD_LIBRARY_PATH is propagated after ssh-agent is executed
    STARTUP="${LD_LIBRARY_PATH:+env LD_LIBRARY_PATH=$LD_LIBRARY_PATH} $STARTUP"
    
    # vim:set ai et sts=2 sw=2 tw=80:
    EOF
    )

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

Conda software installation
~~~~~~~~~~~~~~~~~~~~~~~~~~~

Before installing *Conda*, first stop the *BLISS daemons*:

::

    # as root
    lid10eiger1:~ # /users/blissadm/admin/etc/S70daemons stop

and delete the previous installation of the *BLISS pew environment*:

::

    # as blissadm
    lid10eiger1:~ % (
      rm -rf ${HOME}/.local/bin ${HOME}/.local/lib ${HOME}/lib
      sed -i '/\(PEW\|virtualenv\)/D' ${HOME}/local/BLISS_ENV_VAR 
      find ${HOME}/bin -type l |
          while read l; do
              t=$(readlink ${l})
              if echo ${t} | grep -q "${HOME}/.local/bin"; then
                  echo "Deleting ${l} -> ${t}"
                  rm -f ${l}
              fi
          done
      )
    Deleting /users/blissadm/bin/virtualenv-clone -> /users/blissadm/.local/bin/virtualenv-clone
    Deleting /users/blissadm/bin/virtualenv -> /users/blissadm/.local/bin/virtualenv
    Deleting /users/blissadm/bin/pythonz -> /users/blissadm/.local/bin/pythonz
    Deleting /users/blissadm/bin/pythonz_install -> /users/blissadm/.local/bin/pythonz_install
    Deleting /users/blissadm/bin/pew -> /users/blissadm/.local/bin/pew

Download and install *conda*:

::

    # as blissadm
    lid10eiger1:~ % (
        ln -s /opt/bliss/conda ${HOME}
        export http_proxy=http://proxy.esrf.fr:3128
        export https_proxy=${http_proxy}
        export no_proxy=.esrf.fr
        cat >> ${HOME}/.bash_profile <<EOF
    
    # Proxy
    export http_proxy=${http_proxy}
    export https_proxy=${https_proxy}
    export no_proxy=${no_proxy}
    EOF
        cd ${HOME}/Downloads
        wget https://repo.continuum.io/miniconda/Miniconda2-latest-Linux-x86_64.sh
        chmod a+x Miniconda2-latest-Linux-x86_64.sh 
        ./Miniconda2-latest-Linux-x86_64.sh
        # installation directory: /users/blissadm/conda/miniconda
    ) && . ${HOME}/.bash_profile


Configure the *conda* BCU ESRF channel, update the software
and create the *slsdetector* environment:

::

    # as blissadm
    lid10eiger1:~ % (
      cat > ${HOME}/conda/miniconda/.condarc <<'EOF'
    channels:
      - http://bcu-ci.esrf.fr/stable
      - defaults
      - conda-forge
    EOF
      ${HOME}/conda/miniconda/bin/conda update -n base conda
      ${HOME}/conda/miniconda/bin/conda create -n slsdetector-py37 python=3.7
    )

Patch *blissrc* to use *conda* environments:

::

    # as blissadm
    lid10eiger1:~ % (
      cat > /tmp/blissrc.patch <<'EOF'
    diff -Naur a/blissrc b/blissrc
    --- a/blissrc   2018-08-09 20:42:27.798785071 +0200
    +++ b/blissrc   2018-08-09 20:39:24.325593908 +0200
    @@ -359,17 +359,65 @@
         export LD_LIBRARY_PATH
       fi
     
    -  # Check if pew is required and load the good environment
    +  # Check if pew/conda is required and load the good environment
       if [ -n "${BLISS_PEW_DEFAULT_ENV}" ] && [ -z "${BLISS_PEW_ENV}" ]; then
         BLISS_PEW_ENV=${BLISS_PEW_DEFAULT_ENV}
         export BLISS_PEW_ENV
       fi
    +  if [ -n "${BLISS_CONDA_DEFAULT_ENV}" ] && [ -z "${BLISS_CONDA_ENV}" ]; then
    +    BLISS_CONDA_ENV=${BLISS_CONDA_DEFAULT_ENV}
    +    export BLISS_CONDA_ENV
    +  fi
    +  if [ -n "${BLISS_PEW_ENV}" -a -n "${BLISS_CONDA_ENV}" ]; then
    +    print "Warning: both PEW and CONDA are configured. Ignoring PEW"
    +    unset BLISS_PEW_ENV
    +  fi
    +  redo_cd=0
       if [ -n "${BLISS_PEW_ENV}" ] && [ "${BLISS_PEW_ENV}" != "NONE" ] && \
          [ -x "$(which pew)" ]; then
    -    ve_dir=$(HOME=$BLISSADM pew in ${BLISS_PEW_ENV} bash -c "echo \$VIRTUAL_ENV") && \
    -      . ${ve_dir}/bin/activate
    +    ve_dir=$(HOME=$BLISSADM \
    +             pew in ${BLISS_PEW_ENV} bash -c "echo \$VIRTUAL_ENV") && \
    +             . ${ve_dir}/bin/activate
         unset ve_dir
    -
    +    redo_cd=1
    +  fi
    +  if [ -n "${BLISS_CONDA_ENV}" ] && [ "${BLISS_CONDA_ENV}" != "NONE" ]; then
    +    this_prog=$(basename $(cd /proc/self && readlink exe))
    +    if ([ -z "${CONDA_DEFAULT_ENV}" ] ||
    +        [ "${CONDA_DEFAULT_ENV}" != "${BLISS_CONDA_ENV}" ]) && \
    +       [ -f "${BLISS_CONDA_BASE}/bin/activate" ] && \
    +       [ ${this_prog} == "bash" ]; then
    +      if [ -n "${BLISS_CONDA_BASE}" ]; then
    +        PATH=${BLISS_CONDA_BASE}/bin:${PATH}
    +        export PATH
    +      fi
    +      . activate ${BLISS_CONDA_ENV} > /dev/null
    +      CONDA_SYSROOT=$(echo ${CONDA_PREFIX}/*/sysroot)
    +      export CONDA_SYSROOT
    +      redo_cd=1
    +      BLISS_CONDA_PREV_PYTHONPATH=${PYTHONPATH}
    +    fi
    +    unset this_prog
    +    PYTHONPATH=$(echo ${PYTHONPATH} | 
    +                 sed -e "s,${BLISSADM}/python/bliss_modules\(/[^:]\+\)\?:,,g")
    +    export BLISS_CONDA_PREV_PYTHONPATH
    +  fi
    +  if [ -n "${BLISS_CONDA_ENV}" ] && [ "${BLISS_CONDA_ENV}" == "NONE" ] && \
    +     [ -n "${CONDA_DEFAULT_ENV}" ] && \
    +     [ -f "${BLISS_CONDA_BASE}/bin/deactivate" ]; then
    +    if [ $(basename $0) == "bash" ]; then
    +      . deactivate > /dev/null
    +      redo_cd=1
    +    else
    +      PATH=$(echo ${PATH} | sed "s,${CONDA_PREFIX}/bin:,,")
    +      export PATH
    +    fi
    +    PATH=$(echo ${PATH} | sed "s,${BLISS_CONDA_BASE}/bin:,,")
    +    PYTHONPATH=${BLISS_CONDA_PREV_PYTHONPATH}
    +    export PATH PYTHONPATH
    +    unset CONDA_SYSROOT BLISS_CONDA_PREV_PYTHONPATH
    +  fi
    +  if [ ${redo_cd} -ne 0 ]; then
         # redo the _esrf_cd if it is already defined (ESRF standard)
         if type _esrf_cd > /dev/null 2>&1; then
           _esrf_cd () {
    @@ -377,6 +425,7 @@
           }
         fi
       fi
    +  unset redo_cd
     
       # force BLISSADM [local/]bin to lead the PATH
       one_dir_re="${BLISSADM}/\(local/\)\?bin"
    EOF
      (cd ${HOME}/bin && patch -b -p1 < /tmp/blissrc.patch)
    )

Configure *blissadm* software to use *conda* with the default
*slsdetector-py37* environment:

::

    # as blissadm
    lid10eiger1:~ % \
      sed -i -e '/^BLISS_LIB_PATH/i\
    BLISS_CONDA_BASE=${BLISSADM}/conda/miniconda export BLISS_CONDA_BASE' -e '/^BLISS_LIB_PATH/i\
    BLISS_CONDA_DEFAULT_ENV=slsdetector-py37 export BLISS_CONDA_DEFAULT_ENV' ~/local/BLISS_ENV_VAR

Patch *bliss_drivers* and *bliss_daemons* so they **do not** use
the *conda* environment:

::

    # as blissadm
    lid10eiger1:~ % (
      cat > /tmp/bliss_drivers.patch <<'EOF'
    diff -Naur a/bliss_drivers b/bliss_drivers
    --- a/bliss_drivers     2018-08-10 15:49:35.306633892 +0200
    +++ b/bliss_drivers     2018-08-10 15:50:01.846229119 +0200
    @@ -7,6 +7,9 @@
     # Author(s):    A. Homs (ahoms@esrf.fr)
     ##########################################################################
     
    +BLISS_CONDA_ENV=NONE
    +export BLISS_CONDA_ENV
    +
     . blissrc
     
     COLUMNS=
    EOF
      (cd ${HOME}/bin && patch -b -p1 < /tmp/bliss_drivers.patch)
    
      cat > /tmp/bliss_daemons.patch <<'EOF'
    diff -Naur a/blisswatch b/blisswatch
    --- a/blisswatch        2018-08-09 20:03:15.274806479 +0200
    +++ b/blisswatch        2018-08-09 20:00:00.109181262 +0200
    @@ -2,6 +2,9 @@
     # Copyright (c) 2004 Bliss group, ESRF, France
     # System startup script for HardwareRepositoryServer
     
    +BLISS_CONDA_ENV=NONE
    +export BLISS_CONDA_ENV
    +
     . blissrc
     
     BLISSWATCHHOME=${BLISSADM}/admin/bliss_tools/blisswatch/
    diff -Naur a/dserver_daemon b/dserver_daemon
    --- a/dserver_daemon    2018-08-09 20:06:40.700207938 +0200
    +++ b/dserver_daemon    2018-08-09 20:07:01.347931182 +0200
    @@ -1,6 +1,9 @@
     #! /bin/ksh
     # Copyright (c) 2004 Bliss group, ESRF, France
     
    +BLISS_CONDA_ENV=NONE
    +export BLISS_CONDA_ENV
    +
     . blissrc
     DSERVERHOME=${BLISSADM}/admin/bliss_tools/dserver
     exec ${DSERVERHOME}/dserver_daemon.py 2>&1 >/dev/null
    EOF
      (cd ${HOME}/admin/daemon/src && patch -b -p1 < /tmp/bliss_daemons.patch)
    )

Install basic *conda* packages:

::

    # as blissadm
    lid10eiger1:~ % (
      . blissrc
      conda install gxx_linux-64 gxx-dbg_linux-64 cmake \
                    sip="4.19*" numpy gsl lz4-c=1.8.2 hdf5="1.10*" \
                    libpng gevent
      conda install -c valkyriesystemscorporation libnuma
    )

Restart the *BLISS daemons*:

::

    # as root
    lid10eiger1:~ # /users/blissadm/admin/etc/S70daemons start

BLISS software installation
~~~~~~~~~~~~~~~~~~~~~~~~~~~

Install *PyTango*, needed by *Lima*:

::

    # as blissadm
    lid10eiger1:~ % (. blissrc && conda install pytango)

Install the Python modules needed for building the HTML documentation
with Doxygen, Sphinx and Read-the-Docs:

::

    # as blissadm
    lid10eiger1:~ (. blissrc && conda install sphinx_rtd_theme breathe)
    ...

Include the Python *Scapy* interface por network packet capture and decoding:

::

    # as blissadm
    lid10eiger1:~ % (. blissrc && pip install scapy)
    ...

Eiger calibration development: *Seaborn* and *Spyder*
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The *seaborn* Python module and the *spyder* IDE for are used by Eiger
calibration development (Erik Frojdh). First, de-install any OS installation:

::

    # as root
    lid10eiger1:~ # p=$(dpkg --list spyder\* | grep '^ii' | awk '{print $2}'); \
        [ -n "${p}" ] && dpkg --purge ${p}

::

    # as blissadm
    lid10eiger1:~ % (. blissrc && conda install seaborn spyder)
    ...

Detector software and development account: *opid00*
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Define the Eiger software home
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Add the *eiger.sh* entry in the system-wide Bash login setup scripts:

::

    # as root
    lid10eiger1:~ # cat > /etc/profile.d/eiger.sh <<'EOF'
    EIGER_HOME=~opid00
    export EIGER_HOME
    EOF

Eiger environment setup
^^^^^^^^^^^^^^^^^^^^^^^

Create *eiger_setup.sh*, oriented to prepare the Eiger environment. In
the beginning it just contains the BLISS environment:

::

    # as opid00
    lid10eiger1:~ % cat > eiger_setup.sh <<'EOF'
    # Setup the Eiger data acquisition environment

    # include the BLISS environment
    . blissrc
    EOF

and include it in the *.bash_profile* so it is executed at every login
shell:

::

    lid10eiger1:~ % cat >> .bash_profile <<'EOF'

    # include the PSI/Eiger environment
    . ${EIGER_HOME}/eiger_setup.sh
    EOF

*git-sig* Bash helper
^^^^^^^^^^^^^^^^^^^^^

Add the *git-sig* Bash helper for authoring future commits:

::

    lid10eiger1:~ % cat >> .bashrc <<'EOF'

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
    EOF

Logout from *opid00* and re-login so changes are taken into account for
next steps.

Eiger-500k configuration
~~~~~~~~~~~~~~~~~~~~~~~~

Copy the Eiger-500k detector configuration file and adapt to the new
computer directories:

::

    (slsdetector-py37) lid10eiger1:~ % \
        EIGER_DIR=${EIGER_HOME}/eiger/eiger_v3.1.1
        EIGER_CONFIG=${EIGER_DIR}/config/beb-021-020-direct-FO-10g.config
        mkdir -p $(dirname ${EIGER_CONFIG})
        scp lisgeiger1:${EIGER_CONFIG} $(dirname ${EIGER_CONFIG})
        sed -i 's:lisgeiger1:lid10eiger1:g' ${EIGER_CONFIG}
    ...

The resulting configuration file:

::

    (slsdetector-py37) lid10eiger1:~ % cat ${EIGER_CONFIG}
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
    0:detectorip 192.168.12.22
    0:detectormac 00:50:c2:46:d9:2b
    0:flippeddatax 0

    #bottom
    1:rx_tcpport 1962
    1:rx_udpport 50012
    1:rx_udpport2 50013
    1:rx_udpip 192.168.14.1
    1:detectorip 192.168.14.23
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

    (slsdetector-py37) lid10eiger1:~ % \
        SLS_DETECTOR_SETTINGS=$(grep ^settings ${EIGER_CONFIG} | awk '{print $2}')/standard
        mkdir -p $(dirname ${SLS_DETECTOR_SETTINGS})
        scp -r lisgeiger1:${SLS_DETECTOR_SETTINGS} $(dirname ${SLS_DETECTOR_SETTINGS})
    ...

Add the configuration file to *eiger_setup.sh* and decode the
*EIGER_MODULES*, together with the calibration directory:

::

    (slsdetector-py37) lid10eiger1:~ % cat >> eiger_setup.sh <<'EOF'

    EIGER_DIR=${EIGER_HOME}/eiger/eiger_v3.1.1
    EIGER_CONFIG=${EIGER_DIR}/config/beb-021-020-direct-FO-10g.config
    EIGER_MODULES=$(grep "^hostname" ${EIGER_CONFIG} | cut -d" " -f2 | tr '+' ' ')
    export EIGER_DIR EIGER_CONFIG EIGER_MODULES

    SLS_DETECTOR_SETTINGS=$(grep ^settings ${EIGER_CONFIG} | awk '{print $2}')/standard
    export SLS_DETECTOR_SETTINGS
    EOF

Logout from *opid00* and login again in order to apply the previous
changes.


ESRF package for SlsDetectors
-----------------------------

Install the [GitLab Hardware/sls_detectors
project\|\ https://gitlab.esrf.fr/Hardware/sls_detectors]:

::

    (slsdetector-py37) lid10eiger1:~ % mkdir -p ~/esrf && cd ~/esrf
    (slsdetector-py37) lid10eiger1:~/esrf % git clone -o gitlab git://gitlab.esrf.fr/Hardware/sls_detectors.git
    Cloning into 'sls_detectors'...
    ...

Add the *ESRF scripts* to *eiger_setup.sh*:

::

    (slsdetector-py37) lid10eiger1:~ % cat >> eiger_setup.sh <<'EOF'

    SLS_DETECTORS=${EIGER_HOME}/esrf/sls_detectors
    export SLS_DETECTORS
    PATH=${SLS_DETECTORS}/eiger/scripts:${PATH}
    export PATH
    EOF

Logout and re-login as *opid00* to have the previous environment set.

Lima installation in detector software account
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

First install *flex*, which might needed to compile some *Lima* subsystems:

::

    # as root
    lid10eiger1:~ # apt-get install flex
    ...

De-install *libgsl* and *libnuma-dev*:

::

    # as root
    lid10eiger1:~ # \
        dpkg --purge inkscape bogofilter bogofilter-bdb libgsl0ldbl libgsl0-dev
        dpkg --purge libnuma-dev
    ...

*Lima* is referenced as a submodule by the *sls_detectors* project installed before:

::

    # as opid00
    (slsdetector-py37) lid10eiger1:~ % \
        cd ${SLS_DETECTORS}
        git submodule init Lima
        git submodule update
        LIMA_DIR=${SLS_DETECTORS}/Lima
        cd ${LIMA_DIR}
        submod="third-party/Processlib
            third-party/bitshuffle
            camera/slsdetector
            applications/spec
            applications/tango/python"
        ext_submod_names="bitshuffle"
        ext_submod=$(for s in ${submod}; do
                for m in ${ext_submod_names}; do
                    echo ${s} | grep ${m}
                done
            done)
        re_pat="(${ext_submod_names// /|})"
        gitlab_submod=$(echo "${submod}" | grep -Ev ${re_pat})
        git submodule init ${submod}
        git submodule update
        for s in ${gitlab_submod}; do
                (cd ${s} &&
                     git remote rename origin gitlab &&
                     git remote add github.bliss \
                         $(git config remote.gitlab.url |
                             sed "s%git://gitlab.esrf.fr/limagroup%git://github.com/esrf-bliss%"))
            done
        git remote rename origin gitlab
        git remote add github.bliss git://github.com/esrf-bliss/Lima.git
        git submodule foreach git fetch --all
        git fetch --all
    ...

Eiger software: slsDetectorPackage
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The *slsDetectorPackage* is in turn a submodule of the *Lima/camera/slsdetector*
plugin:

::

    # as opid00
    (slsdetector-py37) lid10eiger1:~ % \
        cd ${LIMA_DIR}/camera/slsdetector
        git submodule init
        git submodule update
        cd slsDetectorPackage
        git remote rename origin github.bliss
        git remote add github.slsdetectorgroup \
            git://github.com/slsdetectorgroup/slsDetectorPackage.git
        git fetch --all
    ...

*Lima* compilation
~~~~~~~~~~~~~~~~~~

Compile *Lima*, including *slsDetectorPackage* using *CMake*:

::

    (slsdetector-py37) lid10eiger1:~ % \
        cd ${LIMA_DIR}
        cp scripts/config.txt_default scripts/config.txt
        mkdir -p ${LIMA_DIR}/install/python
    (slsdetector-py37) lid10eiger1:~/esrf/sls_detectors/Lima % \
        cp camera/slsdetector/tango/SlsDetector.py \
           applications/tango/python/camera && \
        ./install.sh \
            --install-prefix=${LIMA_DIR}/install \
            --install-python-prefix=${LIMA_DIR}/install/python \
            slsdetector numa hdf5 hdf5-bs edfgz edflz4 python pytango-server tests 2>&1
    ...

Build the documentation:

::

    (slsdetector-py37) lid10eiger1:~/esrf/sls_detectors/Lima % make -C docs html
    ...

Add *Lima* to the *PATH*, *LD_LIBRARY_PATH* and *PYTHONPATH* environment variables in
*eiger_setup.sh*:

::

    (slsdetector-py37) lid10eiger1:~ % cat >> eiger_setup.sh <<'EOF'

    LIMA_DIR=${SLS_DETECTORS}/Lima
    PATH=${LIMA_DIR}/install/bin:${PATH}
    LD_LIBRARY_PATH=${LIMA_DIR}/install/lib:${LD_LIBRARY_PATH}
    PYTHONPATH=${LIMA_DIR}/install/python:${PYTHONPATH}
    export LIMA_DIR PATH LD_LIBRARY_PATH PYTHONPATH
    EOF

*eigerDetectorServer* and detector firmwares
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

If necessary, the *eigerDetectorServer* corresponding to the installed *slsDetectorPackage* version
must be copied into the modules embedded Linux. Please refer to :doc:`installation_eiger_server_and_fw`


Test the *slsdetector* *Lima* plugin
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Logout and re-login as *opid00*, so the previous changes can be tested. 
Test the *Lima* plugin without and with *CtControl* instantiation:

::

    (slsdetector-py37) lid10eiger1:~ % \
        cd ${LIMA_DIR}
        (rm -f /tmp/eiger.edf &&
             build/camera/slsdetector/test/test_slsdetector -c ${EIGER_CONFIG})
    ...
    (slsdetector-py37) lid10eiger1:~/esrf/sls_detectors/Lima % \
        mkdir -p /nobackup/lid10eiger12/data/eiger/lima
        ln -s /nobackup/lid10eiger12/data/eiger/lima data
        (rm -f data/img*.edf &&
             python camera/slsdetector/test/test_slsdetector_control.py -c ${EIGER_CONFIG})
    ...

Clean the shared memory segments used by the SlsDetector library, so
thay can be re-created by *opid10*:

::

    # as opid00
    (slsdetector-py37) lid10eiger1:~ % \
        for m in $(ipcs -m | grep '^0x000016' | awk '{print $2}'); do
            ipcrm -m ${m}
        done


Setup *opid10* account
~~~~~~~~~~~~~~~~~~~~~~

Include the Eiger environment at login:

::

    # as opid10
    lid10eiger1:~ % cat >> .bash_profile <<'EOF'

    # include the PSI/Eiger environment
    . ${EIGER_HOME}/eiger_setup.sh
    EOF

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
    lid10eiger1:~ % \
        . ${EIGER_HOME}/eiger_setup.sh
        blissrc -a BLISS_LIB_PATH ${LIMA_DIR}/install/lib
        blissrc -a PYTHONPATH ${LIMA_DIR}/install/python

Rename the Lima installed directories so they are no longer visible, and create the necessary
symbolic links:

::

    # as blissadm
    (slsdetector-py37) lid10eiger1:~ % \
        cd ~/python/bliss_modules
        mv Lima Lima-pack
        cd ~/applications
        mv LimaCCDs LimaCCDs-pack
        cd ~/server/src
        mv LimaCCDs LimaCCDs-pack
        ln -s ${LIMA_DIR}/install/bin/LimaCCDs


Lima Python Tango server configuration in *blissadm*
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Use *jive* server wizard to add the Lima Python Tango device server to
the Tango database:

::

    (slsdetector-py37) lid10eiger1:~ % jive > /dev/null 2>&1 &

Define the server *LimaCCDs/eiger500k* and include it in the *dserver*
local database:

::

    # as blissadm
    lid10eiger1:~ % cat ~/local/daemon/config/device_servers
    [LimaCCDs]
    *eiger500k

::

    # as opid10
    (slsdetector-py37) lid10eiger1:~ % bliss_dserver -fg start LimaCCDs
    Starting: LimaCCDs/eiger500k

Add LimaCCDs and SlsDetector class devices.

+----------------------------------------------------------+------------------------------------------------------+
| LimaCCDs/eiger500k/DEVICE/LimaCCDs                       | id10/limaccds/eiger500k                              |
+----------------------------------------------------------+------------------------------------------------------+
| id10/limaccds/eiger500k->LimaCameraType                  | SlsDetector                                          |
+----------------------------------------------------------+------------------------------------------------------+
| id10/limaccds/eiger500k->NbProcessingThread              | 23                                                   |
+----------------------------------------------------------+------------------------------------------------------+
| id10/limaccds/eiger500k->BufferMaxMemory                 | 30                                                   |
+----------------------------------------------------------+------------------------------------------------------+
| LimaCCDs/eiger500k/DEVICE/SlsDetector                    | id10/slsdetector/eiger500k                           |
+----------------------------------------------------------+------------------------------------------------------+
| id10/slsdetector/eiger500k->config_fname                 | /users/opid00/eiger/eiger_v3.1.1/config/             |
|                                                          | beb-021-020-direct-FO-10g.config                     |
+----------------------------------------------------------+------------------------------------------------------+
| id10/slsdetector/eiger500k->pixel_depth_cpu_affinity_map | | { 4: (((((CPU( 6), CPU(18)), (CPU( 7), CPU(19))),  |
|                                                          |           (CPU(12), CPU(13), CPU(14))),              |
|                                                          | |        (((CPU( 9), CPU(21)), (CPU(10), CPU(22))),  |
|                                                          |           (CPU(15), CPU(16), CPU(17)))),             |
|                                                          | |       CPU(*(range( 1,  6) + range(12, 18))),       |
|                                                          | |       CPU(0),                                      |
|                                                          | |       (('eth0,eth1,eth2,eth4,eth6,eth7,eth8,eth9', |
|                                                          |           {-1: (CPU(0), CPU(0))}),                   |
|                                                          | |        ('eth3',                                    |
|                                                          |           {-1: (CPU( 8), CPU(20))}),                 |
|                                                          | |        ('eth5',                                    |
|                                                          |           {-1: (CPU(11), CPU(23))}))),               |
|                                                          | |   8: (((((CPU( 6), CPU(18)), (CPU( 7), CPU(19))),  |
|                                                          |           (CPU(18), CPU(19), CPU(20))),              |
|                                                          | |        (((CPU( 9), CPU(21)), (CPU(10), CPU(22))),  |
|                                                          |           (CPU(21), CPU(22), CPU(23)))),             |
|                                                          | |       CPU(*(range( 1,  6) + range(12, 18))),       |
|                                                          | |       CPU(0),                                      |
|                                                          | |       (('eth0,eth1,eth2,eth4,eth6,eth7,eth8,eth9', |
|                                                          |           {-1: (CPU(0), CPU(0))}),                   |
|                                                          | |        ('eth3',                                    |
|                                                          |           {-1: (CPU( 8), CPU(20))}),                 |
|                                                          | |        ('eth5',                                    |
|                                                          |           {-1: (CPU(11), CPU(23))}))),               |
|                                                          | |  16: '@8',                                         |
|                                                          | |  32: '@8'}                                         |
+----------------------------------------------------------+------------------------------------------------------+

.. note:: in order to perform high frame rate acquisitions, the CPU affinity must be fixed for 
   the following tasks:

   * Receivers ports (2x2=4): listeners, writers
   * Eiger processing threads: 3 per receiver, configurable
   * Lima processing threads
   * Other OS processes
   * Net-dev group packet dispatching for Rx queues: irq & processing

   The previous example is based on a dual 6-core CPUs backend with *Hyper-Threading Technology* (12 cores, 
   24 threads). After the data acquisition finishes the Lima processing threads will run also on the CPUs
   assigned to listeners and writers (0xfffffe), that is 23 cores in total, which is used for setting the
   NbProcessingThreads. Please note that there are three network groups and four pixel_depth->cpu_affinity
   settings (4-, 8-, 16- and 32-bit). The special global_affinity '@X' is a reference to pixel_depth X.

.. note:: The Intel 10 Gigabit Ethernet Server Adapter has multiple hardware FIFOs per port, called
   queues in the OS terminology. The hardware uses a hash algorithm to dispatch packets into the active
   queues, which includes the source IP address. Each FIFO has an associated IRQ, so the *irqbalance* service
   activates the Receive-Side Scaling (RSS) mechanism by distributing the queues IRQ Service 
   Routine (ISR) CPU affinity on different cores.
   
   The destination FIFO in the Intel adapter depends on the Eiger 10 Gigabit Ethernet data interface IP,
   and thus the CPU where the corresponding ISR will run. ISRs have higher priority than packet dispatch
   tasks. If the queue IRQs are serviced by the same CPU that does the packet dispatching, the latter is
   affected when the frame rate is important. Care must be taken to avoid this kind of CPU conflict by 
   having a deterministic CPU task distribution. With this aim, the Lima plugin sets not only the
   (network stack dispatching) Receive Packet Steering (RPS) CPU Affinity for both data interfaces eth3/eth5
   but also their effective hardware FIFO ISR CPU Affinity (netdev_group CPU affinity).
   
   Lima uses the *ethtool -S* command, which shows the statistics of a network device, including the
   bytes/packets received per FIFO, in order to determine the active FIFOs. Then it uses the files:
   
   * */proc/interrupts*
   * */sys/class/net/<netdev>/device/msi_irqs/<irq>*
   * */proc/irq/<irq>/smp_affinity_list*
   
   to find out the active device IRQs and to set their corresponding CPU affinities. If running,
   the *irqbalance* service is stopped before setting the IRQ affinity and restored during application
   cleanup.
                
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
    lid10eiger1:~ % (
        . ${EIGER_HOME}/eiger_setup.sh
        cd ~/spec/macros/lima
        ln -s ${LIMA_DIR}/applications/spec/limaslsdetector.mac
    )

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

    (slsdetector-py37) lid10eiger1:~ % cat ~blissadm/local/spec/spec.d/eiger/config
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

    (slsdetector-py37) lid10eiger1:~ % cat ~blissadm/local/spec/spec.d/eiger/setup
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
