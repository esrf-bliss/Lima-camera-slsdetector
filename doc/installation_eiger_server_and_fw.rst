Installation of PSI/Eiger embedded server and firmwares
=======================================================

Requirements
------------

The following elements are required in order to perform an upgrade of the PSI/Eiger
embedded server and firmwares (FWs):

* The detector names are expected to be in registered in the *hosts* name service (*/etc/hosts*, DNS),
  and reachable through the IPv4 layer. 

* The *${EIGER_HOME}/eiger_setup.sh* is installed and sourced in *opid00* account, defining *EIGER_CONFIG*, *EIGER_MODULES*

* The *GitLab Hardware/sls_detectors* project is expected to be installed in *${EIGER_HOME}/esrf/sls_detectors*.
  The different Git submodules are recursiverly cloned, so *Lima/camera/slsdetector/slsDetectorPackage* 
  is available.

Please refer to :doc:`installation` in order to fulfill these requirements.


SSH keys
--------

Backup the current PSI/Eiger-500k modules Linux root account SSH
*authorized_keys* and check that the *opid00@lisgeiger1* SSH public key 
is included:

::

    lisgeiger1:~ % \
        EIGER_MODULE_TOP=$(echo ${EIGER_MODULES} | cut -f1 -d" ")
        cat ~/.ssh/id_dsa.pub
        echo
        base_dir="${HOME}/eiger/log/${EIGER_DETECTOR}"
        this_dir="${base_dir}/$(date +%Y-%m-%d-%H%M)"
        mkdir -p ${this_dir} && cd ${this_dir}
        for m in ${EIGER_MODULES}; do
            ssh -x root@${m} cat .ssh/authorized_keys > ssh_authorized_keys_${m}
        done
        cat ssh_authorized_keys_${EIGER_MODULE_TOP} 
    ssh-dss AAAAB3NzaC1kc3MAAACBALGVR0qC2i/HgaJl4fuiwmOVrq46Bz3bs+o3/jdw/dqMaPjx35Ha
    shyC4zS+2wHyZVSjwTMIbVT8LPsNMGxL40ZxqWaAUyzn0XnjJMe3XT7h+yyx+iLUXvyCK489PAwT0srE
    iWbGNeQTgEYiwX+jqezQTiwss2sgypOrrwIrGrZBAAAAFQDjOUdgHjbCc1UMW37Zu+7b/AV1cQAAAIAu
    tumVMJlCqWv30RRscEVMxGuv7UYanyMSnp+fI6pIfAKfcme/tGGKYiiF6biHVmDKcL+tnYloZvuDvfM+
    T7k1vMIf9UcX9ZglJu+6B1N7ZRV8wO9ZKCHcTeVkNqbTu7clhsjjEjwO/EWQrPv7aZPSu8hcJr/dFfP3
    AskvvG04rgAAAIEAjS61ZiP1iGoP0UhJCf4uHDnGra41mvMyqGBFwji93XON49UmVYXVFfYcWeoDzpaH
    JQguFttMCNduiXmZfDYITld+86c9aWCR6g7re977ElFTbutWe+isu/ZFINXOvDEHmBUKd7++4lGDCjsM
    NOQQmG/Ftsi4jE2iWBaI91oyTQI= opid00@lisgeiger1

    ssh-rsa AAAAB3NzaC1yc2EAAAABIwAAAQEAxHKsWjHzIsIt0Pt1lkZA9Px5x0v6e2ZzCh+AEiQz1nk0
    l7kU6K0IzKvYbAj6HnQB3/epYHz6SBRrpsMQcFPHgtQIKlTzeKHNqS6pHBKxKR77jGGq70i7SWZsSBXP
    1/8QxUhV9CNbFJKAkEmvXa5ZO1fKgtiXYXR26X72foOKwZzOtPzOqz2IQ6icxqTMELK1H4R+s0Rvuurl
    lDibdzo5g3xuV9bu/HLYt19zUgIR0KoTzpZEyPSiNwNizUy5oqXiNL4ZdcCjsSwQ1lYDwbzrg5brQA8W
    onqu5dtK4/LSh97jJ7u2V7LCUPr1KZj7OUy1FOHg2MUUjJbJApAMAffXgQ== root@pc6698
    ssh-rsa AAAAB3NzaC1yc2EAAAABIwAAAQEA2cWXxdmZHHfBq0n7iE4aNB6iyHWIUSA1EbLbuCfGhSog
    cn+uTeRXdAQfBvHXAp2JbR+mX30ftMzq28TfWlMKPWvKgcEwh05WbhL9swRIj7+FTryxnZG+LDDpIAUg
    kaGEmrX/9gtQ0wBwsbWPPPuHP6rlvSfoX2QPl6DSEt4AJqKf2QJNfm1SdqpQZOK5bG77VcEt3GRob9Mu
    d4j9Y+Tb8CWnruDQl6+Z66ccWwjCI5mlhKkjzgkaJytURcvEP4y+htqpeW6maoXJmvtS900+SCv1dqTe
    lbiSd2hRMx2ElZ7WCdaspHwkxwYyDtaPyHrGYuunDUhLFk+nDBjCUcuFgQ== theidel@pc6698
    ssh-dss AAAAB3NzaC1kc3MAAACBALeM8Rw5A/3q3oWqP8OuVU1H12nL+UxQiHlpQj/rQ1C02+O1x3SZ
    M32gJZo99s8uFb2YTlbzxYDroLdxw5Z1fM22Dsf9vUDcNcAWfXQK0JV2LIsNLJKC/s3/O51GwvtgxtC4
    wMgG3MMe5kmyI34tJSuVMggcnBepssvH6oWGVNbJAAAAFQCbtQeAikwu20rUfn7eEiCOP/0p9wAAAIBE
    hXhaxl5Ht29VldHfYMMAjY3V1+d2XbleLl7ixOdQLcy2igssBR4w0QRz4V6p0ePCiIhzqiWf5Hvgc1u/
    TBK0Rj9STI1qU1s8A6x/qdNDTbgebSeO6CNGqDTvhIdMgX7dUhgtwx4lzgbKtOtYSZ1XkMr1omwMAK9n
    K7xFCNu4IAAAAIBS0Is0ryjPYiOuEnWptKy9FiBYv2edS3vJ3Ln74HjzJ8FW3jT76xwEiBcaBtvXZ5kM
    VLyDtR3LR63i11Nnrg7nNJa43S4sng9dbBnO3f5/7Ylv2ku/vhATtMouVRn8c+xP1CMEYpGhSIw+OzrG
    xs33FGKr0xkL2yofwrDtHNRA/Q== theidel@pc6698
    ssh-dss AAAAB3NzaC1kc3MAAACBALGVR0qC2i/HgaJl4fuiwmOVrq46Bz3bs+o3/jdw/dqMaPjx35Ha
    shyC4zS+2wHyZVSjwTMIbVT8LPsNMGxL40ZxqWaAUyzn0XnjJMe3XT7h+yyx+iLUXvyCK489PAwT0srE
    iWbGNeQTgEYiwX+jqezQTiwss2sgypOrrwIrGrZBAAAAFQDjOUdgHjbCc1UMW37Zu+7b/AV1cQAAAIAu
    tumVMJlCqWv30RRscEVMxGuv7UYanyMSnp+fI6pIfAKfcme/tGGKYiiF6biHVmDKcL+tnYloZvuDvfM+
    T7k1vMIf9UcX9ZglJu+6B1N7ZRV8wO9ZKCHcTeVkNqbTu7clhsjjEjwO/EWQrPv7aZPSu8hcJr/dFfP3
    AskvvG04rgAAAIEAjS61ZiP1iGoP0UhJCf4uHDnGra41mvMyqGBFwji93XON49UmVYXVFfYcWeoDzpaH
    JQguFttMCNduiXmZfDYITld+86c9aWCR6g7re977ElFTbutWe+isu/ZFINXOvDEHmBUKd7++4lGDCjsM
    NOQQmG/Ftsi4jE2iWBaI91oyTQI= opid00@lisgeiger1

Check that all the keys are identical:

::

    lisgeiger1:~/eiger/psi_eiger_500k_024_025/2018-04-01-1828 % \
        md5sum ssh_authorized_keys_beb*
    1c183bdaa3a2f27029fca84b9cb3b857  ssh_authorized_keys_beb024
    1c183bdaa3a2f27029fca84b9cb3b857  ssh_authorized_keys_beb025

In case the *opid00@lisgeiger1* public key is not included (not the case before), 
add them in order to open SSH sessions automatically on the detector modules:

::

    lisgeiger1:~/eiger/psi_eiger_500k_024_025/2018-04-01-1828 % \
        for m in ${EIGER_MODULES}; do
            ssh -x root@${m} sh -c '"cat >> .ssh/authorized_keys"' < ~/.ssh/id_dsa.pub
        done

Also check that the SSH public host keys are identical (same Linux image):

::

    lisgeiger1:~/eiger/psi_eiger_500k_024_025/2018-04-01-1828 % \
        for m in ${EIGER_MODULES}; do
            ssh-keygen -f ~/.ssh/known_hosts -F ${m} -l
        done
    # Host beb024 found: line 82 type RSA
    1040 21:78:5d:39:d5:cc:92:7a:42:f8:4d:69:38:3b:40:40 |1|m9PYbOqjp0h4qI8tq9u9H8x7pKQ=|wXZou5Y2oMKiULF5ZOuBjV0U7oo= (RSA)
    # Host beb025 found: line 84 type RSA
    1040 21:78:5d:39:d5:cc:92:7a:42:f8:4d:69:38:3b:40:40 |1|Hp2vpbDpXlxTjMSfxo+n+r3B+ZQ=|LaI693N3AKzUER5lgBOOwReHOpI= (RSA)

*eigerDetectorServer*
---------------------

Before transferring the new version of the *eigerDetectorServer*, keep track of 
the current versions stored on the modules:

::

    lisgeiger1:~/eiger/psi_eiger_500k_024_025/2018-04-01-1828 % \
        server_dir="executables"
        server_name="eigerDetectorServer"
        server="${server_dir}/${server_name}"
        server_str=$(echo ${server} | sed 's:/:_:g')
        full_server="/home/root/${server}"
        for m in ${EIGER_MODULES}; do
            ssh -x root@${m} 'ls -l '${server}'*' \
                > ls_${server_str}_${m}.out
        done
        cat ls_${server_str}_${EIGER_MODULE_TOP}.out
        echo
        for m in ${EIGER_MODULES}; do 
            ssh -x root@${m} 'md5sum '${server}'*' \
                > md5sum_${server_str}_${m}.out
        done
        cat md5sum_${server_str}_${EIGER_MODULE_TOP}.out
        echo
        md5sum md5sum_${server_str}_beb*
    -rwxr-xr-x    1 root     root        280601 Jan  1 01:15 executables/eigerDetectorServer
    -rwxr-xr-x    1 root     root        277442 Aug 26  2016 executables/eigerDetectorServer_bkp
    -rwxr-xr-x    1 root     root        277442 Aug 26  2016 executables/eigerDetectorServerv2.0.5.14.3
    -rwxr-xr-x    1 root     root        280601 Jan  1 01:14 executables/eigerDetectorServerv2.3.0.16.2

    4fca193db64ed991da785043e7769082  executables/eigerDetectorServer
    e8a39956bbcb4aac62f109188e8ddbb2  executables/eigerDetectorServer_bkp
    e8a39956bbcb4aac62f109188e8ddbb2  executables/eigerDetectorServerv2.0.5.14.3
    4fca193db64ed991da785043e7769082  executables/eigerDetectorServerv2.3.0.16.2

    754a871d0608c28aa7544230ca728f86  md5sum_executables_eigerDetectorServer_beb024.out
    754a871d0608c28aa7544230ca728f86  md5sum_executables_eigerDetectorServer_beb025.out

Kill the running servers and disable the automatic startup:

::

    lisgeiger1:~/eiger/psi_eiger_500k_024_025/2018-04-01-1828 % \
        for m in ${EIGER_MODULES}; do
            ssh -x root@${m} killall ${server_name}
        done
        for m in ${EIGER_MODULES}; do
            ssh -x root@${m} sed -i '"s:^#\?\('${full_server}'\).*$:#\1 \&:"' \
                                 /etc/init.d/board_com.sh
        done

Force a filesystem *sync* on each host to make the changes persistent,
just before power-cycling:

::

    lisgeiger1:~/eiger/psi_eiger_500k_024_025/2018-04-01-1828 % \
        for m in ${EIGER_MODULES}; do
            ssh -x root@${m} sync
        done

Power-cycle the detector and check that no *eigerDetectorServer* is running:

::

    lisgeiger1:~/eiger/psi_eiger_500k_024_025/2018-04-01-1828 % \
        for m in ${EIGER_MODULES}; do \
            ssh -x root@${m} 'ps -ef | grep '${server}' | grep -v grep'; \
        done

Backup the current version, and transfer the new version:

::

    lisgeiger1:~/eiger/psi_eiger_500k_024_025/2018-04-01-1828 % \
        for m in ${EIGER_MODULES}; do
            ssh -x root@${m} 'mv '${server}' '${server}'_bkp'
        done
        SLS_DETECTOR_PACKAGE=${LIMA_DIR}/camera/slsdetector/slsDetectorPackage
        new_servers=$(cd ${SLS_DETECTOR_PACKAGE} && find -name ${server_name}v\*)
        (cd ${SLS_DETECTOR_PACKAGE} && md5sum ${new_servers})
        echo
        new_server=${SLS_DETECTOR_PACKAGE}/$(echo "${new_servers}" | head -n 1)
        for m in ${EIGER_MODULES}; do
            scp ${new_server} root@${m}:${server_dir}
        done
        for m in ${EIGER_MODULES}; do
            ssh -x root@${m} "cp ${server_dir}/$(basename ${new_server}) ${server}"
        done
    50ef053f1ddd0b49314479a558c9c330  ./slsDetectorSoftware/eigerDetectorServer/bin/eigerDetectorServerv3.1.1.16.0
    50ef053f1ddd0b49314479a558c9c330  ./serverBin/eigerDetectorServerv3.1.1.16.0

    eigerDetectorServerv3.1.1.16.0               100%  286KB 286.2KB/s   00:00    
    eigerDetectorServerv3.1.1.16.0               100%  286KB 286.2KB/s   00:00    

Check that all is as expected:

::

    lisgeiger1:~/eiger/psi_eiger_500k_024_025/2018-04-01-1828 % \
        cd
        this_dir="${base_dir}/$(date +%Y-%m-%d-%H%M)"
        mkdir -p ${this_dir} && cd ${this_dir}
        for m in ${EIGER_MODULES}; do
            ssh -x root@${m} 'ls -l '${server}'*' \
                > ls_${server_str}_${m}.out
        done
        cat ls_${server_str}_${EIGER_MODULE_TOP}.out
        echo
        for m in ${EIGER_MODULES}; do
            ssh -x root@${m} 'md5sum '${server}'*' \
                > md5sum_${server_str}_${m}.out
        done
        cat md5sum_${server_str}_${EIGER_MODULE_TOP}.out
        echo
        md5sum md5sum_${server_str}_beb*
    -rwxr-xr-x    1 root     root        293085 Jan 10 02:35 executables/eigerDetectorServer
    -rwxr-xr-x    1 root     root        280601 Jan  1 01:15 executables/eigerDetectorServer_bkp
    -rwxr-xr-x    1 root     root        277442 Aug 26  2016 executables/eigerDetectorServerv2.0.5.14.3
    -rwxr-xr-x    1 root     root        280601 Jan  1 01:14 executables/eigerDetectorServerv2.3.0.16.2
    -rwxr-xr-x    1 root     root        293085 Jan 10 02:34 executables/eigerDetectorServerv3.1.1.16.0

    50ef053f1ddd0b49314479a558c9c330  executables/eigerDetectorServer
    4fca193db64ed991da785043e7769082  executables/eigerDetectorServer_bkp
    e8a39956bbcb4aac62f109188e8ddbb2  executables/eigerDetectorServerv2.0.5.14.3
    4fca193db64ed991da785043e7769082  executables/eigerDetectorServerv2.3.0.16.2
    50ef053f1ddd0b49314479a558c9c330  executables/eigerDetectorServerv3.1.1.16.0

    4168a104e53ee71f763ed5f0e0b43859  md5sum_executables_eigerDetectorServer_beb024.out
    4168a104e53ee71f763ed5f0e0b43859  md5sum_executables_eigerDetectorServer_beb025.out

Force a another filesystem *sync*:

::

    lisgeiger1:~/eiger/psi_eiger_500k_024_025/2018-04-01-1927 % \
        cd
        for m in ${EIGER_MODULES}; do
            ssh -x root@${m} sync
        done

And finally perform a *paranoid* check after power-cycling the detector:

::

    lisgeiger1:~ % \
        prev_dir=${this_dir}
        this_dir="${base_dir}/$(date +%Y-%m-%d-%H%M)"
        mkdir -p ${this_dir} && cd ${this_dir}
        for m in ${EIGER_MODULES}; do
            ssh -x root@${m} 'md5sum '${server}'*' \
                > md5sum_${server_str}_${m}.out
        done
        cd ..
        for m in ${EIGER_MODULES}; do
            (diff ${prev_dir}/md5sum_${server_str}_${m}.out ${this_dir} &&
                echo "${m} OK" || echo "${m} changed")
        done
    beb024 OK
    beb025 OK


Firmware flash
--------------

.. note:: older modules *beb021/020* (Eiger-500k #1) and *beb074/071/064/102/072/073/087/088*
   (Eiger-2M) use bigger Xilinx Virtex5 FX70T FPGAs in Front-End-Board (FEB). New modules
   like *beb024/025* (Eiger-500k #2) use in their FEBs Xilinx Virtex5 FX30T FPGAs.
   **Updated**: The module *beb024/025* FEBs were changed during its front-end update with
   Xilinx Virtex5 FX70T FPGAs. No automatic determination of the FPGA type can be performed by
   software. **Solved**: the *flash.config* file includes the *Feb* section with the
   *FpgaType*, which is used by the code below.

The current FWs (v18 and later) allow entering into flash mode from the Linux environment,
without the need of pressing the button in the rear panel. The latestversion of the 
*eiger_flash* utility exploits this and enters into flash mode automatically.

.. note:: two BEB FWs variants allow using fiber optic or twisted-pair (copper) transceivers:
   *beb_fiber.bit* and *beb_copper.bit*. The good file must be specified in the command below.

Run the *eiger_flash* utility to update the FEB left/right and BEB FWs,
as well as the kernel image:

::

    lisgeiger1:~ % (
        . ${EIGER_HOME}/eiger_setup.sh
        base_dir="${HOME}/eiger/log/${EIGER_DETECTOR}"

        cd ${SLS_DETECTORS}/eiger/config
        fw_ver=$(cat detector/${EIGER_DETECTOR}/setup/${EIGER_DETECTOR_SETUP}/\
    detector/fw)
        fw_dir="fw/${fw_ver}"
        flash_config="detector/${EIGER_DETECTOR}/setup/${EIGER_DETECTOR_SETUP}/\
    detector/flash.config"

        fpga_type=$(python <<EOF
    from configparser import ConfigParser
    c = ConfigParser()
    c.read("${flash_config}")
    print(f'{c["Feb"]["FpgaType"].lower()}')
    EOF
    )

        this_dir="${base_dir}/$(date +%Y-%m-%d-%H%M)"
        mkdir -p ${this_dir}
        eiger_flash -c ${flash_config} \
                    -m ${fw_dir}/beb_fiber.bit \
                    -l ${fw_dir}/feb_l_${fpga_type}.bit \
                    -r ${fw_dir}/feb_r_${fpga_type}.bit \
                    -k ${fw_dir}/simpleImage.virtex440-eiger-beb-hwid1_local \
                    -o ${this_dir}/eiger_flash.log ${EIGER_MODULES}
    )
    Eiger flash - Fri Sep 11 16:17:11 2020
    9ad0445fc4958ff780cc85998b5bf968  fw/v24/beb_fiber.bit
    0e872295daaf42278219dc938550daba  fw/v24/feb_l_fx70t.bit
    437976fee26a47bb6e9884adf10d5d77  fw/v24/feb_r_fx70t.bit
    1f27879faa7082f9ed2bb2b24b84ea99  fw/v24/simpleImage.virtex440-eiger-beb-hwid1_local

    Connecting to Extreme Switch esmgmt ...
    Getting status of ports 1,2 ...
    Quitting ...
    Done!
    [beb109] Executing: nc -p 3000 -u beb109 3000
    [beb116] Executing: nc -p 3000 -u beb116 3000
    [beb109] Not in firmware flash mode ... ping'ing ...
    [beb116] Not in firmware flash mode ... ping'ing ...
    [beb109] ping OK ... Check ssh ...
    [beb116] ping OK ... Check ssh ...
    [beb109] Checking flash-mode setup files ...
    [beb116] Checking flash-mode setup files ...
    [beb109] Starting flash-mode (boot_recovery) ...
    [beb116] Starting flash-mode (boot_recovery) ...
    [beb109] Waiting for flash-mode (20 sec) ...
    [beb116] Waiting for flash-mode (20 sec) ...
    [beb109] Restarting Ethernet connection ...
    [beb116] Restarting Ethernet connection ...
    Connecting to Extreme Switch esmgmt ...
    Restarting ports 1,2 ...
    Quitting ...
    Done!
    Connecting to Extreme Switch esmgmt ...
    Disabling Auto-Negotiation on ports 1,2 ...
    Quitting ...
    Done!
    [beb116] Waiting for connection (10 sec) ...
    [beb109] Waiting for connection (10 sec) ...
    Connecting to Extreme Switch esmgmt ...
    Getting status of ports 1,2 ...
    Quitting ...
    Done!
    [beb109] Executing: nc -p 3000 -u beb109 3000
    [beb116] Executing: nc -p 3000 -u beb116 3000
    [beb109] Entered into flash-mode OK!
    [beb116] Entered into flash-mode OK!
    [beb116] Executing: xterm -title Eiger beb116 console -e cat /tmp/eiger_flash_con_pipe.beb116
    [beb109] Executing: xterm -title Eiger beb109 console -e cat /tmp/eiger_flash_con_pipe.beb109
    [beb116] Uploading MAIN_BIT fw/v24/beb_fiber.bit to /fw0 (4923823 bytes)
    [beb109] Uploading MAIN_BIT fw/v24/beb_fiber.bit to /fw0 (4923823 bytes)
    [beb116] Transferred MAIN_BIT bit file fw/v24/beb_fiber.bit (took 2.1 sec)
    [beb116] Waiting for firmware flash to finish ...
    [beb109] Transferred MAIN_BIT bit file fw/v24/beb_fiber.bit (took 2.4 sec)
    [beb109] Waiting for firmware flash to finish ...
    [beb109] Firmware flash finished OK (took 35.1 sec)
    [beb116] Firmware flash finished OK (took 38.8 sec)
    [beb116] Uploading LEFT_BIT fw/v24/feb_l_fx70t.bit to /febl (3378270 bytes)
    [beb109] Uploading LEFT_BIT fw/v24/feb_l_fx70t.bit to /febl (3378270 bytes)
    [beb116] Transferred LEFT_BIT bit file fw/v24/feb_l_fx70t.bit (took 1.7 sec)
    [beb116] Waiting for firmware flash to finish ...
    [beb109] Transferred LEFT_BIT bit file fw/v24/feb_l_fx70t.bit (took 1.8 sec)
    [beb109] Waiting for firmware flash to finish ...
    [beb109] Firmware flash finished OK (took 181.3 sec)
    [beb116] Firmware flash finished OK (took 182.2 sec)
    [beb116] Uploading RIGHT_BIT fw/v24/feb_r_fx70t.bit to /febr (3378271 bytes)
    [beb109] Uploading RIGHT_BIT fw/v24/feb_r_fx70t.bit to /febr (3378271 bytes)
    [beb116] Transferred RIGHT_BIT bit file fw/v24/feb_r_fx70t.bit (took 1.7 sec)
    [beb116] Waiting for firmware flash to finish ...
    [beb109] Transferred RIGHT_BIT bit file fw/v24/feb_r_fx70t.bit (took 1.7 sec)
    [beb109] Waiting for firmware flash to finish ...
    [beb109] Firmware flash finished OK (took 185.5 sec)
    [beb116] Firmware flash finished OK (took 186.8 sec)
    [beb116] Uploading KERNEL_LOCAL fw/v24/simpleImage.virtex440-eiger-beb-hwid1_local to /kernel (2068980 bytes)
    [beb109] Uploading KERNEL_LOCAL fw/v24/simpleImage.virtex440-eiger-beb-hwid1_local to /kernel (2068980 bytes)
    [beb116] Transferred KERNEL_LOCAL bit file fw/v24/simpleImage.virtex440-eiger-beb-hwid1_local (took 1.1 sec)
    [beb116] Waiting for firmware flash to finish ...
    [beb109] Transferred KERNEL_LOCAL bit file fw/v24/simpleImage.virtex440-eiger-beb-hwid1_local (took 1.1 sec)
    [beb109] Waiting for firmware flash to finish ...
    [beb109] Firmware flash finished OK (took 13.5 sec)
    [beb116] Firmware flash finished OK (took 14.2 sec)
    Connecting to Extreme Switch esmgmt ...
    Enabling Auto-Negotiation on ports 1,2 ...
    Quitting ...
    Done!
    Press any key to quit ...

Showing in the console for the FX70T FW:

::

    *** Output from beb116 console ***
    TFTP WRQ (write request): /fw0
    Receiving bitfile for parallel flash location 0
    transfer done: total len = 4923823 
    field 3  key='a' len=  46  system.ncd;HW_TIMEOUT=FALSE;UserID=0xFFFFFFFF
    field 4  key='b' len=  15  5vfx100tff1136
    field 5  key='c' len=  11  2019/12/18
    field 6  key='d' len=   9  15:29:55
    field 7  len=4923712 
    Doing bitswap for Parallel Flash...done
    XFlash_Unlock()
    XFlash_Erase()
    XFlash_Write()
    Compare
    XFlash_Lock()
    Success
    TFTP WRQ (write request): /febl
    Receiving bitfile for spi flash feb left
    transfer done: total len = 3378270 
    field 3  key='a' len=  31  feb_left.ncd;UserID=0xFFFFFFFF
    field 4  key='b' len=  13  5vfx70tff665
    field 5  key='c' len=  11  2019/07/29
    field 6  key='d' len=   9  14:54:29
    field 7  len=3378176 
    Copying to WriteBuffer...done
    Chip Erase Starting
    address     = 0x00000000
    end_address = 0x00330000
    len         = 3378176
    Chip Erase Complete
    Writing
    done.. Now reading back
    Compare
    Success
    TFTP WRQ (write request): /febr
    Receiving bitfile for spi flash feb right
    transfer done: total len = 3378271 
    field 3  key='a' len=  32  feb_right.ncd;UserID=0xFFFFFFFF
    field 4  key='b' len=  13  5vfx70tff665
    field 5  key='c' len=  11  2019/07/29
    field 6  key='d' len=   9  14:57:22
    field 7  len=3378176 
    Copying to WriteBuffer...done
    Chip Erase Starting
    address     = 0x00000000
    end_address = 0x00330000
    len         = 3378176
    Chip Erase Complete
    Writing
    done.. Now reading back
    Compare
    Success
    TFTP WRQ (write request): /kernel
    Receiving linux kernel
    transfer done: total len = 2068980 
    Linux Kernel:  len=2068980
    XFlash_Unlock()
    XFlash_Erase()
    XFlash_Write()
    Compare
    XFlash_Lock()
    Success
    
Start the *eigerDetectorServer* and check that everything is OK:

::

    lisgeiger1:~ % \
        for m in ${EIGER_MODULES}; do
            ssh -x root@${m} 'nohup '${server}' > /dev/null 2>&1 &'
        done

Once verified that the new server runs fine with the new firmware, restore automatic startup:

::

    lisgeiger1:~ % \
        for m in ${EIGER_MODULES}; do
            ssh -x root@${m} sed -i '"s:^#\?\('${full_server}'\).*$:\1 \&:"' \
                                 /etc/init.d/board_com.sh
        done
        for m in ${EIGER_MODULES}; do
            ssh -x root@${m} sync
        done

Power-cycle the detector and verify that the servers start automatically:

::

    lisgeiger1:~ % \
        for m in ${EIGER_MODULES}; do \
            ssh -x root@${m} 'ps -ef | grep '${server}' | grep -v grep'; \
        done
      961 root       0:00 /home/root/executables/eigerDetectorServer
      965 root       0:00 /home/root/executables/eigerDetectorServer -stopserver
      961 root       0:00 /home/root/executables/eigerDetectorServer
      965 root       0:00 /home/root/executables/eigerDetectorServer -stopserver
