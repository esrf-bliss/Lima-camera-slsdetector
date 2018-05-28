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

    lisgeiger1:~ % cat ~/.ssh/id_dsa.pub
    ssh-dss AAAAB3NzaC1kc3MAAACBALGVR0qC2i/HgaJl4fuiwmOVrq46Bz3bs+o3/jdw/dqMaPjx35Ha
    shyC4zS+2wHyZVSjwTMIbVT8LPsNMGxL40ZxqWaAUyzn0XnjJMe3XT7h+yyx+iLUXvyCK489PAwT0srE
    iWbGNeQTgEYiwX+jqezQTiwss2sgypOrrwIrGrZBAAAAFQDjOUdgHjbCc1UMW37Zu+7b/AV1cQAAAIAu
    tumVMJlCqWv30RRscEVMxGuv7UYanyMSnp+fI6pIfAKfcme/tGGKYiiF6biHVmDKcL+tnYloZvuDvfM+
    T7k1vMIf9UcX9ZglJu+6B1N7ZRV8wO9ZKCHcTeVkNqbTu7clhsjjEjwO/EWQrPv7aZPSu8hcJr/dFfP3
    AskvvG04rgAAAIEAjS61ZiP1iGoP0UhJCf4uHDnGra41mvMyqGBFwji93XON49UmVYXVFfYcWeoDzpaH
    JQguFttMCNduiXmZfDYITld+86c9aWCR6g7re977ElFTbutWe+isu/ZFINXOvDEHmBUKd7++4lGDCjsM
    NOQQmG/Ftsi4jE2iWBaI91oyTQI= opid00@lisgeiger1

    lisgeiger1:~ % if [ $(echo ${EIGER_MODULES} | wc -w) -eq 2 ]; then \
        det_name="500k$(echo ${EIGER_MODULES} | sed 's/ \?beb/_/g')"; \
    else \
        det_name="2m"; \
    fi; \
    base_dir="${HOME}/eiger/psi_eiger_${det_name}"

    lisgeiger1:~ % this_dir="${base_dir}/$(date +%Y-%m-%d-%H%M)"
    lisgeiger1:~ % mkdir -p ${this_dir} && cd ${this_dir}
    lisgeiger1:~/eiger/psi_eiger_500k_024_025/2018-04-01-1828 % for m in ${EIGER_MODULES}; do \
        ssh -x root@${m} cat .ssh/authorized_keys > ssh_authorized_keys_${m}; \
    done

    lisgeiger1:~/eiger/psi_eiger_500k_024_025/2018-04-01-1828 % cat ssh_authorized_keys_beb024 
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

    lisgeiger1:~/eiger/psi_eiger_500k_024_025/2018-04-01-1828 % md5sum ssh_authorized_keys_beb*
    1c183bdaa3a2f27029fca84b9cb3b857  ssh_authorized_keys_beb024
    1c183bdaa3a2f27029fca84b9cb3b857  ssh_authorized_keys_beb025
    
In case the *opid00@lisgeiger1* public key is not included (not the case before), 
add them in order to open SSH sessions automatically on the detector modules:

::

    lisgeiger1:~/eiger/psi_eiger_500k_024_025/2018-04-01-1828 % for m in ${EIGER_MODULES}; do \
        ssh -x root@${m} sh -c '"cat >> .ssh/authorized_keys"' < ~/.ssh/id_dsa.pub; \
    done
    
Also check that the SSH public host keys are identical (same Linux image):

::

    lisgeiger1:~/eiger/psi_eiger_500k_024_025/2018-04-01-1828 % for m in ${EIGER_MODULES}; do \
        ssh-keygen -f ~/.ssh/known_hosts -F ${m} -l; \
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

    lisgeiger1:~/eiger/psi_eiger_500k_024_025/2018-04-01-1828 % for m in ${EIGER_MODULES}; do \
        ssh -x root@${m} 'ls -l executables/eigerDetectorServer*' \
            > ls_executables_eigerDetectorServer_${m}.out; \
    done

    lisgeiger1:~/eiger/psi_eiger_500k_024_025/2018-04-01-1828 % cat ls_executables_eigerDetectorServer_beb024.out 
    -rwxr-xr-x    1 root     root        280601 Jan  1 01:15 executables/eigerDetectorServer
    -rwxr-xr-x    1 root     root        277442 Aug 26  2016 executables/eigerDetectorServer_bkp
    -rwxr-xr-x    1 root     root        277442 Aug 26  2016 executables/eigerDetectorServerv2.0.5.14.3
    -rwxr-xr-x    1 root     root        280601 Jan  1 01:14 executables/eigerDetectorServerv2.3.0.16.2

    lisgeiger1:~/eiger/psi_eiger_500k_024_025/2018-04-01-1828 %  for m in ${EIGER_MODULES}; do 
        ssh -x root@${m} 'md5sum executables/eigerDetectorServer*' \
            > md5sum_executables_eigerDetectorServer_${m}.out; \
    done

    lisgeiger1:~/eiger/psi_eiger_500k_024_025/2018-04-01-1828 %  cat md5sum_executables_eigerDetectorServer_beb024.out 
    4fca193db64ed991da785043e7769082  executables/eigerDetectorServer
    e8a39956bbcb4aac62f109188e8ddbb2  executables/eigerDetectorServer_bkp
    e8a39956bbcb4aac62f109188e8ddbb2  executables/eigerDetectorServerv2.0.5.14.3
    4fca193db64ed991da785043e7769082  executables/eigerDetectorServerv2.3.0.16.2

    lisgeiger1:~/eiger/psi_eiger_500k_024_025/2018-04-01-1828 % md5sum md5sum_executables_eigerDetectorServer_beb*
    754a871d0608c28aa7544230ca728f86  md5sum_executables_eigerDetectorServer_beb024.out
    754a871d0608c28aa7544230ca728f86  md5sum_executables_eigerDetectorServer_beb025.out

Backup the current version, and transfer the new version:

::

    lisgeiger1:~/eiger/psi_eiger_500k_024_025/2018-04-01-1828 % for m in ${EIGER_MODULES}; do \
        ssh -x root@${m} 'mv executables/eigerDetectorServer executables/eigerDetectorServer_bkp'; \
    done

    lisgeiger1:~/eiger/psi_eiger_500k_024_025/2018-04-01-1828 % SLS_DETECTOR_PACKAGE=${LIMA_DIR}/camera/slsdetector/slsDetectorPackage
    lisgeiger1:~/eiger/psi_eiger_500k_024_025/2018-04-01-1828 % eiger_servers=$(cd ${SLS_DETECTOR_PACKAGE} && find -name eigerDetectorServerv\*)
    lisgeiger1:~/eiger/psi_eiger_500k_024_025/2018-04-01-1828 % (cd ${SLS_DETECTOR_PACKAGE} && md5sum ${eiger_servers})
    50ef053f1ddd0b49314479a558c9c330  ./slsDetectorSoftware/eigerDetectorServer/bin/eigerDetectorServerv3.1.1.16.0
    50ef053f1ddd0b49314479a558c9c330  ./serverBin/eigerDetectorServerv3.1.1.16.0

    lisgeiger1:~/eiger/psi_eiger_500k_024_025/2018-04-01-1828 % eiger_server=${SLS_DETECTOR_PACKAGE}/$(echo "${eiger_servers}" | head -n 1)

    lisgeiger1:~/eiger/psi_eiger_500k_024_025/2018-04-01-1828 % for m in ${EIGER_MODULES}; do \
        scp ${eiger_server} root@${m}:executables; \
    done
    eigerDetectorServerv3.1.1.16.0               100%  286KB 286.2KB/s   00:00    
    eigerDetectorServerv3.1.1.16.0               100%  286KB 286.2KB/s   00:00    

    lisgeiger1:~/eiger/psi_eiger_500k_024_025/2018-04-01-1828 % for m in ${EIGER_MODULES}; do \
        ssh -x root@${m} "cp executables/$(basename ${eiger_server}) executables/eigerDetectorServer"; \
    done
    
    
Check that all is as expected:

::

    lisgeiger1:~/eiger/psi_eiger_500k_024_025/2018-04-01-1828 % cd
    lisgeiger1:~ % this_dir="${base_dir}/$(date +%Y-%m-%d-%H%M)"
    lisgeiger1:~ % mkdir -p ${this_dir} && cd ${this_dir}
    lisgeiger1:~/eiger/psi_eiger_500k_024_025/2018-04-01-1927 % for m in ${EIGER_MODULES}; do \
        ssh -x root@${m} 'ls -l executables/eigerDetectorServer*' \
            > ls_executables_eigerDetectorServer_${m}.out; \
    done

    lisgeiger1:~/eiger/psi_eiger_500k_024_025/2018-04-01-1927 % cat ls_executables_eigerDetectorServer_beb024.out 
    -rwxr-xr-x    1 root     root        293085 Jan 10 02:35 executables/eigerDetectorServer
    -rwxr-xr-x    1 root     root        280601 Jan  1 01:15 executables/eigerDetectorServer_bkp
    -rwxr-xr-x    1 root     root        277442 Aug 26  2016 executables/eigerDetectorServerv2.0.5.14.3
    -rwxr-xr-x    1 root     root        280601 Jan  1 01:14 executables/eigerDetectorServerv2.3.0.16.2
    -rwxr-xr-x    1 root     root        293085 Jan 10 02:34 executables/eigerDetectorServerv3.1.1.16.0

    lisgeiger1:~/eiger/psi_eiger_500k_024_025/2018-04-01-1927 % for m in ${EIGER_MODULES}; do \
        ssh -x root@${m} 'md5sum executables/eigerDetectorServer*' \
            > md5sum_executables_eigerDetectorServer_${m}.out; \
    done

    lisgeiger1:~/eiger/psi_eiger_500k_024_025/2018-04-01-1927 % cat md5sum_executables_eigerDetectorServer_beb024.out 
    50ef053f1ddd0b49314479a558c9c330  executables/eigerDetectorServer
    4fca193db64ed991da785043e7769082  executables/eigerDetectorServer_bkp
    e8a39956bbcb4aac62f109188e8ddbb2  executables/eigerDetectorServerv2.0.5.14.3
    4fca193db64ed991da785043e7769082  executables/eigerDetectorServerv2.3.0.16.2
    50ef053f1ddd0b49314479a558c9c330  executables/eigerDetectorServerv3.1.1.16.0

    lisgeiger1:~/eiger/psi_eiger_500k_024_025/2018-04-01-1927 % md5sum md5sum_executables_eigerDetectorServer_beb*
    4168a104e53ee71f763ed5f0e0b43859  md5sum_executables_eigerDetectorServer_beb024.out
    4168a104e53ee71f763ed5f0e0b43859  md5sum_executables_eigerDetectorServer_beb025.out
    
Force a filesystem *sync* on each host to make the changes persistent,
just before power-cycling:

::

    lisgeiger1:~/eiger/psi_eiger_500k_024_025/2018-04-01-1927 % cd
    lisgeiger1:~ % for m in ${EIGER_MODULES}; do \
        ssh -x root@${m} sync; \
    done
    
And finally perform a *paranoid* check after power-cycling the detector:

::

    lisgeiger1:~ % this_dir="${base_dir}/$(date +%Y-%m-%d-%H%M)"
    lisgeiger1:~ % mkdir -p ${this_dir} && cd ${this_dir}
    lisgeiger1:~/eiger/psi_eiger_500k_024_025/2018-04-01-1934 % for m in ${EIGER_MODULES}; do \
        ssh -x root@${m} 'md5sum executables/eigerDetectorServer*' \
            > md5sum_executables_eigerDetectorServer_${m}.out; \
    done

    lisgeiger1:~/eiger/psi_eiger_500k_024_025/2018-04-01-1934 % cd ..
    lisgeiger1:~/eiger/psi_eiger_500k_024_025 % for m in ${EIGER_MODULES}; do \
        diff 2018-04-01-1927/md5sum_executables_eigerDetectorServer_${m}.out 2018-04-01-1934 && \
            echo "${m} OK" || echo "${m} changed"; \
    done
    beb024 OK
    beb025 OK


Firmware flash
--------------

.. note:: older modules *beb021/020* (Eiger-500k #1) and *beb074/071/064/102/072/073/087/088*
   (Eiger-2M) use bigger Xilinx Virtex5 FX70T FPGAs in Front-End-Board (FEB). New modules
   like *beb024/025* (Eiger-500k #2) use in their FEBs Xilinx Virtex5 FX30T FPGAs. For the
   moment **no automatic determination of the FPGA type is performed by the *eiger_flash*
   utility**. **To-Do**: investigate if the */febl* and */febr banks* can be read through *tftp*
   and add a mapping of the FW MD5 signatures in order to identify the good type.

The new FWs (v18 and later) allow entering into flash mode from the Linux environment,
without the need of pressing the button in the rear panel. The latestversion of the 
*eiger_flash* utility exploits this and enters into flash mode automatically.

.. note:: In case the FW in the detector is too old (pre v18) and does not
   support software reset into flash mode, the *eiger_flash* utility will ask to
   manually push the internal buttons in the detector rear panel with a clip:

   ::

       lisgeiger1:~ % cd ~/eiger/fw_v18
       lisgeiger1:~/eiger/fw_v18 % ~/esrf/sls_detectors/eiger/scripts/eiger_flash -m beb_fiber.bit -l feb_l_fx70t.bit -r feb_r_fx70t.bit -k simpleImage.virtex440-eiger-beb-hwid1_local ${EIGER_MODULES}
       [beb024] Executing: nc -p 3000 -u beb024 3000
       [beb025] Executing: nc -p 3000 -u beb025 3000
       [beb024] Not in firmware flash mode ... ping'ing ...
       [beb025] Not in firmware flash mode ... ping'ing ...
       [beb024] ping OK ... Check ssh ...
       [beb025] ping OK ... Check ssh ...
       [beb024] Checking flash-mode setup files ...
       [beb025] Checking flash-mode setup files ...
       [beb024] Remote and local files differ!
       [beb024] Local: 7f0e3fb00aa722d1b9c0b943b1870c70  boot_recovery
       [beb024] Local: 89d25988ed13fbb94dd48ed4d6b49e0d  z_mem
       [beb024] Local: 3f95900e1928d3c59a6ec3afbc5373b0  z_mem_write
       [beb024] remote: No file found!
       [beb024] Copying flash-mode setup files ...
       [beb025] Remote and local files differ!
       [beb025] Local: 7f0e3fb00aa722d1b9c0b943b1870c70  boot_recovery
       [beb025] Local: 89d25988ed13fbb94dd48ed4d6b49e0d  z_mem
       [beb025] Local: 3f95900e1928d3c59a6ec3afbc5373b0  z_mem_write
       [beb025] remote: No file found!
       [beb025] Copying flash-mode setup files ...
       [beb025] Starting flash-mode (boot_recovery) ...
       [beb024] Starting flash-mode (boot_recovery) ...
       [beb025] Waiting for flash-mode (20 sec) ...
       [beb024] Waiting for flash-mode (20 sec) ...
       [beb025] Restarting Ethernet connection ...
       [beb025] Waiting for connection (10 sec) ...
       [beb024] Restarting Ethernet connection ...
       [beb024] Waiting for connection (10 sec) ...
       [beb025] Executing: nc -p 3000 -u beb025 3000
       [beb024] Executing: nc -p 3000 -u beb024 3000
       Hosts beb024,beb025 are not in firmware flash mode!
       Please insert a clip into the rear panel hole until all LEDs are red,
         and then wait until LED #4 blinks gren/red
       Press any key to quit ...

Run the *eiger_flash* utility to update the FEB left/right and BEB FWs,
as well as the kernel image:
 
::

    lisgeiger1:~ % cd ~/eiger/fw_v20
    lisgeiger1:~/eiger/fw_v20 % which eiger_flash
    /users/opid00/esrf/sls_detectors/eiger/scripts/eiger_flash
    
    lisgeiger1:~/eiger/fw_v20 % md5sum *
    b2b66c1acae90e3f2b4c4488e99d6b42  beb_copper.bit
    f9e6e360cfa696957cf4fd5035bed5e1  beb_fiber.bit
    fe59229e8ebdb5e8d76ff315cd28cc7d  feb_l_fx30t.bit
    eb42ebe9a3c580ab12de0b2c2a7c8c5d  feb_l_fx70t.bit
    7a988f0e39930bf86d9af9dee060ef04  feb_r_fx30t.bit
    4bf1f88d376fd9651b45c2b5b2b021eb  feb_r_fx70t.bit
    1f27879faa7082f9ed2bb2b24b84ea99  simpleImage.virtex440-eiger-beb-hwid1_local
    
    lisgeiger1:~/eiger/fw_v20 % eiger_flash -m beb_fiber.bit -l feb_l_fx30t.bit -r feb_r_fx30t.bit -k simpleImage.virtex440-eiger-beb-hwid1_local ${EIGER_MODULES} 
    [beb024] Executing: nc -p 3000 -u beb024 3000
    [beb025] Executing: nc -p 3000 -u beb025 3000
    [beb024] Not in firmware flash mode ... ping'ing ...
    [beb025] Not in firmware flash mode ... ping'ing ...
    [beb024] ping OK ... Check ssh ...
    [beb025] ping OK ... Check ssh ...
    [beb024] Checking flash-mode setup files ...
    [beb025] Checking flash-mode setup files ...
    [beb024] Starting flash-mode (boot_recovery) ...
    [beb025] Starting flash-mode (boot_recovery) ...
    [beb025] Waiting for flash-mode (20 sec) ...
    [beb024] Waiting for flash-mode (20 sec) ...
    [beb025] Restarting Ethernet connection ...
    [beb025] Disabling eth4 ...
    [beb024] Restarting Ethernet connection ...
    [beb024] Disabling eth2 ...
    [beb025] Enabling eth4 ...
    [beb025] Waiting for connection (10 sec) ...
    [beb024] Enabling eth2 ...
    [beb024] Waiting for connection (10 sec) ...
    [beb025] Executing: nc -p 3000 -u beb025 3000
    [beb025] Entered into flash-mode OK!
    [beb024] Executing: nc -p 3000 -u beb024 3000
    [beb024] Entered into flash-mode OK!
    [beb024] Uploading MAIN_BIT beb_fiber.bit to /fw0 (4923823 bytes)
    [beb025] Uploading MAIN_BIT beb_fiber.bit to /fw0 (4923823 bytes)
    [beb024] Transferred MAIN_BIT bit file beb_fiber.bit (took 1.4 sec)
    [beb024] Waiting for firmware flash to finish ...
    [beb025] Transferred MAIN_BIT bit file beb_fiber.bit (took 1.4 sec)
    [beb025] Waiting for firmware flash to finish ...
    [beb025] Firmware flash finished OK (took 44.1 sec)
    [beb024] Firmware flash finished OK (took 47.1 sec)
    [beb024] Uploading LEFT_BIT feb_l_fx30t.bit to /febl (1689721 bytes)
    [beb025] Uploading LEFT_BIT feb_l_fx30t.bit to /febl (1689721 bytes)
    [beb024] Transferred LEFT_BIT bit file feb_l_fx30t.bit (took 0.5 sec)
    [beb024] Waiting for firmware flash to finish ...
    [beb025] Transferred LEFT_BIT bit file feb_l_fx30t.bit (took 0.5 sec)
    [beb025] Waiting for firmware flash to finish ...
    [beb024] Firmware flash finished OK (took 94.4 sec)
    [beb025] Firmware flash finished OK (took 95.5 sec)
    [beb024] Uploading RIGHT_BIT feb_r_fx30t.bit to /febr (1689721 bytes)
    [beb025] Uploading RIGHT_BIT feb_r_fx30t.bit to /febr (1689721 bytes)
    [beb025] Transferred RIGHT_BIT bit file feb_r_fx30t.bit (took 0.5 sec)
    [beb025] Waiting for firmware flash to finish ...
    [beb024] Transferred RIGHT_BIT bit file feb_r_fx30t.bit (took 0.5 sec)
    [beb024] Waiting for firmware flash to finish ...
    [beb025] Firmware flash finished OK (took 94.0 sec)
    [beb024] Firmware flash finished OK (took 94.3 sec)
    [beb024] Uploading KERNEL_LOCAL simpleImage.virtex440-eiger-beb-hwid1_local to /kernel (2068980 bytes)
    [beb025] Uploading KERNEL_LOCAL simpleImage.virtex440-eiger-beb-hwid1_local to /kernel (2068980 bytes)
    [beb024] Transferred KERNEL_LOCAL bit file simpleImage.virtex440-eiger-beb-hwid1_local (took 0.6 sec)
    [beb024] Waiting for firmware flash to finish ...
    [beb025] Transferred KERNEL_LOCAL bit file simpleImage.virtex440-eiger-beb-hwid1_local (took 0.6 sec)
    [beb025] Waiting for firmware flash to finish ...
    [beb025] Firmware flash finished OK (took 16.2 sec)
    [beb024] Firmware flash finished OK (took 17.3 sec)
    Press any key to quit ...
    
Showing in the console for the FX30T FW:

::

    *** Output from beb024 console ***
    TFTP WRQ (write request): /fw0
    Receiving bitfile for parallel flash location 0
    transfer done: total len = 4923823 
    field 3  key='a' len=  46  system.ncd;HW_TIMEOUT=FALSE;UserID=0xFFFFFFFF
    field 4  key='b' len=  15  5vfx100tff1136
    field 5  key='c' len=  11  2017/08/17
    field 6  key='d' len=   9  14:08:39
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
    transfer done: total len = 1689721 
    field 3  key='a' len=  26  feb.ncd;UserID=0xFFFFFFFF
    field 4  key='b' len=  13  5vfx30tff665
    ERROR: Bitfile is for wrong FPGA type: 5vfx30tff665  expected: 5vfx70tff665
    Something went wrong. Perhaps it is a bit file for the smaller Front End FPGA, trying that...
    field 3  key='a' len=  26  feb.ncd;UserID=0xFFFFFFFF
    field 4  key='b' len=  13  5vfx30tff665
    field 5  key='c' len=  11  2017/08/17
    field 6  key='d' len=   9  11:19:48
    field 7  len=1689632 
    Copying to WriteBuffer...done
    Chip Erase Starting
    address     = 0x00000000
    end_address = 0x00190000
    len         = 1689632
    Chip Erase Complete
    Writing
    done.. Now reading back
    Compare
    Success
    TFTP WRQ (write request): /febr
    Receiving bitfile for spi flash feb right
    transfer done: total len = 1689721 
    field 3  key='a' len=  26  feb.ncd;UserID=0xFFFFFFFF
    field 4  key='b' len=  13  5vfx30tff665
    ERROR: Bitfile is for wrong FPGA type: 5vfx30tff665  expected: 5vfx70tff665
    Something went wrong. Perhaps it is a bit file for the smaller Front End FPGA, trying that...
    field 3  key='a' len=  26  feb.ncd;UserID=0xFFFFFFFF
    field 4  key='b' len=  13  5vfx30tff665
    field 5  key='c' len=  11  2017/08/17
    field 6  key='d' len=   9  11:06:48
    field 7  len=1689632 
    Copying to WriteBuffer...done
    Chip Erase Starting
    address     = 0x00000000
    end_address = 0x00190000
    len         = 1689632
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

Console output on the FX70T FW:

::

    *** Output from beb024 console ***
    TFTP WRQ (write request): /fw0
    Receiving bitfile for parallel flash location 0
    transfer done: total len = 4923823 
    field 3  key='a' len=  46  system.ncd;HW_TIMEOUT=FALSE;UserID=0xFFFFFFFF
    field 4  key='b' len=  15  5vfx100tff1136
    field 5  key='c' len=  11  2017/08/17
    field 6  key='d' len=   9  14:08:39
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
    transfer done: total len = 3378265 
    field 3  key='a' len=  26  feb.ncd;UserID=0xFFFFFFFF
    field 4  key='b' len=  13  5vfx70tff665
    field 5  key='c' len=  11  2017/08/17
    field 6  key='d' len=   9  11:06:42
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
    transfer done: total len = 3378265 
    field 3  key='a' len=  26  feb.ncd;UserID=0xFFFFFFFF
    field 4  key='b' len=  13  5vfx70tff665
    field 5  key='c' len=  11  2017/08/17
    field 6  key='d' len=   9  11:06:39
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
    
.. note:: **To-Do** add a *ManualEthernetConnection* restart in case the modules are not 
   directly connected to the backend computer, or just not defined in the 
   *eiger_flash* utility.
