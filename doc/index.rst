.. _camera-slsdetector:

SlsDetector camera
------------------

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
The *slsDetectorPackage-v2.3.x* is needed by the SlsDetector LIMA plugin. As explained in
:doc:`installation`, the *slsDetectorPackage* is included as a submodule in the SlsDetector camera
plugin. It will be automatically compiled and installed during the LIMA build procedure.

In addition to that, a *configuration file*, containing the commands necessary to initialise both
the *slsDetector" and *slsReceiver* instances, is required.

The library protocol uses Unix System-V IPC shared memory blocks to exchange information between processes.
The segments, referred to by keys matching hex *000016xx*, must be owned by the user running the plugin,
if it is not *root*. The following command, which removes the existing segments, must be run by the
segments' owner (or *root*) so they can be deleted/created by another user:

.. code-block:: sh

  ipcs -m | \
    grep -E '^0x000016[0-9a-z]{2}' | \
    awk '{print $2}' | while read m; do \
      ipcrm -m $m; \
  done

High-performance Acquisitions
.............................

High-performance acquisitions require a specific backend computer setup.
Please refer to the :doc:`installation`.

Installation & Module configuration
```````````````````````````````````

- Follow the steps indicated in :doc:`installation`

As a reference, see:

- :ref:`build_installation`

- :ref:`tango_installation`


Initialisation and Capabilities
````````````````````````````````
In order to help people to understand how the camera plugin has been implemented in LImA this section
provides some important information about the developer's choices.

Camera initialisation
......................
The SlsDetector plugin exports two kind classes: one generic *SlsDetector::Camera* class, with the common
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
