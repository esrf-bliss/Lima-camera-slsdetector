############################################################################
# This file is part of LImA, a Library for Image Acquisition
#
# Copyright (C) : 2009-2011
# European Synchrotron Radiation Facility
# BP 220, Grenoble 38043
# FRANCE
#
# This is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 3 of the License, or
# (at your option) any later version.
#
# This software is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, see <http://www.gnu.org/licenses/>.
############################################################################
import os, sys, string, gc, time
import getopt
from Lima import Core, SlsDetector
import processlib

Core.DEB_GLOBAL(Core.DebModTest)

class ImageStatusCallback(Core.CtControl.ImageStatusCallback):

    Core.DEB_CLASS(Core.DebModTest, "ImageStatusCallback")

    @Core.DEB_MEMBER_FUNCT
    def __init__(self, ct, acq_state, print_time=1, sleep_time=0,
                 all_frames=False):
        Core.CtControl.ImageStatusCallback.__init__(self)

        self.m_ct = ct
        self.m_acq_state = acq_state
        self.m_nb_frames = 0
        
        self.m_last_print_ts = 0
        self.m_print_time = print_time
        self.m_sleep_time = sleep_time
        if all_frames:
            rate_policy = self.RateAllFrames
        else:
            rate_policy = self.RateAsFastAsPossible
        self.setRatePolicy(rate_policy)
        
    @Core.DEB_MEMBER_FUNCT
    def imageStatusChanged(self, img_status):
        last_acq_frame_nb = img_status.LastImageAcquired;
        last_base_frame_nb = img_status.LastBaseImageReady;
        last_ready_frame_nb = img_status.LastImageReady;
        last_saved_frame_nb = img_status.LastImageSaved;

        if last_acq_frame_nb == 0:
            ct_acq = self.m_ct.acquisition()
            self.m_nb_frames = ct_acq.getAcqNbFrames()

        acq_state_changed = False
        msg = ''
        if ((last_acq_frame_nb == self.m_nb_frames - 1) and 
            (self.m_acq_state.get() == Core.AcqState.Acquiring)):
            msg = 'All frames acquired!'

        if ((last_base_frame_nb == self.m_nb_frames - 1) and
            (self.m_acq_state.get() == Core.AcqState.Acquiring)):
            msg = 'All frames internally processed!'

        if ((last_ready_frame_nb == self.m_nb_frames - 1) and
            (self.m_acq_state.get() == Core.AcqState.Acquiring)):
            msg = 'All frames processed!'
            self.m_acq_state.set(Core.AcqState.Saving)
            acq_state_changed = True

        if last_saved_frame_nb == self.m_nb_frames - 1:
            msg = 'All frames saved!'
            self.m_acq_state.set(Core.AcqState.Finished)
            acq_state_changed = True
            
        now = time.time()
        if ((now - self.m_last_print_ts >= self.m_print_time) or
            acq_state_changed):
            deb.Always("Last Acquired: %8d, Last Base: %8d, " \
                       "Last Ready: %8d, Last Saved: %8d" %
                       (last_acq_frame_nb, last_base_frame_nb,
                        last_ready_frame_nb, last_saved_frame_nb))
                      
            self.m_last_print_ts = now

        if msg:
            deb.Always(msg)

        if self.m_sleep_time > 0:
            time.sleep(self.m_sleep_time)

class SlsDetectorAcq:

    Core.DEB_CLASS(Core.DebModTest, "SlsDetectorAcq")
    
    @Core.DEB_MEMBER_FUNCT
    def __init__(self, config_fname, use_events=False, print_time=1,
                 sleep_time=0, all_frames=False):
        self.m_cam           = SlsDetector.Camera(config_fname)
        self.m_hw_inter      = SlsDetector.Interface(self.m_cam)
        self.m_acq_state     = Core.AcqState()

        self.m_corr = None
        if self.m_cam.getType() == SlsDetector.Camera.EigerDet:
            self.m_eiger = SlsDetector.Eiger(self.m_cam)
            self.m_corr = self.m_eiger.createCorrectionTask()
        else:
            deb.Warning("Non-supported type: %s" % self.m_cam.getType())

        self.m_ct            = Core.CtControl(self.m_hw_inter)
        self.m_ct_acq        = self.m_ct.acquisition()
        self.m_ct_saving     = self.m_ct.saving()
        self.m_ct_image      = self.m_ct.image()
        self.m_ct_buffer     = self.m_ct.buffer()
        self.m_ct_display    = self.m_ct.display()

        self.m_use_events    = use_events
        self.m_print_time    = print_time
        self.m_sleep_time    = sleep_time
        
        if self.m_corr:
            self.m_ct.setReconstructionTask(self.m_corr)

        if self.m_use_events:
            cb = ImageStatusCallback(self.m_ct, self.m_acq_state, print_time,
                                     sleep_time, all_frames)
            self.m_img_status_cb = cb
            self.m_ct.registerImageStatusCallback(self.m_img_status_cb)

        self.m_ct_display.setNames('_ccd_ds_', 'slsdetector_live')
        self.m_ct_display.setActive(True)
        

    @Core.DEB_MEMBER_FUNCT
    def __del__(self):
        if self.m_use_events:
            del self.m_img_status_cb;	gc.collect()
            
        del self.m_ct_buffer, self.m_ct_image, self.m_ct_saving, self.m_ct_acq
        del self.m_ct;			gc.collect()
        del self.m_corr;		gc.collect()
        del self.m_eiger;		gc.collect()
        del self.m_acq_state;		gc.collect()
        del self.m_hw_inter;		gc.collect()
        del self.m_cam;			gc.collect()

    @Core.DEB_MEMBER_FUNCT
    def start(self):
        self.m_ct.prepareAcq()
        self.m_acq_state.set(Core.AcqState.Acquiring)
        self.m_ct.startAcq()

    @Core.DEB_MEMBER_FUNCT
    def wait(self):
        if self.m_use_events:
            state_mask = Core.AcqState.Acquiring | Core.AcqState.Saving
            self.m_acq_state.waitNot(state_mask)
        else:
            nb_frames = self.m_ct_acq.getAcqNbFrames()
            last_print_ts = 0
            running_states = [Core.AcqState.Acquiring, Core.AcqState.Saving]
            while self.m_acq_state.get() in running_states:
                img_status = self.m_ct.getImageStatus()
                last_acq_frame_nb = img_status.LastImageAcquired;
                last_saved_frame_nb = img_status.LastImageSaved;

                acq_state_changed = False
                msg = ''
                if ((last_acq_frame_nb == nb_frames - 1) and 
                    (self.m_acq_state.get() == Core.AcqState.Acquiring)):
                    msg = 'All frames acquired!'
                    self.m_acq_state.set(Core.AcqState.Saving)
                    acq_state_changed = True

                if last_saved_frame_nb == nb_frames - 1:
                    msg = 'All frames saved!'
                    self.m_acq_state.set(Core.AcqState.Finished)
                    acq_state_changed = True
            
                now = time.time()
                if ((now - last_print_ts >= self.m_print_time) or
                    acq_state_changed):
                    deb.Always("Last Acquired: %8d, Last Saved: %8d" % 
                               (last_acq_frame_nb, last_saved_frame_nb))
                    last_print_ts = now

                if msg:
                    print msg

                time.sleep(self.m_sleep_time)

        self.m_cam.waitState(SlsDetector.Idle);
        deb.Trace("Camera finished");

        pool_thread_mgr = processlib.PoolThreadMgr.get()
        pool_thread_mgr.wait()

    @Core.DEB_MEMBER_FUNCT
    def run(self):
        self.start()
        self.wait()

    @Core.DEB_MEMBER_FUNCT
    def initSaving(self, dir, prefix, suffix, idx, fmt, mode, frames_per_file):
        self.m_ct_saving.setDirectory(dir)
        self.m_ct_saving.setPrefix(prefix)
        self.m_ct_saving.setSuffix(suffix)
        self.m_ct_saving.setNextNumber(idx)
        self.m_ct_saving.setFormat(fmt)
        self.m_ct_saving.setSavingMode(mode)
        self.m_ct_saving.setFramesPerFile(frames_per_file)
        
    @Core.DEB_MEMBER_FUNCT
    def setExpTime(self, exp_time):
        self.m_ct_acq.setAcqExpoTime(exp_time)

    @Core.DEB_MEMBER_FUNCT
    def setNbAcqFrames(self, nb_acq_frames):
        self.m_ct_acq.setAcqNbFrames(nb_acq_frames)

    @Core.DEB_MEMBER_FUNCT
    def setBin(self, bin):
        self.m_ct_image.setBin(bin)

    @Core.DEB_MEMBER_FUNCT
    def setRoi(self, roi):
        self.m_ct_image.setRoi(roi)


@Core.DEB_GLOBAL_FUNCT
def test_slsdetector_control(config_fname, enable_debug, use_events, 
                             print_time, sleep_time, all_frames):

    if enable_debug:
        Core.DebParams.enableModuleFlags(Core.DebParams.AllFlags)
        Core.DebParams.enableTypeFlags(Core.DebParams.AllFlags)
    else:
        Core.DebParams.disableModuleFlags(Core.DebParams.AllFlags)

    deb.Always("Creating SlsDetectorAcq")
    acq = SlsDetectorAcq(config_fname, use_events, print_time, sleep_time,
                         all_frames)
    deb.Always("Done!")
    
    acq.initSaving("data", "img", ".edf", 0, Core.CtSaving.EDF, 
                   Core.CtSaving.AutoFrame, 1);

    deb.Always("First run with default pars")
    acq.run()
    deb.Always("Done!")
    
    exp_time = 100e-3
    acq.setExpTime(exp_time)

    nb_acq_frames = 50
    acq.setNbAcqFrames(nb_acq_frames)

    deb.Always("Run exp_time=%s, nb_acq_frames=%s" % (exp_time, nb_acq_frames))
    acq.run()
    deb.Always("Done!")
    
    bin = Core.Bin(2, 2)
    acq.setBin(bin)

    nb_acq_frames = 5
    acq.setNbAcqFrames(nb_acq_frames)

    deb.Always("Run bin=<%sx%s>, nb_acq_frames=%s" % 
               (bin.getX(), bin.getY(), nb_acq_frames))
    acq.run()
    deb.Always("Done!")
    
    roi = Core.Roi(Core.Point(256, 512), Core.Size(256, 512));
    acq.setRoi(roi);

    roi_tl, roi_size = roi.getTopLeft(), roi.getSize()
    deb.Always("Run roi=<%s,%s>-<%sx%s>" %
               (roi_tl.x, roi_tl.y,
                roi_size.getWidth(), roi_size.getHeight()))
    acq.run()
    deb.Always("Done!")
    

def main(argv):

    config_fname = None
    enable_debug = False
    use_events = False
    print_time = 1
    sleep_time = 0
    all_frames = False

    opts, args = getopt.getopt(argv[1:], 'c:dep:s:a')
    for opt, val in opts:
        if opt == '-c':
            config_fname = val
        if opt == '-d':
            enable_debug = True
        if opt == '-e':
            use_events = True
        if opt == '-p':
            print_time = float(val)
        if opt == '-s':
            sleep_time = float(val)
        if opt == '-a':
            all_frames = True

    if not config_fname:
        raise ValueError("Must provide the configuration file")

    test_slsdetector_control(config_fname, enable_debug, use_events, 
                             print_time, sleep_time, all_frames)

        
if __name__ == '__main__':
    main(sys.argv)
