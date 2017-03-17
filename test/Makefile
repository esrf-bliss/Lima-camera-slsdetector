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
LIMA_DIR := $(shell pwd)/../../..

include $(LIMA_DIR)/config.inc
include $(LIMA_DIR)/control/control.inc

SRCS = test_slsdetector.cpp 

SLS_DETECTORS_BIN = $(SLS_DETECTORS_DIR)/bin
SLS_DETECTORS_SW = $(SLS_DETECTORS_DIR)/slsDetectorSoftware
SLS_DETECTORS_RECV = $(SLS_DETECTORS_DIR)/slsReceiverSoftware

SLS_DETECTORS_INCL_FLAGS = -I$(SLS_DETECTORS_SW)/include \
			   -I$(SLS_DETECTORS_SW)/commonFiles \
			   -I$(SLS_DETECTORS_SW)/slsDetector \
			   -I$(SLS_DETECTORS_SW)/multiSlsDetector \
			   -I$(SLS_DETECTORS_SW)/slsDetectorServer \
			   -I$(SLS_DETECTORS_SW)/slsDetectorAnalysis \
			   -I$(SLS_DETECTORS_RECV)/include \
			   -I$(SLS_DETECTORS_SW)/slsReceiverInterface

CPPFLAGS = -I../include $(CT_CPPFLAGS) $(SLS_DETECTORS_INCL_FLAGS)

DEPLIBS = $(CT_DEPLIBS) \
          $(LIMA_DIR)/build,limacore $(LIMA_DIR)/build,limaslsdetector
LDFLAGS = $(CT_LDFLAGS) $(call lib-depflags,$(DEPLIBS))
LDLIBS = $(CT_LDLIBS) $(call lib-deplibs,$(DEPLIBS))


test-progs = test_slsdetector test_slsdetector_control
old-progs  = 

all: 	$(test-progs)

test_slsdetector:		test_slsdetector.o
	$(CXX) $(LDFLAGS) -o $@ $+ $(LDLIBS)

test_slsdetector_control:	test_slsdetector_control.o
	$(CXX) $(LDFLAGS) -o $@ $+ $(LDLIBS)

clean:
	rm -f *.o *.P $(test-progs) $(old-progs)

test_slsdetector.o:		test_slsdetector.cpp test_slsdetector.h

test_slsdetector_control.o:	test_slsdetector_control.cpp

%.o : %.cpp
	$(COMPILE.cpp) $(CPPFLAGS) -MD $(CXXFLAGS) -o $@ $<
	@cp $*.d $*.P; \
	sed -e 's/#.*//' -e 's/^[^:]*: *//' -e 's/ *\\$$//' \
	-e '/^$$/ d' -e 's/$$/ :/' < $*.d >> $*.P; \
	rm -f $*.d

-include $(SRCS:.cpp=.P)
