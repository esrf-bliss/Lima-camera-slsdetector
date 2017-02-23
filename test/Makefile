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
include ../../../config.inc

SRCS = test_sls_detector.cpp 

SLS_DETECTORS_BIN = $(SLS_DETECTORS_DIR)/bin
SLS_DETECTORS_SW = $(SLS_DETECTORS_DIR)/slsDetectorSoftware
SLS_DETECTORS_RECV = $(SLS_DETECTORS_DIR)/slsReceiverSoftware

LDFLAGS = -pthread -L$(SLS_DETECTORS_BIN) -L../../../build \
	  -L../../../third-party/Processlib/build 
LDLIBS = -llimaslsdetector -lSlsDetector -lSlsReceiver -llimacore -lprocesslib 

ifneq ($(COMPILE_CBF_SAVING), 0)
LDFLAGS += -L../../../third-party/CBFLib/lib
LDLIBS += -lcbf
endif

ifneq ($(COMPILE_CONFIG), 0)
LDFLAGS += -L../../../third-party/libconfig/lib/.libs
LDLIBS += -lconfig++
endif

LIMA_INCL_FLAGS = -I../../../hardware/include -I../../../common/include \
		  -I../../../control/include/ \
		  -I../../../control/software_operation/include/ \
		  -I../../../third-party/Processlib/core/include \
		  -I../../../third-party/Processlib/tasks/include \
		  -I../../../third-party/Sps/Include

SLS_DETECTORS_INCL_FLAGS = -I$(SLS_DETECTORS_SW)/include \
			   -I$(SLS_DETECTORS_SW)/commonFiles \
			   -I$(SLS_DETECTORS_SW)/slsDetector \
			   -I$(SLS_DETECTORS_SW)/multiSlsDetector \
			   -I$(SLS_DETECTORS_SW)/slsDetectorServer \
			   -I$(SLS_DETECTORS_SW)/slsDetectorAnalysis \
			   -I$(SLS_DETECTORS_RECV)/include \
			   -I$(SLS_DETECTORS_SW)/slsReceiverInterface



CXXFLAGS = -I../include $(LIMA_INCL_FLAGS) $(SLS_DETECTORS_INCL_FLAGS) \
	   -Wall -pthread -g

test-progs = test_sls_detector
old-progs  = 

all: 	$(test-progs)

test_sls_detector:		test_sls_detector.o
	$(CXX) $(LDFLAGS) -o $@ $+ $(LDLIBS)

clean:
	rm -f *.o *.P $(test-progs) $(old-progs)

%.o : %.cpp
	$(COMPILE.cpp) -MD $(CXXFLAGS) -o $@ $<
	@cp $*.d $*.P; \
	sed -e 's/#.*//' -e 's/^[^:]*: *//' -e 's/ *\\$$//' \
	-e '/^$$/ d' -e 's/$$/ :/' < $*.d >> $*.P; \
	rm -f $*.d

-include $(SRCS:.cpp=.P)
