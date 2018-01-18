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

SLS_DETECTORS_DIR = ../slsDetectorPackage
SLS_DETECTORS_BIN = $(SLS_DETECTORS_DIR)/bin
SLS_DETECTORS_SW = $(SLS_DETECTORS_DIR)/slsDetectorSoftware
SLS_DETECTORS_RECV = $(SLS_DETECTORS_DIR)/slsReceiverSoftware

SLS_DETECTORS_INCL_FLAGS = -I$(SLS_DETECTORS_SW)/include \
			   -I$(SLS_DETECTORS_SW)/commonFiles \
			   -I$(SLS_DETECTORS_SW)/slsDetector \
			   -I$(SLS_DETECTORS_SW)/multiSlsDetector \
			   -I$(SLS_DETECTORS_SW)/slsDetectorServer \
			   -I$(SLS_DETECTORS_SW)/slsDetectorAnalysis \
			   -I$(SLS_DETECTORS_SW)/slsReceiverInterface \
			   -I$(SLS_DETECTORS_RECV)/include

CPPFLAGS = $(CT_CPPFLAGS) $(SLS_DETECTORS_INCL_FLAGS)
CXXFLAGS = -I../include $(CT_CXXFLAGS)

slsdetector-objs = SlsDetectorInterface.o \
		   SlsDetectorEiger.o \
		   SlsDetectorCamera.o \
		   SlsDetectorArgs.o \
		   SlsDetectorDefs.o

SRCS = $(slsdetector-objs:.o=.cpp) 

all:	SlsDetector.o

SlsDetector.o:	$(slsdetector-objs)
	$(LD) -o $@ -r $+

clean:
	rm -f *.o *.P

%.o : %.cpp
	$(COMPILE.cpp) $(CPPFLAGS) -MD $(CXXFLAGS) -o $@ $<
	@cp $*.d $*.P; \
	sed -e 's/#.*//' -e 's/^[^:]*: *//' -e 's/ *\\$$//' \
	-e '/^$$/ d' -e 's/$$/ :/' < $*.d >> $*.P; \
	rm -f $*.d

-include $(SRCS:.cpp=.P)

.PHONY: check-syntax
check-syntax:
	$(CXX) $(CPPFLAGS) -Wextra -fsyntax-only $(CXXFLAGS) $(CHK_SOURCES)
