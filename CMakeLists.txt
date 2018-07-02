###########################################################################
# This file is part of LImA, a Library for Image Acquisition
#
#  Copyright (C) : 2009-2017
#  European Synchrotron Radiation Facility
#  BP 220, Grenoble 38043
#  FRANCE
# 
#  Contact: lima@esrf.fr
# 
#  This is free software; you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation; either version 3 of the License, or
#  (at your option) any later version.
# 
#  This software is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
# 
#  You should have received a copy of the GNU General Public License
#  along with this program; if not, see <http://www.gnu.org/licenses/>.
############################################################################
set(NAME slsdetector)

set(SLS_DETECTOR_DIR slsDetectorPackage)
set(SLS_DETECTOR_SW ${SLS_DETECTOR_DIR}/slsDetectorSoftware)
set(SLS_DETECTOR_RECV ${SLS_DETECTOR_DIR}/slsReceiverSoftware)

option (USE_TEXTCLIENT "Text Client" ON)
option (USE_RECEIVER "Receiver" ON)
option (USE_GUI "GUI" ON)

add_subdirectory(${SLS_DETECTOR_DIR})

set(${NAME}_srcs src/SlsDetectorDefs.cpp
		 src/SlsDetectorArgs.cpp
		 src/SlsDetectorCPUAffinity.cpp
		 src/SlsDetectorModel.cpp
		 src/SlsDetectorReceiver.cpp
		 src/SlsDetectorCamera.cpp
		 src/SlsDetectorEiger.cpp
		 src/SlsDetectorInterface.cpp)

add_library(lima${NAME} SHARED ${${NAME}_srcs})

target_include_directories(lima${NAME} PUBLIC 
			   include
			   ${SLS_DETECTOR_SW}/include
			   ${SLS_DETECTOR_SW}/commonFiles
			   ${SLS_DETECTOR_SW}/slsDetector
			   ${SLS_DETECTOR_SW}/multiSlsDetector
			   ${SLS_DETECTOR_SW}/slsDetectorServer
			   ${SLS_DETECTOR_SW}/slsDetectorAnalysis
			   ${SLS_DETECTOR_SW}/slsReceiverInterface
			   ${SLS_DETECTOR_RECV}/include)

target_link_libraries(lima${NAME}
		      limacore
		      slsDetectorShared slsReceiverShared zmq numa)

limatools_set_library_soversion(lima${NAME} VERSION)
install(TARGETS lima${NAME} LIBRARY DESTINATION lib)

if(LIMA_ENABLE_PYTHON)
    limatools_run_sip_for_camera(${NAME})
    install(FILES python/__init__.py DESTINATION 
            ${PYTHON_SITE_PACKAGES_DIR}/Lima/SlsDetector)
endif()

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake ${CMAKE_MODULE_PATH})

if(LIMA_ENABLE_TESTS)
	add_subdirectory(test/)
endif()
