###########################################################################
# This file is part of LImA, a Library for Image Acquisition
#
#  Copyright (C) : 2009-2019
#  European Synchrotron Radiation Facility
#  CS40220 38043 Grenoble Cedex 9
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

cmake_minimum_required(VERSION 3.1)

project(slsdetector)

# Include additional modules that are used inconditionnaly
include(GNUInstallDirs)
include(GenerateExportHeader)

# If conda build, always set lib dir to 'lib'
if(NOT LIMA_ENABLE_NUMA)
  message(FATAL_ERROR
          "slsdetector requires NUMA control in Lima: set LIMA_ENABLE_NUMA ON")
endif()

# If conda build, always set lib dir to 'lib'
if($ENV{CONDA_BUILD})
  set(CMAKE_INSTALL_LIBDIR "lib")
endif()

# Set lower / upper case project names
string(TOUPPER "${PROJECT_NAME}" PROJECT_NAME_UPPER)
string(TOLOWER "${PROJECT_NAME}" PROJECT_NAME_LOWER)

# Set targets export name (used by lima and dependencies)
set(TARGETS_EXPORT_NAME "${PROJECT_NAME_LOWER}-targets")
#set(namespace "${PROJECT_NAME}::")

# Check if project is being used directly or via add_subdirectory
set(CAMERA_MASTER_PROJECT OFF)
if (CMAKE_CURRENT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR)
  set(CAMERA_MASTER_PROJECT ON)
endif()

if (CAMERA_MASTER_PROJECT)
  find_package(Lima REQUIRED COMPONENTS devel tools)
endif()

# CMake additional macros
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake ${LIMA_CMAKE_INCLUDE_DIRS} ${CMAKE_MODULE_PATH})

include(LimaTools)

# Set version
include(project_version)

# Enable python binding code compilation using sip generator
if (CAMERA_MASTER_PROJECT)
  option(LIMA_ENABLE_PYTHON "compile python binding code?" OFF)
  option(LIMA_ENABLE_PYTANGO_SERVER "install python tango server?" OFF)

  if (LIMA_ENABLE_PYTHON)
    limatools_find_python_and_sip()
  endif()
endif()

# Additional packages
find_package(Numa REQUIRED)

# Remove warning on alignment of packet members
SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Waddress-of-packed-member")

# slsDetectorPackage
set(SLS_DETECTOR_DIR ${CMAKE_CURRENT_SOURCE_DIR}/slsDetectorPackage)
set(SLS_DETECTOR_SUPPORT ${SLS_DETECTOR_DIR}/slsSupportLib)
set(SLS_DETECTOR_SW ${SLS_DETECTOR_DIR}/slsDetectorSoftware)
set(SLS_DETECTOR_RECV ${SLS_DETECTOR_DIR}/slsReceiverSoftware)

set(SLS_USE_TEXTCLIENT ON CACHE BOOL "Text Client")
set(SLS_USE_RECEIVER ON CACHE BOOL "Receiver")
set(SLS_USE_GUI OFF CACHE BOOL "GUI")
set(SLS_USE_TESTS ON CACHE BOOL "TESTS")

add_subdirectory(${SLS_DETECTOR_DIR})

file(GLOB_RECURSE SLSDETECTOR_INCS "${CMAKE_CURRENT_SOURCE_DIR}/include/*.h")

add_library(slsdetector SHARED
  src/SlsDetectorDefs.cpp
  src/SlsDetectorFrameMap.cpp
  src/SlsDetectorArgs.cpp
  src/SlsDetectorCPUAffinity.cpp
  src/SlsDetectorBebTools.cpp
  src/SlsDetectorModel.cpp
  src/SlsDetectorReceiver.cpp
  src/SlsDetectorCamera.cpp
  src/SlsDetectorReconstruction.cpp
  src/SlsDetectorBuffer.cpp
  src/SlsDetectorEiger.cpp
  src/SlsDetectorJungfrau.cpp
  src/SlsDetectorInterface.cpp
  ${SLSDETECTOR_INCS}
)

# Additional compile definitions
add_compile_definitions(NO_WARN_X86_INTRINSICS)

target_compile_options(slsdetector PRIVATE "-Wno-address-of-packed-member")

# Generate export macros
generate_export_header(slsdetector)

# Set version and output name
set_target_properties(slsdetector PROPERTIES
  OUTPUT_NAME "lima${PROJECT_NAME_LOWER}"
  VERSION "${PROJECT_VERSION}"
  SOVERSION "${PACKAGE_VERSION_MAJOR}.${PACKAGE_VERSION_MINOR}")

# Add dependencies
target_include_directories(slsdetector
    PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    PUBLIC "$<BUILD_INTERFACE:${SLS_DETECTOR_SUPPORT}/include>"
    PUBLIC "$<BUILD_INTERFACE:${SLS_DETECTOR_SW}/include>"
    PUBLIC "$<BUILD_INTERFACE:${SLS_DETECTOR_RECV}/include>"
    PUBLIC "$<BUILD_INTERFACE:${NUMA_INCLUDE_DIR}>"
    PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>"           # For export header
    PUBLIC "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

message("NUMA_LIBRARY=" ${NUMA_LIBRARY})

target_link_libraries(slsdetector
  PUBLIC limacore
  PUBLIC slsDetectorShared slsReceiverShared
  PUBLIC ${NUMA_LIBRARY}
)

# Binding code for python
if(LIMA_ENABLE_PYTHON)
  limatools_run_sip_for_camera(slsdetector)
endif()

# Generate and install package config file and version
if(CAMERA_MASTER_PROJECT)
  set (PROJECT_LIBRARIES slsdetector)
  set(SIP_INSTALL_DIR ${CMAKE_INSTALL_DATADIR}/sip/lima)
  set(CMAKE_INSTALL_DIR ${CMAKE_INSTALL_DATADIR}/cmake/lima)
  include(package_config)
endif()

## Installation

install(
  TARGETS slsdetector
  EXPORT "${TARGETS_EXPORT_NAME}"
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}   # import library
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}   # .so files are libraries
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}   # .dll files are binaries
  INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}  # this does not actually install anything (but used by downstream projects)
)

install(
  DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/
  COMPONENT devel
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
  FILES_MATCHING PATTERN "*.h"
)

install(
  FILES
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}/project_version.h
    ${PROJECT_BINARY_DIR}/slsdetector_export.h
  COMPONENT devel
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

if(LIMA_ENABLE_PYTHON)
  install(
    DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/python/
    DESTINATION "${PYTHON_SITE_PACKAGES_DIR}/Lima/SlsDetector"
    )
  if (LIMA_ENABLE_PYTANGO_SERVER)
    add_subdirectory(tango)
  endif()
endif()

## Tests
if(CAMERA_ENABLE_TESTS)
    enable_testing()
    add_subdirectory(test)
endif()
