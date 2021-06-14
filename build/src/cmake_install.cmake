# Install script for directory: /home/angel/Documents/02_master_thesis/modularslam/src

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY FILES "/home/angel/Documents/02_master_thesis/modularslam/build/src/libmodularslam.a")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/src" TYPE FILE FILES
    "/home/angel/Documents/02_master_thesis/modularslam/src/Transform.h"
    "/home/angel/Documents/02_master_thesis/modularslam/src/LandMark.h"
    "/home/angel/Documents/02_master_thesis/modularslam/src/WorldModelInterface.h"
    "/home/angel/Documents/02_master_thesis/modularslam/src/PoseFactor.h"
    "/home/angel/Documents/02_master_thesis/modularslam/src/RefFrame.h"
    "/home/angel/Documents/02_master_thesis/modularslam/src/WorldModelPlotter.h"
    "/home/angel/Documents/02_master_thesis/modularslam/src/PlotDataSet.h"
    "/home/angel/Documents/02_master_thesis/modularslam/src/SFPlot.h"
    "/home/angel/Documents/02_master_thesis/modularslam/src/WorldModel.h"
    "/home/angel/Documents/02_master_thesis/modularslam/src/KeyFrame.h"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/src" TYPE FILE FILES "/home/angel/Documents/02_master_thesis/modularslam/build/generated_headers/src//version.h")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/modularslam" TYPE FILE FILES
    "/home/angel/Documents/02_master_thesis/modularslam/build/generated/modularslamConfig.cmake"
    "/home/angel/Documents/02_master_thesis/modularslam/build/generated/modularslamConfigVersion.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/modularslam/modularslamTargets.cmake")
    file(DIFFERENT EXPORT_FILE_CHANGED FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/modularslam/modularslamTargets.cmake"
         "/home/angel/Documents/02_master_thesis/modularslam/build/src/CMakeFiles/Export/lib/cmake/modularslam/modularslamTargets.cmake")
    if(EXPORT_FILE_CHANGED)
      file(GLOB OLD_CONFIG_FILES "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/modularslam/modularslamTargets-*.cmake")
      if(OLD_CONFIG_FILES)
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/modularslam/modularslamTargets.cmake\" will be replaced.  Removing files [${OLD_CONFIG_FILES}].")
        file(REMOVE ${OLD_CONFIG_FILES})
      endif()
    endif()
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/modularslam" TYPE FILE FILES "/home/angel/Documents/02_master_thesis/modularslam/build/src/CMakeFiles/Export/lib/cmake/modularslam/modularslamTargets.cmake")
  if("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Rr][Ee][Ll][Ee][Aa][Ss][Ee])$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/modularslam" TYPE FILE FILES "/home/angel/Documents/02_master_thesis/modularslam/build/src/CMakeFiles/Export/lib/cmake/modularslam/modularslamTargets-release.cmake")
  endif()
endif()

