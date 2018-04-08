# Install script for directory: /home/amjad/Desktop/pg2017w/ros/src/cooperative_driving/cooperative_driving_application

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/amjad/Desktop/pg2017w/ros/install")
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

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  include("/home/amjad/Desktop/pg2017w/ros/build/cooperative_driving/cooperative_driving_application/catkin_generated/safe_execute_install.cmake")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/cooperative_driving_application" TYPE FILE FILES "/home/amjad/Desktop/pg2017w/ros/devel/include/cooperative_driving_application/dynamic_parametersConfig.h")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  execute_process(COMMAND "/usr/bin/python" -m compileall "/home/amjad/Desktop/pg2017w/ros/devel/lib/python2.7/dist-packages/cooperative_driving_application/cfg")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages/cooperative_driving_application" TYPE DIRECTORY FILES "/home/amjad/Desktop/pg2017w/ros/devel/lib/python2.7/dist-packages/cooperative_driving_application/cfg")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/amjad/Desktop/pg2017w/ros/build/cooperative_driving/cooperative_driving_application/catkin_generated/installspace/cooperative_driving_application.pc")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/cooperative_driving_application/cmake" TYPE FILE FILES
    "/home/amjad/Desktop/pg2017w/ros/build/cooperative_driving/cooperative_driving_application/catkin_generated/installspace/cooperative_driving_applicationConfig.cmake"
    "/home/amjad/Desktop/pg2017w/ros/build/cooperative_driving/cooperative_driving_application/catkin_generated/installspace/cooperative_driving_applicationConfig-version.cmake"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/cooperative_driving_application" TYPE FILE FILES "/home/amjad/Desktop/pg2017w/ros/src/cooperative_driving/cooperative_driving_application/package.xml")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/amjad/Desktop/pg2017w/ros/build/cooperative_driving/cooperative_driving_application/test/cmake_install.cmake")

endif()

