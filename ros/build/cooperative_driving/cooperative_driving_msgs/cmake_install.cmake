# Install script for directory: /home/amjad/Desktop/pg2017w/ros/src/cooperative_driving/cooperative_driving_msgs

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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/cooperative_driving_msgs/msg" TYPE FILE FILES
    "/home/amjad/Desktop/pg2017w/ros/src/cooperative_driving/cooperative_driving_msgs/msg/Direction.msg"
    "/home/amjad/Desktop/pg2017w/ros/src/cooperative_driving/cooperative_driving_msgs/msg/Directions.msg"
    "/home/amjad/Desktop/pg2017w/ros/src/cooperative_driving/cooperative_driving_msgs/msg/Command.msg"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/cooperative_driving_msgs/cmake" TYPE FILE FILES "/home/amjad/Desktop/pg2017w/ros/build/cooperative_driving/cooperative_driving_msgs/catkin_generated/installspace/cooperative_driving_msgs-msg-paths.cmake")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/amjad/Desktop/pg2017w/ros/devel/include/cooperative_driving_msgs")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/amjad/Desktop/pg2017w/ros/devel/share/roseus/ros/cooperative_driving_msgs")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/amjad/Desktop/pg2017w/ros/devel/share/common-lisp/ros/cooperative_driving_msgs")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/amjad/Desktop/pg2017w/ros/devel/share/gennodejs/ros/cooperative_driving_msgs")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  execute_process(COMMAND "/usr/bin/python" -m compileall "/home/amjad/Desktop/pg2017w/ros/devel/lib/python2.7/dist-packages/cooperative_driving_msgs")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/home/amjad/Desktop/pg2017w/ros/devel/lib/python2.7/dist-packages/cooperative_driving_msgs")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/amjad/Desktop/pg2017w/ros/build/cooperative_driving/cooperative_driving_msgs/catkin_generated/installspace/cooperative_driving_msgs.pc")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/cooperative_driving_msgs/cmake" TYPE FILE FILES "/home/amjad/Desktop/pg2017w/ros/build/cooperative_driving/cooperative_driving_msgs/catkin_generated/installspace/cooperative_driving_msgs-msg-extras.cmake")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/cooperative_driving_msgs/cmake" TYPE FILE FILES
    "/home/amjad/Desktop/pg2017w/ros/build/cooperative_driving/cooperative_driving_msgs/catkin_generated/installspace/cooperative_driving_msgsConfig.cmake"
    "/home/amjad/Desktop/pg2017w/ros/build/cooperative_driving/cooperative_driving_msgs/catkin_generated/installspace/cooperative_driving_msgsConfig-version.cmake"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/cooperative_driving_msgs" TYPE FILE FILES "/home/amjad/Desktop/pg2017w/ros/src/cooperative_driving/cooperative_driving_msgs/package.xml")
endif()

