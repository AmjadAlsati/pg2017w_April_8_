cmake_minimum_required(VERSION 2.8.3)
project(cooperative_driving_common)

find_package(catkin REQUIRED COMPONENTS)

# Catkin specific
catkin_package(
  INCLUDE_DIRS include)

catkin_python_setup()

if (CATKIN_ENABLE_TESTING)
  find_package(catkin COMPONENTS rostest roscpp geometry_msgs)
  include_directories(include ${catkin_INCLUDE_DIRS})
  add_subdirectory(test)
endif (CATKIN_ENABLE_TESTING)
