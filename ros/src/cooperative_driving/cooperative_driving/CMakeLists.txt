cmake_minimum_required(VERSION 2.8.3)
project(cooperative_driving)

find_package(catkin REQUIRED COMPONENTS
  roslaunch)

roslaunch_add_file_check(launch/demo_raspi.launch)
roslaunch_add_file_check(launch/demo_pc.launch)
roslaunch_add_file_check(launch/robot_raspi.launch)
roslaunch_add_file_check(launch/robot_pc.launch)

# Catkin specific
catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS
)
