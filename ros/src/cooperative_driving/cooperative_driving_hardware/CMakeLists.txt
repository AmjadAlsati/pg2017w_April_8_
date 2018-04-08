cmake_minimum_required(VERSION 2.8.3)
project(cooperative_driving_hardware)

find_package(catkin REQUIRED COMPONENTS
  controller_interface
  controller_manager
  hardware_interface
  message_generation
  roscpp
  roslaunch
  cooperative_driving_common
  cooperative_driving_networking
)

roslaunch_add_file_check(launch/distance-controller.xml)
roslaunch_add_file_check(launch/drive-controller.xml)
roslaunch_add_file_check(launch/hardware_arm.xml)
roslaunch_add_file_check(launch/hardware_x86_64.xml)
roslaunch_add_file_check(launch/led-controller-pi.xml)

add_message_files(
  FILES
  DistanceValue.msg
  DistanceValues.msg  
  LedValue.msg
  LedValues.msg
  LedCommand.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

# Catkin specific
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp message_runtime
)

add_compile_options(-Wall -Wextra -pedantic)
add_compile_options(-Wno-missing-braces)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Build configuration
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/ccs_robot.cc
  src/ccs_robot_nodelet.cc
  src/distance_sensor_controller.cc
  src/led_controller.cc
)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 11)
set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD_REQUIRED ON)
