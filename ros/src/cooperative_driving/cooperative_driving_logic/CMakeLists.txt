cmake_minimum_required(VERSION 2.8.3)
project(cooperative_driving_logic)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  roslaunch
  nav_msgs
  cooperative_driving_common
  cooperative_driving_networking
  cooperative_driving_vision
  cooperative_driving_msgs
  dynamic_reconfigure)

roslaunch_add_file_check(launch/logic.xml)

# Messages, services and actions
add_message_files(
  FILES
  ReflekteState.msg)

add_service_files(
  FILES
  ChangeState.srv)

generate_messages(  # Generates messages & services
  DEPENDENCIES)

#add dynamic reconfigure api
generate_dynamic_reconfigure_options(
  config/dynamic_parameters.cfg
)

# Catkin specific
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp message_runtime nav_msgs)

add_compile_options(-Wall -Wextra -pedantic)
add_compile_options(-Wno-missing-braces)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Build configuration
include_directories(
  include
  ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME}
  src/reflekte_nodelet.cc
  src/reflekte_state_machine.cc)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 11)
set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD_REQUIRED ON)
