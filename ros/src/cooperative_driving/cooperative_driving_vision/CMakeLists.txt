cmake_minimum_required(VERSION 2.8.3)
project(cooperative_driving_vision)

find_package(catkin REQUIRED COMPONENTS
  nodelet
  message_generation
  roscpp
  roslaunch
  sensor_msgs
  cv_bridge
  image_geometry
  image_transport
  tf
  dynamic_reconfigure
  cooperative_driving_common)

find_package(OpenCV REQUIRED) 

roslaunch_add_file_check(launch/feature_extraction.xml)

# Messages, services and actions
add_message_files(
  FILES
  Features.msg
  Region.msg
  Moment.msg
)

generate_messages(  # Generates messages & services
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

generate_dynamic_reconfigure_options(  # Generate dynamic reconfigure API
  config/dynamic_parameters.cfg)

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
  ${OpenCV_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/feature_extractor.cc
  src/feature_extractor_nodelet.cc)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(feature_image_renderer
  src/feature_image_renderer.cc)
add_dependencies(feature_image_renderer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(feature_image_renderer ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

if(CMAKE_BUILD_TYPE MATCHES "^Debug$")
  add_executable(feature_extraction_debugger
    src/feature_extraction_debugger.cc)
  add_dependencies(feature_extraction_debugger ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  target_link_libraries(feature_extraction_debugger ${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
endif()
