cmake_minimum_required(VERSION 2.8.3)
project(cooperative_driving_networking)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  std_msgs
  cooperative_driving_common
  cooperative_driving_msgs
)

find_package(PCAP REQUIRED)

set(NETWORKING_NODE networking)
set(NETWORKING_NODE_SRC src/${NETWORKING_NODE})

add_service_files(
    FILES
    BroadcastMessage.srv
)

add_message_files(
    FILES
    CooperativeAwarenessMessage.msg
    EmergencyBrake.msg
    Platooning.msg
    StringStamped.msg
    Token.msg
)

generate_messages(  # Generates messages & services
  DEPENDENCIES
  std_msgs
)

# Catkin specific
catkin_package(
  CATKIN_DEPENDS roscpp message_runtime
)

include_directories(
  include
  ${PCAP_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)

add_compile_options(-Wall -Wextra -pedantic)
add_compile_options(-Wno-missing-braces)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_STANDARD_REQUIRED ON)

add_executable(${NETWORKING_NODE}
  ${NETWORKING_NODE_SRC}/transceiver.c
  ${NETWORKING_NODE_SRC}/radiotap.c
  ${NETWORKING_NODE_SRC}/util.c
  ${NETWORKING_NODE_SRC}/wifi_packet.cc
  ${NETWORKING_NODE_SRC}/main.cc)

add_dependencies(${NETWORKING_NODE}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS})

add_custom_target(set_cap ALL
    COMMAND ${CMAKE_CURRENT_LIST_DIR}/scripts/set_capabilities.sh ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_LIB_DESTINATION}/${PROJECT_NAME}/${NETWORKING_NODE}
    DEPENDS ${NETWORKING_NODE}
)

target_link_libraries(${NETWORKING_NODE}
  ${PCAP_LIBRARY}
  ${catkin_LIBRARIES}
 )

set_property(TARGET ${NETWORKING_NODE} PROPERTY CXX_STANDARD 11)
set_property(TARGET ${NETWORKING_NODE} PROPERTY C_STANDARD 11)
set_property(TARGET ${NETWORKING_NODE} PROPERTY CXX_STANDARD_REQUIRED ON)
set_property(TARGET ${NETWORKING_NODE} PROPERTY C_STANDARD_REQUIRED ON)
