# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "cooperative_driving_vision: 3 messages, 0 services")

set(MSG_I_FLAGS "-Icooperative_driving_vision:/home/amjad/Desktop/pg2017w/ros/src/cooperative_driving/cooperative_driving_vision/msg;-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/kinetic/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(cooperative_driving_vision_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/amjad/Desktop/pg2017w/ros/src/cooperative_driving/cooperative_driving_vision/msg/Features.msg" NAME_WE)
add_custom_target(_cooperative_driving_vision_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "cooperative_driving_vision" "/home/amjad/Desktop/pg2017w/ros/src/cooperative_driving/cooperative_driving_vision/msg/Features.msg" "std_msgs/ColorRGBA:cooperative_driving_vision/Moment:cooperative_driving_vision/Region:std_msgs/Header:geometry_msgs/Point"
)

get_filename_component(_filename "/home/amjad/Desktop/pg2017w/ros/src/cooperative_driving/cooperative_driving_vision/msg/Region.msg" NAME_WE)
add_custom_target(_cooperative_driving_vision_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "cooperative_driving_vision" "/home/amjad/Desktop/pg2017w/ros/src/cooperative_driving/cooperative_driving_vision/msg/Region.msg" "std_msgs/ColorRGBA:cooperative_driving_vision/Moment"
)

get_filename_component(_filename "/home/amjad/Desktop/pg2017w/ros/src/cooperative_driving/cooperative_driving_vision/msg/Moment.msg" NAME_WE)
add_custom_target(_cooperative_driving_vision_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "cooperative_driving_vision" "/home/amjad/Desktop/pg2017w/ros/src/cooperative_driving/cooperative_driving_vision/msg/Moment.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(cooperative_driving_vision
  "/home/amjad/Desktop/pg2017w/ros/src/cooperative_driving/cooperative_driving_vision/msg/Features.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/ColorRGBA.msg;/home/amjad/Desktop/pg2017w/ros/src/cooperative_driving/cooperative_driving_vision/msg/Moment.msg;/home/amjad/Desktop/pg2017w/ros/src/cooperative_driving/cooperative_driving_vision/msg/Region.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/cooperative_driving_vision
)
_generate_msg_cpp(cooperative_driving_vision
  "/home/amjad/Desktop/pg2017w/ros/src/cooperative_driving/cooperative_driving_vision/msg/Region.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/ColorRGBA.msg;/home/amjad/Desktop/pg2017w/ros/src/cooperative_driving/cooperative_driving_vision/msg/Moment.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/cooperative_driving_vision
)
_generate_msg_cpp(cooperative_driving_vision
  "/home/amjad/Desktop/pg2017w/ros/src/cooperative_driving/cooperative_driving_vision/msg/Moment.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/cooperative_driving_vision
)

### Generating Services

### Generating Module File
_generate_module_cpp(cooperative_driving_vision
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/cooperative_driving_vision
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(cooperative_driving_vision_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(cooperative_driving_vision_generate_messages cooperative_driving_vision_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/amjad/Desktop/pg2017w/ros/src/cooperative_driving/cooperative_driving_vision/msg/Features.msg" NAME_WE)
add_dependencies(cooperative_driving_vision_generate_messages_cpp _cooperative_driving_vision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amjad/Desktop/pg2017w/ros/src/cooperative_driving/cooperative_driving_vision/msg/Region.msg" NAME_WE)
add_dependencies(cooperative_driving_vision_generate_messages_cpp _cooperative_driving_vision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amjad/Desktop/pg2017w/ros/src/cooperative_driving/cooperative_driving_vision/msg/Moment.msg" NAME_WE)
add_dependencies(cooperative_driving_vision_generate_messages_cpp _cooperative_driving_vision_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(cooperative_driving_vision_gencpp)
add_dependencies(cooperative_driving_vision_gencpp cooperative_driving_vision_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS cooperative_driving_vision_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(cooperative_driving_vision
  "/home/amjad/Desktop/pg2017w/ros/src/cooperative_driving/cooperative_driving_vision/msg/Features.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/ColorRGBA.msg;/home/amjad/Desktop/pg2017w/ros/src/cooperative_driving/cooperative_driving_vision/msg/Moment.msg;/home/amjad/Desktop/pg2017w/ros/src/cooperative_driving/cooperative_driving_vision/msg/Region.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/cooperative_driving_vision
)
_generate_msg_eus(cooperative_driving_vision
  "/home/amjad/Desktop/pg2017w/ros/src/cooperative_driving/cooperative_driving_vision/msg/Region.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/ColorRGBA.msg;/home/amjad/Desktop/pg2017w/ros/src/cooperative_driving/cooperative_driving_vision/msg/Moment.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/cooperative_driving_vision
)
_generate_msg_eus(cooperative_driving_vision
  "/home/amjad/Desktop/pg2017w/ros/src/cooperative_driving/cooperative_driving_vision/msg/Moment.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/cooperative_driving_vision
)

### Generating Services

### Generating Module File
_generate_module_eus(cooperative_driving_vision
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/cooperative_driving_vision
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(cooperative_driving_vision_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(cooperative_driving_vision_generate_messages cooperative_driving_vision_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/amjad/Desktop/pg2017w/ros/src/cooperative_driving/cooperative_driving_vision/msg/Features.msg" NAME_WE)
add_dependencies(cooperative_driving_vision_generate_messages_eus _cooperative_driving_vision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amjad/Desktop/pg2017w/ros/src/cooperative_driving/cooperative_driving_vision/msg/Region.msg" NAME_WE)
add_dependencies(cooperative_driving_vision_generate_messages_eus _cooperative_driving_vision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amjad/Desktop/pg2017w/ros/src/cooperative_driving/cooperative_driving_vision/msg/Moment.msg" NAME_WE)
add_dependencies(cooperative_driving_vision_generate_messages_eus _cooperative_driving_vision_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(cooperative_driving_vision_geneus)
add_dependencies(cooperative_driving_vision_geneus cooperative_driving_vision_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS cooperative_driving_vision_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(cooperative_driving_vision
  "/home/amjad/Desktop/pg2017w/ros/src/cooperative_driving/cooperative_driving_vision/msg/Features.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/ColorRGBA.msg;/home/amjad/Desktop/pg2017w/ros/src/cooperative_driving/cooperative_driving_vision/msg/Moment.msg;/home/amjad/Desktop/pg2017w/ros/src/cooperative_driving/cooperative_driving_vision/msg/Region.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/cooperative_driving_vision
)
_generate_msg_lisp(cooperative_driving_vision
  "/home/amjad/Desktop/pg2017w/ros/src/cooperative_driving/cooperative_driving_vision/msg/Region.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/ColorRGBA.msg;/home/amjad/Desktop/pg2017w/ros/src/cooperative_driving/cooperative_driving_vision/msg/Moment.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/cooperative_driving_vision
)
_generate_msg_lisp(cooperative_driving_vision
  "/home/amjad/Desktop/pg2017w/ros/src/cooperative_driving/cooperative_driving_vision/msg/Moment.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/cooperative_driving_vision
)

### Generating Services

### Generating Module File
_generate_module_lisp(cooperative_driving_vision
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/cooperative_driving_vision
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(cooperative_driving_vision_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(cooperative_driving_vision_generate_messages cooperative_driving_vision_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/amjad/Desktop/pg2017w/ros/src/cooperative_driving/cooperative_driving_vision/msg/Features.msg" NAME_WE)
add_dependencies(cooperative_driving_vision_generate_messages_lisp _cooperative_driving_vision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amjad/Desktop/pg2017w/ros/src/cooperative_driving/cooperative_driving_vision/msg/Region.msg" NAME_WE)
add_dependencies(cooperative_driving_vision_generate_messages_lisp _cooperative_driving_vision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amjad/Desktop/pg2017w/ros/src/cooperative_driving/cooperative_driving_vision/msg/Moment.msg" NAME_WE)
add_dependencies(cooperative_driving_vision_generate_messages_lisp _cooperative_driving_vision_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(cooperative_driving_vision_genlisp)
add_dependencies(cooperative_driving_vision_genlisp cooperative_driving_vision_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS cooperative_driving_vision_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(cooperative_driving_vision
  "/home/amjad/Desktop/pg2017w/ros/src/cooperative_driving/cooperative_driving_vision/msg/Features.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/ColorRGBA.msg;/home/amjad/Desktop/pg2017w/ros/src/cooperative_driving/cooperative_driving_vision/msg/Moment.msg;/home/amjad/Desktop/pg2017w/ros/src/cooperative_driving/cooperative_driving_vision/msg/Region.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/cooperative_driving_vision
)
_generate_msg_nodejs(cooperative_driving_vision
  "/home/amjad/Desktop/pg2017w/ros/src/cooperative_driving/cooperative_driving_vision/msg/Region.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/ColorRGBA.msg;/home/amjad/Desktop/pg2017w/ros/src/cooperative_driving/cooperative_driving_vision/msg/Moment.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/cooperative_driving_vision
)
_generate_msg_nodejs(cooperative_driving_vision
  "/home/amjad/Desktop/pg2017w/ros/src/cooperative_driving/cooperative_driving_vision/msg/Moment.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/cooperative_driving_vision
)

### Generating Services

### Generating Module File
_generate_module_nodejs(cooperative_driving_vision
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/cooperative_driving_vision
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(cooperative_driving_vision_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(cooperative_driving_vision_generate_messages cooperative_driving_vision_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/amjad/Desktop/pg2017w/ros/src/cooperative_driving/cooperative_driving_vision/msg/Features.msg" NAME_WE)
add_dependencies(cooperative_driving_vision_generate_messages_nodejs _cooperative_driving_vision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amjad/Desktop/pg2017w/ros/src/cooperative_driving/cooperative_driving_vision/msg/Region.msg" NAME_WE)
add_dependencies(cooperative_driving_vision_generate_messages_nodejs _cooperative_driving_vision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amjad/Desktop/pg2017w/ros/src/cooperative_driving/cooperative_driving_vision/msg/Moment.msg" NAME_WE)
add_dependencies(cooperative_driving_vision_generate_messages_nodejs _cooperative_driving_vision_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(cooperative_driving_vision_gennodejs)
add_dependencies(cooperative_driving_vision_gennodejs cooperative_driving_vision_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS cooperative_driving_vision_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(cooperative_driving_vision
  "/home/amjad/Desktop/pg2017w/ros/src/cooperative_driving/cooperative_driving_vision/msg/Features.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/ColorRGBA.msg;/home/amjad/Desktop/pg2017w/ros/src/cooperative_driving/cooperative_driving_vision/msg/Moment.msg;/home/amjad/Desktop/pg2017w/ros/src/cooperative_driving/cooperative_driving_vision/msg/Region.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/cooperative_driving_vision
)
_generate_msg_py(cooperative_driving_vision
  "/home/amjad/Desktop/pg2017w/ros/src/cooperative_driving/cooperative_driving_vision/msg/Region.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/ColorRGBA.msg;/home/amjad/Desktop/pg2017w/ros/src/cooperative_driving/cooperative_driving_vision/msg/Moment.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/cooperative_driving_vision
)
_generate_msg_py(cooperative_driving_vision
  "/home/amjad/Desktop/pg2017w/ros/src/cooperative_driving/cooperative_driving_vision/msg/Moment.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/cooperative_driving_vision
)

### Generating Services

### Generating Module File
_generate_module_py(cooperative_driving_vision
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/cooperative_driving_vision
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(cooperative_driving_vision_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(cooperative_driving_vision_generate_messages cooperative_driving_vision_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/amjad/Desktop/pg2017w/ros/src/cooperative_driving/cooperative_driving_vision/msg/Features.msg" NAME_WE)
add_dependencies(cooperative_driving_vision_generate_messages_py _cooperative_driving_vision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amjad/Desktop/pg2017w/ros/src/cooperative_driving/cooperative_driving_vision/msg/Region.msg" NAME_WE)
add_dependencies(cooperative_driving_vision_generate_messages_py _cooperative_driving_vision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amjad/Desktop/pg2017w/ros/src/cooperative_driving/cooperative_driving_vision/msg/Moment.msg" NAME_WE)
add_dependencies(cooperative_driving_vision_generate_messages_py _cooperative_driving_vision_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(cooperative_driving_vision_genpy)
add_dependencies(cooperative_driving_vision_genpy cooperative_driving_vision_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS cooperative_driving_vision_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/cooperative_driving_vision)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/cooperative_driving_vision
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(cooperative_driving_vision_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(cooperative_driving_vision_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/cooperative_driving_vision)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/cooperative_driving_vision
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(cooperative_driving_vision_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(cooperative_driving_vision_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/cooperative_driving_vision)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/cooperative_driving_vision
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(cooperative_driving_vision_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(cooperative_driving_vision_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/cooperative_driving_vision)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/cooperative_driving_vision
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(cooperative_driving_vision_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(cooperative_driving_vision_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/cooperative_driving_vision)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/cooperative_driving_vision\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/cooperative_driving_vision
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(cooperative_driving_vision_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(cooperative_driving_vision_generate_messages_py geometry_msgs_generate_messages_py)
endif()
