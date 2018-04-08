# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "cooperative_driving_networking: 5 messages, 1 services")

set(MSG_I_FLAGS "-Icooperative_driving_networking:/home/amjad/Desktop/pg2017w/ros/src/cooperative_driving/cooperative_driving_networking/msg;-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(cooperative_driving_networking_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/amjad/Desktop/pg2017w/ros/src/cooperative_driving/cooperative_driving_networking/srv/BroadcastMessage.srv" NAME_WE)
add_custom_target(_cooperative_driving_networking_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "cooperative_driving_networking" "/home/amjad/Desktop/pg2017w/ros/src/cooperative_driving/cooperative_driving_networking/srv/BroadcastMessage.srv" ""
)

get_filename_component(_filename "/home/amjad/Desktop/pg2017w/ros/src/cooperative_driving/cooperative_driving_networking/msg/Token.msg" NAME_WE)
add_custom_target(_cooperative_driving_networking_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "cooperative_driving_networking" "/home/amjad/Desktop/pg2017w/ros/src/cooperative_driving/cooperative_driving_networking/msg/Token.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/amjad/Desktop/pg2017w/ros/src/cooperative_driving/cooperative_driving_networking/msg/EmergencyBrake.msg" NAME_WE)
add_custom_target(_cooperative_driving_networking_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "cooperative_driving_networking" "/home/amjad/Desktop/pg2017w/ros/src/cooperative_driving/cooperative_driving_networking/msg/EmergencyBrake.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/amjad/Desktop/pg2017w/ros/src/cooperative_driving/cooperative_driving_networking/msg/StringStamped.msg" NAME_WE)
add_custom_target(_cooperative_driving_networking_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "cooperative_driving_networking" "/home/amjad/Desktop/pg2017w/ros/src/cooperative_driving/cooperative_driving_networking/msg/StringStamped.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/amjad/Desktop/pg2017w/ros/src/cooperative_driving/cooperative_driving_networking/msg/CooperativeAwarenessMessage.msg" NAME_WE)
add_custom_target(_cooperative_driving_networking_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "cooperative_driving_networking" "/home/amjad/Desktop/pg2017w/ros/src/cooperative_driving/cooperative_driving_networking/msg/CooperativeAwarenessMessage.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/amjad/Desktop/pg2017w/ros/src/cooperative_driving/cooperative_driving_networking/msg/Platooning.msg" NAME_WE)
add_custom_target(_cooperative_driving_networking_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "cooperative_driving_networking" "/home/amjad/Desktop/pg2017w/ros/src/cooperative_driving/cooperative_driving_networking/msg/Platooning.msg" "std_msgs/Header"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(cooperative_driving_networking
  "/home/amjad/Desktop/pg2017w/ros/src/cooperative_driving/cooperative_driving_networking/msg/CooperativeAwarenessMessage.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/cooperative_driving_networking
)
_generate_msg_cpp(cooperative_driving_networking
  "/home/amjad/Desktop/pg2017w/ros/src/cooperative_driving/cooperative_driving_networking/msg/EmergencyBrake.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/cooperative_driving_networking
)
_generate_msg_cpp(cooperative_driving_networking
  "/home/amjad/Desktop/pg2017w/ros/src/cooperative_driving/cooperative_driving_networking/msg/Token.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/cooperative_driving_networking
)
_generate_msg_cpp(cooperative_driving_networking
  "/home/amjad/Desktop/pg2017w/ros/src/cooperative_driving/cooperative_driving_networking/msg/StringStamped.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/cooperative_driving_networking
)
_generate_msg_cpp(cooperative_driving_networking
  "/home/amjad/Desktop/pg2017w/ros/src/cooperative_driving/cooperative_driving_networking/msg/Platooning.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/cooperative_driving_networking
)

### Generating Services
_generate_srv_cpp(cooperative_driving_networking
  "/home/amjad/Desktop/pg2017w/ros/src/cooperative_driving/cooperative_driving_networking/srv/BroadcastMessage.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/cooperative_driving_networking
)

### Generating Module File
_generate_module_cpp(cooperative_driving_networking
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/cooperative_driving_networking
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(cooperative_driving_networking_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(cooperative_driving_networking_generate_messages cooperative_driving_networking_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/amjad/Desktop/pg2017w/ros/src/cooperative_driving/cooperative_driving_networking/srv/BroadcastMessage.srv" NAME_WE)
add_dependencies(cooperative_driving_networking_generate_messages_cpp _cooperative_driving_networking_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amjad/Desktop/pg2017w/ros/src/cooperative_driving/cooperative_driving_networking/msg/Token.msg" NAME_WE)
add_dependencies(cooperative_driving_networking_generate_messages_cpp _cooperative_driving_networking_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amjad/Desktop/pg2017w/ros/src/cooperative_driving/cooperative_driving_networking/msg/EmergencyBrake.msg" NAME_WE)
add_dependencies(cooperative_driving_networking_generate_messages_cpp _cooperative_driving_networking_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amjad/Desktop/pg2017w/ros/src/cooperative_driving/cooperative_driving_networking/msg/StringStamped.msg" NAME_WE)
add_dependencies(cooperative_driving_networking_generate_messages_cpp _cooperative_driving_networking_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amjad/Desktop/pg2017w/ros/src/cooperative_driving/cooperative_driving_networking/msg/CooperativeAwarenessMessage.msg" NAME_WE)
add_dependencies(cooperative_driving_networking_generate_messages_cpp _cooperative_driving_networking_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amjad/Desktop/pg2017w/ros/src/cooperative_driving/cooperative_driving_networking/msg/Platooning.msg" NAME_WE)
add_dependencies(cooperative_driving_networking_generate_messages_cpp _cooperative_driving_networking_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(cooperative_driving_networking_gencpp)
add_dependencies(cooperative_driving_networking_gencpp cooperative_driving_networking_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS cooperative_driving_networking_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(cooperative_driving_networking
  "/home/amjad/Desktop/pg2017w/ros/src/cooperative_driving/cooperative_driving_networking/msg/CooperativeAwarenessMessage.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/cooperative_driving_networking
)
_generate_msg_eus(cooperative_driving_networking
  "/home/amjad/Desktop/pg2017w/ros/src/cooperative_driving/cooperative_driving_networking/msg/EmergencyBrake.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/cooperative_driving_networking
)
_generate_msg_eus(cooperative_driving_networking
  "/home/amjad/Desktop/pg2017w/ros/src/cooperative_driving/cooperative_driving_networking/msg/Token.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/cooperative_driving_networking
)
_generate_msg_eus(cooperative_driving_networking
  "/home/amjad/Desktop/pg2017w/ros/src/cooperative_driving/cooperative_driving_networking/msg/StringStamped.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/cooperative_driving_networking
)
_generate_msg_eus(cooperative_driving_networking
  "/home/amjad/Desktop/pg2017w/ros/src/cooperative_driving/cooperative_driving_networking/msg/Platooning.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/cooperative_driving_networking
)

### Generating Services
_generate_srv_eus(cooperative_driving_networking
  "/home/amjad/Desktop/pg2017w/ros/src/cooperative_driving/cooperative_driving_networking/srv/BroadcastMessage.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/cooperative_driving_networking
)

### Generating Module File
_generate_module_eus(cooperative_driving_networking
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/cooperative_driving_networking
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(cooperative_driving_networking_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(cooperative_driving_networking_generate_messages cooperative_driving_networking_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/amjad/Desktop/pg2017w/ros/src/cooperative_driving/cooperative_driving_networking/srv/BroadcastMessage.srv" NAME_WE)
add_dependencies(cooperative_driving_networking_generate_messages_eus _cooperative_driving_networking_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amjad/Desktop/pg2017w/ros/src/cooperative_driving/cooperative_driving_networking/msg/Token.msg" NAME_WE)
add_dependencies(cooperative_driving_networking_generate_messages_eus _cooperative_driving_networking_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amjad/Desktop/pg2017w/ros/src/cooperative_driving/cooperative_driving_networking/msg/EmergencyBrake.msg" NAME_WE)
add_dependencies(cooperative_driving_networking_generate_messages_eus _cooperative_driving_networking_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amjad/Desktop/pg2017w/ros/src/cooperative_driving/cooperative_driving_networking/msg/StringStamped.msg" NAME_WE)
add_dependencies(cooperative_driving_networking_generate_messages_eus _cooperative_driving_networking_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amjad/Desktop/pg2017w/ros/src/cooperative_driving/cooperative_driving_networking/msg/CooperativeAwarenessMessage.msg" NAME_WE)
add_dependencies(cooperative_driving_networking_generate_messages_eus _cooperative_driving_networking_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amjad/Desktop/pg2017w/ros/src/cooperative_driving/cooperative_driving_networking/msg/Platooning.msg" NAME_WE)
add_dependencies(cooperative_driving_networking_generate_messages_eus _cooperative_driving_networking_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(cooperative_driving_networking_geneus)
add_dependencies(cooperative_driving_networking_geneus cooperative_driving_networking_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS cooperative_driving_networking_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(cooperative_driving_networking
  "/home/amjad/Desktop/pg2017w/ros/src/cooperative_driving/cooperative_driving_networking/msg/CooperativeAwarenessMessage.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/cooperative_driving_networking
)
_generate_msg_lisp(cooperative_driving_networking
  "/home/amjad/Desktop/pg2017w/ros/src/cooperative_driving/cooperative_driving_networking/msg/EmergencyBrake.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/cooperative_driving_networking
)
_generate_msg_lisp(cooperative_driving_networking
  "/home/amjad/Desktop/pg2017w/ros/src/cooperative_driving/cooperative_driving_networking/msg/Token.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/cooperative_driving_networking
)
_generate_msg_lisp(cooperative_driving_networking
  "/home/amjad/Desktop/pg2017w/ros/src/cooperative_driving/cooperative_driving_networking/msg/StringStamped.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/cooperative_driving_networking
)
_generate_msg_lisp(cooperative_driving_networking
  "/home/amjad/Desktop/pg2017w/ros/src/cooperative_driving/cooperative_driving_networking/msg/Platooning.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/cooperative_driving_networking
)

### Generating Services
_generate_srv_lisp(cooperative_driving_networking
  "/home/amjad/Desktop/pg2017w/ros/src/cooperative_driving/cooperative_driving_networking/srv/BroadcastMessage.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/cooperative_driving_networking
)

### Generating Module File
_generate_module_lisp(cooperative_driving_networking
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/cooperative_driving_networking
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(cooperative_driving_networking_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(cooperative_driving_networking_generate_messages cooperative_driving_networking_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/amjad/Desktop/pg2017w/ros/src/cooperative_driving/cooperative_driving_networking/srv/BroadcastMessage.srv" NAME_WE)
add_dependencies(cooperative_driving_networking_generate_messages_lisp _cooperative_driving_networking_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amjad/Desktop/pg2017w/ros/src/cooperative_driving/cooperative_driving_networking/msg/Token.msg" NAME_WE)
add_dependencies(cooperative_driving_networking_generate_messages_lisp _cooperative_driving_networking_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amjad/Desktop/pg2017w/ros/src/cooperative_driving/cooperative_driving_networking/msg/EmergencyBrake.msg" NAME_WE)
add_dependencies(cooperative_driving_networking_generate_messages_lisp _cooperative_driving_networking_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amjad/Desktop/pg2017w/ros/src/cooperative_driving/cooperative_driving_networking/msg/StringStamped.msg" NAME_WE)
add_dependencies(cooperative_driving_networking_generate_messages_lisp _cooperative_driving_networking_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amjad/Desktop/pg2017w/ros/src/cooperative_driving/cooperative_driving_networking/msg/CooperativeAwarenessMessage.msg" NAME_WE)
add_dependencies(cooperative_driving_networking_generate_messages_lisp _cooperative_driving_networking_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amjad/Desktop/pg2017w/ros/src/cooperative_driving/cooperative_driving_networking/msg/Platooning.msg" NAME_WE)
add_dependencies(cooperative_driving_networking_generate_messages_lisp _cooperative_driving_networking_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(cooperative_driving_networking_genlisp)
add_dependencies(cooperative_driving_networking_genlisp cooperative_driving_networking_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS cooperative_driving_networking_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(cooperative_driving_networking
  "/home/amjad/Desktop/pg2017w/ros/src/cooperative_driving/cooperative_driving_networking/msg/CooperativeAwarenessMessage.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/cooperative_driving_networking
)
_generate_msg_nodejs(cooperative_driving_networking
  "/home/amjad/Desktop/pg2017w/ros/src/cooperative_driving/cooperative_driving_networking/msg/EmergencyBrake.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/cooperative_driving_networking
)
_generate_msg_nodejs(cooperative_driving_networking
  "/home/amjad/Desktop/pg2017w/ros/src/cooperative_driving/cooperative_driving_networking/msg/Token.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/cooperative_driving_networking
)
_generate_msg_nodejs(cooperative_driving_networking
  "/home/amjad/Desktop/pg2017w/ros/src/cooperative_driving/cooperative_driving_networking/msg/StringStamped.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/cooperative_driving_networking
)
_generate_msg_nodejs(cooperative_driving_networking
  "/home/amjad/Desktop/pg2017w/ros/src/cooperative_driving/cooperative_driving_networking/msg/Platooning.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/cooperative_driving_networking
)

### Generating Services
_generate_srv_nodejs(cooperative_driving_networking
  "/home/amjad/Desktop/pg2017w/ros/src/cooperative_driving/cooperative_driving_networking/srv/BroadcastMessage.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/cooperative_driving_networking
)

### Generating Module File
_generate_module_nodejs(cooperative_driving_networking
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/cooperative_driving_networking
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(cooperative_driving_networking_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(cooperative_driving_networking_generate_messages cooperative_driving_networking_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/amjad/Desktop/pg2017w/ros/src/cooperative_driving/cooperative_driving_networking/srv/BroadcastMessage.srv" NAME_WE)
add_dependencies(cooperative_driving_networking_generate_messages_nodejs _cooperative_driving_networking_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amjad/Desktop/pg2017w/ros/src/cooperative_driving/cooperative_driving_networking/msg/Token.msg" NAME_WE)
add_dependencies(cooperative_driving_networking_generate_messages_nodejs _cooperative_driving_networking_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amjad/Desktop/pg2017w/ros/src/cooperative_driving/cooperative_driving_networking/msg/EmergencyBrake.msg" NAME_WE)
add_dependencies(cooperative_driving_networking_generate_messages_nodejs _cooperative_driving_networking_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amjad/Desktop/pg2017w/ros/src/cooperative_driving/cooperative_driving_networking/msg/StringStamped.msg" NAME_WE)
add_dependencies(cooperative_driving_networking_generate_messages_nodejs _cooperative_driving_networking_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amjad/Desktop/pg2017w/ros/src/cooperative_driving/cooperative_driving_networking/msg/CooperativeAwarenessMessage.msg" NAME_WE)
add_dependencies(cooperative_driving_networking_generate_messages_nodejs _cooperative_driving_networking_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amjad/Desktop/pg2017w/ros/src/cooperative_driving/cooperative_driving_networking/msg/Platooning.msg" NAME_WE)
add_dependencies(cooperative_driving_networking_generate_messages_nodejs _cooperative_driving_networking_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(cooperative_driving_networking_gennodejs)
add_dependencies(cooperative_driving_networking_gennodejs cooperative_driving_networking_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS cooperative_driving_networking_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(cooperative_driving_networking
  "/home/amjad/Desktop/pg2017w/ros/src/cooperative_driving/cooperative_driving_networking/msg/CooperativeAwarenessMessage.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/cooperative_driving_networking
)
_generate_msg_py(cooperative_driving_networking
  "/home/amjad/Desktop/pg2017w/ros/src/cooperative_driving/cooperative_driving_networking/msg/EmergencyBrake.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/cooperative_driving_networking
)
_generate_msg_py(cooperative_driving_networking
  "/home/amjad/Desktop/pg2017w/ros/src/cooperative_driving/cooperative_driving_networking/msg/Token.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/cooperative_driving_networking
)
_generate_msg_py(cooperative_driving_networking
  "/home/amjad/Desktop/pg2017w/ros/src/cooperative_driving/cooperative_driving_networking/msg/StringStamped.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/cooperative_driving_networking
)
_generate_msg_py(cooperative_driving_networking
  "/home/amjad/Desktop/pg2017w/ros/src/cooperative_driving/cooperative_driving_networking/msg/Platooning.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/cooperative_driving_networking
)

### Generating Services
_generate_srv_py(cooperative_driving_networking
  "/home/amjad/Desktop/pg2017w/ros/src/cooperative_driving/cooperative_driving_networking/srv/BroadcastMessage.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/cooperative_driving_networking
)

### Generating Module File
_generate_module_py(cooperative_driving_networking
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/cooperative_driving_networking
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(cooperative_driving_networking_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(cooperative_driving_networking_generate_messages cooperative_driving_networking_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/amjad/Desktop/pg2017w/ros/src/cooperative_driving/cooperative_driving_networking/srv/BroadcastMessage.srv" NAME_WE)
add_dependencies(cooperative_driving_networking_generate_messages_py _cooperative_driving_networking_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amjad/Desktop/pg2017w/ros/src/cooperative_driving/cooperative_driving_networking/msg/Token.msg" NAME_WE)
add_dependencies(cooperative_driving_networking_generate_messages_py _cooperative_driving_networking_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amjad/Desktop/pg2017w/ros/src/cooperative_driving/cooperative_driving_networking/msg/EmergencyBrake.msg" NAME_WE)
add_dependencies(cooperative_driving_networking_generate_messages_py _cooperative_driving_networking_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amjad/Desktop/pg2017w/ros/src/cooperative_driving/cooperative_driving_networking/msg/StringStamped.msg" NAME_WE)
add_dependencies(cooperative_driving_networking_generate_messages_py _cooperative_driving_networking_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amjad/Desktop/pg2017w/ros/src/cooperative_driving/cooperative_driving_networking/msg/CooperativeAwarenessMessage.msg" NAME_WE)
add_dependencies(cooperative_driving_networking_generate_messages_py _cooperative_driving_networking_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amjad/Desktop/pg2017w/ros/src/cooperative_driving/cooperative_driving_networking/msg/Platooning.msg" NAME_WE)
add_dependencies(cooperative_driving_networking_generate_messages_py _cooperative_driving_networking_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(cooperative_driving_networking_genpy)
add_dependencies(cooperative_driving_networking_genpy cooperative_driving_networking_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS cooperative_driving_networking_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/cooperative_driving_networking)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/cooperative_driving_networking
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(cooperative_driving_networking_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/cooperative_driving_networking)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/cooperative_driving_networking
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(cooperative_driving_networking_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/cooperative_driving_networking)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/cooperative_driving_networking
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(cooperative_driving_networking_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/cooperative_driving_networking)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/cooperative_driving_networking
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(cooperative_driving_networking_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/cooperative_driving_networking)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/cooperative_driving_networking\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/cooperative_driving_networking
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(cooperative_driving_networking_generate_messages_py std_msgs_generate_messages_py)
endif()
