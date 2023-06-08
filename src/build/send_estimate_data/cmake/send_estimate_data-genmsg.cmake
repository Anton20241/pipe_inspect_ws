# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "send_estimate_data: 1 messages, 0 services")

set(MSG_I_FLAGS "-Isend_estimate_data:/home/anton20241/pipe_inspect_ws/src/send_estimate_data/msg;-Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(send_estimate_data_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/anton20241/pipe_inspect_ws/src/send_estimate_data/msg/Poses.msg" NAME_WE)
add_custom_target(_send_estimate_data_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "send_estimate_data" "/home/anton20241/pipe_inspect_ws/src/send_estimate_data/msg/Poses.msg" "geometry_msgs/Quaternion:geometry_msgs/PoseStamped:geometry_msgs/Pose:std_msgs/Header:geometry_msgs/Point"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(send_estimate_data
  "/home/anton20241/pipe_inspect_ws/src/send_estimate_data/msg/Poses.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/send_estimate_data
)

### Generating Services

### Generating Module File
_generate_module_cpp(send_estimate_data
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/send_estimate_data
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(send_estimate_data_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(send_estimate_data_generate_messages send_estimate_data_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/anton20241/pipe_inspect_ws/src/send_estimate_data/msg/Poses.msg" NAME_WE)
add_dependencies(send_estimate_data_generate_messages_cpp _send_estimate_data_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(send_estimate_data_gencpp)
add_dependencies(send_estimate_data_gencpp send_estimate_data_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS send_estimate_data_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(send_estimate_data
  "/home/anton20241/pipe_inspect_ws/src/send_estimate_data/msg/Poses.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/send_estimate_data
)

### Generating Services

### Generating Module File
_generate_module_eus(send_estimate_data
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/send_estimate_data
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(send_estimate_data_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(send_estimate_data_generate_messages send_estimate_data_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/anton20241/pipe_inspect_ws/src/send_estimate_data/msg/Poses.msg" NAME_WE)
add_dependencies(send_estimate_data_generate_messages_eus _send_estimate_data_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(send_estimate_data_geneus)
add_dependencies(send_estimate_data_geneus send_estimate_data_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS send_estimate_data_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(send_estimate_data
  "/home/anton20241/pipe_inspect_ws/src/send_estimate_data/msg/Poses.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/send_estimate_data
)

### Generating Services

### Generating Module File
_generate_module_lisp(send_estimate_data
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/send_estimate_data
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(send_estimate_data_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(send_estimate_data_generate_messages send_estimate_data_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/anton20241/pipe_inspect_ws/src/send_estimate_data/msg/Poses.msg" NAME_WE)
add_dependencies(send_estimate_data_generate_messages_lisp _send_estimate_data_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(send_estimate_data_genlisp)
add_dependencies(send_estimate_data_genlisp send_estimate_data_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS send_estimate_data_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(send_estimate_data
  "/home/anton20241/pipe_inspect_ws/src/send_estimate_data/msg/Poses.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/send_estimate_data
)

### Generating Services

### Generating Module File
_generate_module_nodejs(send_estimate_data
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/send_estimate_data
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(send_estimate_data_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(send_estimate_data_generate_messages send_estimate_data_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/anton20241/pipe_inspect_ws/src/send_estimate_data/msg/Poses.msg" NAME_WE)
add_dependencies(send_estimate_data_generate_messages_nodejs _send_estimate_data_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(send_estimate_data_gennodejs)
add_dependencies(send_estimate_data_gennodejs send_estimate_data_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS send_estimate_data_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(send_estimate_data
  "/home/anton20241/pipe_inspect_ws/src/send_estimate_data/msg/Poses.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/send_estimate_data
)

### Generating Services

### Generating Module File
_generate_module_py(send_estimate_data
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/send_estimate_data
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(send_estimate_data_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(send_estimate_data_generate_messages send_estimate_data_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/anton20241/pipe_inspect_ws/src/send_estimate_data/msg/Poses.msg" NAME_WE)
add_dependencies(send_estimate_data_generate_messages_py _send_estimate_data_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(send_estimate_data_genpy)
add_dependencies(send_estimate_data_genpy send_estimate_data_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS send_estimate_data_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/send_estimate_data)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/send_estimate_data
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(send_estimate_data_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/send_estimate_data)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/send_estimate_data
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(send_estimate_data_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/send_estimate_data)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/send_estimate_data
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(send_estimate_data_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/send_estimate_data)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/send_estimate_data
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(send_estimate_data_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/send_estimate_data)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/send_estimate_data\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/send_estimate_data
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(send_estimate_data_generate_messages_py geometry_msgs_generate_messages_py)
endif()
