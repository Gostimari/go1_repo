# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "metrics_extractor: 1 messages, 0 services")

set(MSG_I_FLAGS "-Imetrics_extractor:/root/catkin_ws/src/metrics_extractor/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(metrics_extractor_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/root/catkin_ws/src/metrics_extractor/msg/MotorState.msg" NAME_WE)
add_custom_target(_metrics_extractor_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "metrics_extractor" "/root/catkin_ws/src/metrics_extractor/msg/MotorState.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(metrics_extractor
  "/root/catkin_ws/src/metrics_extractor/msg/MotorState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/metrics_extractor
)

### Generating Services

### Generating Module File
_generate_module_cpp(metrics_extractor
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/metrics_extractor
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(metrics_extractor_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(metrics_extractor_generate_messages metrics_extractor_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/root/catkin_ws/src/metrics_extractor/msg/MotorState.msg" NAME_WE)
add_dependencies(metrics_extractor_generate_messages_cpp _metrics_extractor_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(metrics_extractor_gencpp)
add_dependencies(metrics_extractor_gencpp metrics_extractor_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS metrics_extractor_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(metrics_extractor
  "/root/catkin_ws/src/metrics_extractor/msg/MotorState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/metrics_extractor
)

### Generating Services

### Generating Module File
_generate_module_eus(metrics_extractor
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/metrics_extractor
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(metrics_extractor_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(metrics_extractor_generate_messages metrics_extractor_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/root/catkin_ws/src/metrics_extractor/msg/MotorState.msg" NAME_WE)
add_dependencies(metrics_extractor_generate_messages_eus _metrics_extractor_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(metrics_extractor_geneus)
add_dependencies(metrics_extractor_geneus metrics_extractor_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS metrics_extractor_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(metrics_extractor
  "/root/catkin_ws/src/metrics_extractor/msg/MotorState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/metrics_extractor
)

### Generating Services

### Generating Module File
_generate_module_lisp(metrics_extractor
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/metrics_extractor
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(metrics_extractor_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(metrics_extractor_generate_messages metrics_extractor_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/root/catkin_ws/src/metrics_extractor/msg/MotorState.msg" NAME_WE)
add_dependencies(metrics_extractor_generate_messages_lisp _metrics_extractor_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(metrics_extractor_genlisp)
add_dependencies(metrics_extractor_genlisp metrics_extractor_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS metrics_extractor_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(metrics_extractor
  "/root/catkin_ws/src/metrics_extractor/msg/MotorState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/metrics_extractor
)

### Generating Services

### Generating Module File
_generate_module_nodejs(metrics_extractor
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/metrics_extractor
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(metrics_extractor_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(metrics_extractor_generate_messages metrics_extractor_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/root/catkin_ws/src/metrics_extractor/msg/MotorState.msg" NAME_WE)
add_dependencies(metrics_extractor_generate_messages_nodejs _metrics_extractor_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(metrics_extractor_gennodejs)
add_dependencies(metrics_extractor_gennodejs metrics_extractor_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS metrics_extractor_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(metrics_extractor
  "/root/catkin_ws/src/metrics_extractor/msg/MotorState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/metrics_extractor
)

### Generating Services

### Generating Module File
_generate_module_py(metrics_extractor
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/metrics_extractor
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(metrics_extractor_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(metrics_extractor_generate_messages metrics_extractor_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/root/catkin_ws/src/metrics_extractor/msg/MotorState.msg" NAME_WE)
add_dependencies(metrics_extractor_generate_messages_py _metrics_extractor_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(metrics_extractor_genpy)
add_dependencies(metrics_extractor_genpy metrics_extractor_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS metrics_extractor_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/metrics_extractor)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/metrics_extractor
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(metrics_extractor_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/metrics_extractor)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/metrics_extractor
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(metrics_extractor_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/metrics_extractor)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/metrics_extractor
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(metrics_extractor_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/metrics_extractor)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/metrics_extractor
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(metrics_extractor_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/metrics_extractor)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/metrics_extractor\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/metrics_extractor
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(metrics_extractor_generate_messages_py std_msgs_generate_messages_py)
endif()
