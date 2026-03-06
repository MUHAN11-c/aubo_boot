# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "rviz: 0 messages, 1 services")

set(MSG_I_FLAGS "-Istd_msgs:/home/mu/IVG/ros_catkin_ws/install_isolated/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(rviz_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/mu/IVG/ros_catkin_ws/src/rviz/srv/SendFilePath.srv" NAME_WE)
add_custom_target(_rviz_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "rviz" "/home/mu/IVG/ros_catkin_ws/src/rviz/srv/SendFilePath.srv" "std_msgs/String"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services
_generate_srv_cpp(rviz
  "/home/mu/IVG/ros_catkin_ws/src/rviz/srv/SendFilePath.srv"
  "${MSG_I_FLAGS}"
  "/home/mu/IVG/ros_catkin_ws/install_isolated/share/std_msgs/cmake/../msg/String.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rviz
)

### Generating Module File
_generate_module_cpp(rviz
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rviz
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(rviz_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(rviz_generate_messages rviz_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/mu/IVG/ros_catkin_ws/src/rviz/srv/SendFilePath.srv" NAME_WE)
add_dependencies(rviz_generate_messages_cpp _rviz_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(rviz_gencpp)
add_dependencies(rviz_gencpp rviz_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS rviz_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages

### Generating Services
_generate_srv_eus(rviz
  "/home/mu/IVG/ros_catkin_ws/src/rviz/srv/SendFilePath.srv"
  "${MSG_I_FLAGS}"
  "/home/mu/IVG/ros_catkin_ws/install_isolated/share/std_msgs/cmake/../msg/String.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/rviz
)

### Generating Module File
_generate_module_eus(rviz
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/rviz
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(rviz_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(rviz_generate_messages rviz_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/mu/IVG/ros_catkin_ws/src/rviz/srv/SendFilePath.srv" NAME_WE)
add_dependencies(rviz_generate_messages_eus _rviz_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(rviz_geneus)
add_dependencies(rviz_geneus rviz_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS rviz_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages

### Generating Services
_generate_srv_lisp(rviz
  "/home/mu/IVG/ros_catkin_ws/src/rviz/srv/SendFilePath.srv"
  "${MSG_I_FLAGS}"
  "/home/mu/IVG/ros_catkin_ws/install_isolated/share/std_msgs/cmake/../msg/String.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rviz
)

### Generating Module File
_generate_module_lisp(rviz
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rviz
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(rviz_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(rviz_generate_messages rviz_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/mu/IVG/ros_catkin_ws/src/rviz/srv/SendFilePath.srv" NAME_WE)
add_dependencies(rviz_generate_messages_lisp _rviz_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(rviz_genlisp)
add_dependencies(rviz_genlisp rviz_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS rviz_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages

### Generating Services
_generate_srv_nodejs(rviz
  "/home/mu/IVG/ros_catkin_ws/src/rviz/srv/SendFilePath.srv"
  "${MSG_I_FLAGS}"
  "/home/mu/IVG/ros_catkin_ws/install_isolated/share/std_msgs/cmake/../msg/String.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/rviz
)

### Generating Module File
_generate_module_nodejs(rviz
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/rviz
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(rviz_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(rviz_generate_messages rviz_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/mu/IVG/ros_catkin_ws/src/rviz/srv/SendFilePath.srv" NAME_WE)
add_dependencies(rviz_generate_messages_nodejs _rviz_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(rviz_gennodejs)
add_dependencies(rviz_gennodejs rviz_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS rviz_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages

### Generating Services
_generate_srv_py(rviz
  "/home/mu/IVG/ros_catkin_ws/src/rviz/srv/SendFilePath.srv"
  "${MSG_I_FLAGS}"
  "/home/mu/IVG/ros_catkin_ws/install_isolated/share/std_msgs/cmake/../msg/String.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rviz
)

### Generating Module File
_generate_module_py(rviz
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rviz
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(rviz_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(rviz_generate_messages rviz_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/mu/IVG/ros_catkin_ws/src/rviz/srv/SendFilePath.srv" NAME_WE)
add_dependencies(rviz_generate_messages_py _rviz_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(rviz_genpy)
add_dependencies(rviz_genpy rviz_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS rviz_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rviz)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rviz
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(rviz_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/rviz)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/rviz
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(rviz_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rviz)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rviz
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(rviz_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/rviz)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/rviz
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(rviz_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rviz)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rviz\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rviz
    DESTINATION ${genpy_INSTALL_DIR}
    # skip all init files
    PATTERN "__init__.py" EXCLUDE
    PATTERN "__init__.pyc" EXCLUDE
  )
  # install init files which are not in the root folder of the generated code
  string(REGEX REPLACE "([][+.*()^])" "\\\\\\1" ESCAPED_PATH "${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rviz")
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rviz
    DESTINATION ${genpy_INSTALL_DIR}
    FILES_MATCHING
    REGEX "${ESCAPED_PATH}/.+/__init__.pyc?$"
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(rviz_generate_messages_py std_msgs_generate_messages_py)
endif()
