# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "hippo2: 0 messages, 1 services")

set(MSG_I_FLAGS "-Istd_msgs:/opt/ros/hydro/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(hippo2_generate_messages ALL)

#
#  langs = gencpp;geneus;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services
_generate_srv_cpp(hippo2
  "/home/pioneer/group41/src/hippo2/srv/Int.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/hippo2
)

### Generating Module File
_generate_module_cpp(hippo2
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/hippo2
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(hippo2_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(hippo2_generate_messages hippo2_generate_messages_cpp)

# target for backward compatibility
add_custom_target(hippo2_gencpp)
add_dependencies(hippo2_gencpp hippo2_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS hippo2_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages

### Generating Services
_generate_srv_eus(hippo2
  "/home/pioneer/group41/src/hippo2/srv/Int.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/hippo2
)

### Generating Module File
_generate_module_eus(hippo2
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/hippo2
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(hippo2_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(hippo2_generate_messages hippo2_generate_messages_eus)

# target for backward compatibility
add_custom_target(hippo2_geneus)
add_dependencies(hippo2_geneus hippo2_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS hippo2_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages

### Generating Services
_generate_srv_lisp(hippo2
  "/home/pioneer/group41/src/hippo2/srv/Int.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/hippo2
)

### Generating Module File
_generate_module_lisp(hippo2
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/hippo2
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(hippo2_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(hippo2_generate_messages hippo2_generate_messages_lisp)

# target for backward compatibility
add_custom_target(hippo2_genlisp)
add_dependencies(hippo2_genlisp hippo2_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS hippo2_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages

### Generating Services
_generate_srv_py(hippo2
  "/home/pioneer/group41/src/hippo2/srv/Int.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/hippo2
)

### Generating Module File
_generate_module_py(hippo2
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/hippo2
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(hippo2_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(hippo2_generate_messages hippo2_generate_messages_py)

# target for backward compatibility
add_custom_target(hippo2_genpy)
add_dependencies(hippo2_genpy hippo2_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS hippo2_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/hippo2)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/hippo2
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
add_dependencies(hippo2_generate_messages_cpp std_msgs_generate_messages_cpp)

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/hippo2)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/hippo2
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
add_dependencies(hippo2_generate_messages_eus std_msgs_generate_messages_eus)

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/hippo2)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/hippo2
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
add_dependencies(hippo2_generate_messages_lisp std_msgs_generate_messages_lisp)

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/hippo2)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/hippo2\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/hippo2
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
add_dependencies(hippo2_generate_messages_py std_msgs_generate_messages_py)
