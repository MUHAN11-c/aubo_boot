#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "moveit_servo::moveit_servo_lib" for configuration "Release"
set_property(TARGET moveit_servo::moveit_servo_lib APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(moveit_servo::moveit_servo_lib PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libmoveit_servo_lib.so.2.2.3"
  IMPORTED_SONAME_RELEASE "libmoveit_servo_lib.so.2.2.3"
  )

list(APPEND _IMPORT_CHECK_TARGETS moveit_servo::moveit_servo_lib )
list(APPEND _IMPORT_CHECK_FILES_FOR_moveit_servo::moveit_servo_lib "${_IMPORT_PREFIX}/lib/libmoveit_servo_lib.so.2.2.3" )

# Import target "moveit_servo::moveit_servo_lib_parameters" for configuration "Release"
set_property(TARGET moveit_servo::moveit_servo_lib_parameters APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(moveit_servo::moveit_servo_lib_parameters PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libmoveit_servo_lib_parameters.so.2.2.3"
  IMPORTED_SONAME_RELEASE "libmoveit_servo_lib_parameters.so.2.2.3"
  )

list(APPEND _IMPORT_CHECK_TARGETS moveit_servo::moveit_servo_lib_parameters )
list(APPEND _IMPORT_CHECK_FILES_FOR_moveit_servo::moveit_servo_lib_parameters "${_IMPORT_PREFIX}/lib/libmoveit_servo_lib_parameters.so.2.2.3" )

# Import target "moveit_servo::pose_tracking" for configuration "Release"
set_property(TARGET moveit_servo::pose_tracking APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(moveit_servo::pose_tracking PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libpose_tracking.so"
  IMPORTED_SONAME_RELEASE "libpose_tracking.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS moveit_servo::pose_tracking )
list(APPEND _IMPORT_CHECK_FILES_FOR_moveit_servo::pose_tracking "${_IMPORT_PREFIX}/lib/libpose_tracking.so" )

# Import target "moveit_servo::servo_server" for configuration "Release"
set_property(TARGET moveit_servo::servo_server APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(moveit_servo::servo_server PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libservo_server.so"
  IMPORTED_SONAME_RELEASE "libservo_server.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS moveit_servo::servo_server )
list(APPEND _IMPORT_CHECK_FILES_FOR_moveit_servo::servo_server "${_IMPORT_PREFIX}/lib/libservo_server.so" )

# Import target "moveit_servo::servo_controller_input" for configuration "Release"
set_property(TARGET moveit_servo::servo_controller_input APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(moveit_servo::servo_controller_input PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libservo_controller_input.so"
  IMPORTED_SONAME_RELEASE "libservo_controller_input.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS moveit_servo::servo_controller_input )
list(APPEND _IMPORT_CHECK_FILES_FOR_moveit_servo::servo_controller_input "${_IMPORT_PREFIX}/lib/libservo_controller_input.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
