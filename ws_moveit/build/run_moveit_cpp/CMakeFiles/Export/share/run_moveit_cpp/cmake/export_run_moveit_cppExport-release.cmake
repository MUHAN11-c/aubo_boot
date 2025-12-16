#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "run_moveit_cpp::run_moveit_cpp" for configuration "Release"
set_property(TARGET run_moveit_cpp::run_moveit_cpp APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(run_moveit_cpp::run_moveit_cpp PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/run_moveit_cpp/run_moveit_cpp"
  )

list(APPEND _IMPORT_CHECK_TARGETS run_moveit_cpp::run_moveit_cpp )
list(APPEND _IMPORT_CHECK_FILES_FOR_run_moveit_cpp::run_moveit_cpp "${_IMPORT_PREFIX}/lib/run_moveit_cpp/run_moveit_cpp" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
