#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "run_ompl_constrained_planning::run_ompl_constrained_planning" for configuration "Release"
set_property(TARGET run_ompl_constrained_planning::run_ompl_constrained_planning APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(run_ompl_constrained_planning::run_ompl_constrained_planning PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/run_ompl_constrained_planning/run_ompl_constrained_planning"
  )

list(APPEND _IMPORT_CHECK_TARGETS run_ompl_constrained_planning::run_ompl_constrained_planning )
list(APPEND _IMPORT_CHECK_FILES_FOR_run_ompl_constrained_planning::run_ompl_constrained_planning "${_IMPORT_PREFIX}/lib/run_ompl_constrained_planning/run_ompl_constrained_planning" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
