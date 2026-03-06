#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "eigenpy::eigenpy" for configuration ""
set_property(TARGET eigenpy::eigenpy APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(eigenpy::eigenpy PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libeigenpy.so"
  IMPORTED_SONAME_NOCONFIG "libeigenpy.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS eigenpy::eigenpy )
list(APPEND _IMPORT_CHECK_FILES_FOR_eigenpy::eigenpy "${_IMPORT_PREFIX}/lib/libeigenpy.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
