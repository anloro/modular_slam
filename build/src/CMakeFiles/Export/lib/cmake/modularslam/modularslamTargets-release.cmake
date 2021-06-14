#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "modularslam::modularslam" for configuration "Release"
set_property(TARGET modularslam::modularslam APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(modularslam::modularslam PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libmodularslam.so"
  IMPORTED_SONAME_RELEASE "libmodularslam.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS modularslam::modularslam )
list(APPEND _IMPORT_CHECK_FILES_FOR_modularslam::modularslam "${_IMPORT_PREFIX}/lib/libmodularslam.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
