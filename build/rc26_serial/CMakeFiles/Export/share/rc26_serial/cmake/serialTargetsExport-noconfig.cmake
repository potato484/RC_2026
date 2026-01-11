#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "rc26_serial::serial_driver" for configuration ""
set_property(TARGET rc26_serial::serial_driver APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(rc26_serial::serial_driver PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libserial_driver.so"
  IMPORTED_SONAME_NOCONFIG "libserial_driver.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS rc26_serial::serial_driver )
list(APPEND _IMPORT_CHECK_FILES_FOR_rc26_serial::serial_driver "${_IMPORT_PREFIX}/lib/libserial_driver.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
