# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_mid360_driver_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED mid360_driver_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(mid360_driver_FOUND FALSE)
  elseif(NOT mid360_driver_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(mid360_driver_FOUND FALSE)
  endif()
  return()
endif()
set(_mid360_driver_CONFIG_INCLUDED TRUE)

# output package information
if(NOT mid360_driver_FIND_QUIETLY)
  message(STATUS "Found mid360_driver: 1.0.0 (${mid360_driver_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'mid360_driver' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${mid360_driver_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(mid360_driver_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "ament_cmake_export_dependencies-extras.cmake;ament_cmake_export_libraries-extras.cmake")
foreach(_extra ${_extras})
  include("${mid360_driver_DIR}/${_extra}")
endforeach()
