cmake_minimum_required(VERSION 3.16)
project(rc26_small_gicp VERSION 1.0.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the type of build." FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

option(BUILD_SHARED_LIBS "Build shared libraries" ON)
option(CMAKE_POSITION_INDEPENDENT_CODE "Generate position-independent code" ON)
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

# Build options
option(BUILD_HELPER "Build helper library" ON)
option(ENABLE_COVERAGE "Enable coverage" OFF)

# Dependency options
set(BUILD_WITH_OPENMP "auto" CACHE STRING "Build with OpenMP")
option(BUILD_WITH_PCL "Build with PCL" OFF)
option(BUILD_WITH_MARCH_NATIVE "Build with -march=native" OFF)

# Eigen is the sole mandatory dependency
find_package(Eigen3 CONFIG)

if(NOT Eigen3_FOUND)
  message(STATUS "System Eigen not found. Download Eigen 3.4.0.")
  include(FetchContent)
  FetchContent_Populate(
    Eigen3
    URL https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz
  )
  add_library(Eigen3::Eigen INTERFACE IMPORTED GLOBAL)
  set_target_properties(Eigen3::Eigen PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${eigen3_SOURCE_DIR}"
  )
endif()

if(BUILD_WITH_OPENMP STREQUAL "auto")
  find_package(OpenMP)
  set(BUILD_WITH_OPENMP ${OpenMP_FOUND})
elseif(BUILD_WITH_OPENMP)
  find_package(OpenMP REQUIRED)
endif()

if (BUILD_WITH_PCL)
  find_package(PCL REQUIRED)
  add_compile_definitions(BUILD_WITH_PCL)
  if (NOT TARGET PCL::PCL)
    add_library(PCL::PCL INTERFACE IMPORTED)
    set_target_properties(PCL::PCL PROPERTIES
      INTERFACE_INCLUDE_DIRECTORIES "${PCL_INCLUDE_DIRS}"
      INTERFACE_LINK_LIBRARIES "${PCL_LIBRARIES}"
    )
  endif()
endif()

if(BUILD_WITH_MARCH_NATIVE)
  add_compile_options(-march=native)
endif()

if(MSVC)
  add_compile_definitions(_USE_MATH_DEFINES)
  add_compile_options(/bigobj)
  if(BUILD_WITH_OPENMP)
    add_compile_options(/openmp:llvm)
  endif()
endif()

##############
## Coverage ##
##############
if(ENABLE_COVERAGE)
  # https://danielsieger.com/blog/2022/03/06/code-coverage-for-cpp.html
  set(CMAKE_CXX_FLAGS "-O0 -coverage")

  find_program(LCOV lcov REQUIRED)
  find_program(GENHTML genhtml REQUIRED)

  add_custom_target(coverage
    COMMAND ${LCOV} --directory . --capture --output-file coverage.info --ignore-errors mismatch
    COMMAND ${LCOV} --remove coverage.info -o coverage.info '/usr/*' --ignore-errors mismatch
    COMMAND ${GENHTML} --demangle-cpp -o coverage coverage.info
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR})
endif()

###########
## Build ##
###########

include_directories(include)

# Helper library
if(BUILD_HELPER)
  add_library(small_gicp
    src/small_gicp/registration/registration.cpp
    src/small_gicp/registration/registration_helper.cpp
  )
  target_include_directories(small_gicp PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(small_gicp PUBLIC
    Eigen3::Eigen
    $<TARGET_NAME_IF_EXISTS:OpenMP::OpenMP_CXX>
  )
endif()

#############
## Install ##
#############

include(GNUInstallDirs)
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(FILES LICENSE DESTINATION ${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME})

if(BUILD_HELPER)
  install(TARGETS small_gicp
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  )
endif()
