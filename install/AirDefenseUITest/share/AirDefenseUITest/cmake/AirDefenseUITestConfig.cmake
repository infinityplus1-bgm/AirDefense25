# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_AirDefenseUITest_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED AirDefenseUITest_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(AirDefenseUITest_FOUND FALSE)
  elseif(NOT AirDefenseUITest_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(AirDefenseUITest_FOUND FALSE)
  endif()
  return()
endif()
set(_AirDefenseUITest_CONFIG_INCLUDED TRUE)

# output package information
if(NOT AirDefenseUITest_FIND_QUIETLY)
  message(STATUS "Found AirDefenseUITest: 0.0.0 (${AirDefenseUITest_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'AirDefenseUITest' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${AirDefenseUITest_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(AirDefenseUITest_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "ament_cmake_export_dependencies-extras.cmake")
foreach(_extra ${_extras})
  include("${AirDefenseUITest_DIR}/${_extra}")
endforeach()
