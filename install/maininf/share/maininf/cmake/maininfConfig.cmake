# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_maininf_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED maininf_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(maininf_FOUND FALSE)
  elseif(NOT maininf_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(maininf_FOUND FALSE)
  endif()
  return()
endif()
set(_maininf_CONFIG_INCLUDED TRUE)

# output package information
if(NOT maininf_FIND_QUIETLY)
  message(STATUS "Found maininf: 0.0.0 (${maininf_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'maininf' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${maininf_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(maininf_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "ament_cmake_export_dependencies-extras.cmake")
foreach(_extra ${_extras})
  include("${maininf_DIR}/${_extra}")
endforeach()
