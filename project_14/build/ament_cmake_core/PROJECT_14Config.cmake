# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_PROJECT_14_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED PROJECT_14_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(PROJECT_14_FOUND FALSE)
  elseif(NOT PROJECT_14_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(PROJECT_14_FOUND FALSE)
  endif()
  return()
endif()
set(_PROJECT_14_CONFIG_INCLUDED TRUE)

# output package information
if(NOT PROJECT_14_FIND_QUIETLY)
  message(STATUS "Found PROJECT_14: 0.0.1 (${PROJECT_14_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'PROJECT_14' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${PROJECT_14_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(PROJECT_14_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${PROJECT_14_DIR}/${_extra}")
endforeach()
