# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_tp3_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED tp3_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(tp3_FOUND FALSE)
  elseif(NOT tp3_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(tp3_FOUND FALSE)
  endif()
  return()
endif()
set(_tp3_CONFIG_INCLUDED TRUE)

# output package information
if(NOT tp3_FIND_QUIETLY)
  message(STATUS "Found tp3: 0.0.0 (${tp3_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'tp3' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${tp3_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(tp3_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${tp3_DIR}/${_extra}")
endforeach()
