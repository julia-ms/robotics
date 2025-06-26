# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_tp2_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED tp2_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(tp2_FOUND FALSE)
  elseif(NOT tp2_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(tp2_FOUND FALSE)
  endif()
  return()
endif()
set(_tp2_CONFIG_INCLUDED TRUE)

# output package information
if(NOT tp2_FIND_QUIETLY)
  message(STATUS "Found tp2: 0.0.0 (${tp2_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'tp2' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${tp2_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(tp2_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${tp2_DIR}/${_extra}")
endforeach()
