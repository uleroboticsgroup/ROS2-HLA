# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_Prototipo_GZ_HLA_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED Prototipo_GZ_HLA_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(Prototipo_GZ_HLA_FOUND FALSE)
  elseif(NOT Prototipo_GZ_HLA_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(Prototipo_GZ_HLA_FOUND FALSE)
  endif()
  return()
endif()
set(_Prototipo_GZ_HLA_CONFIG_INCLUDED TRUE)

# output package information
if(NOT Prototipo_GZ_HLA_FIND_QUIETLY)
  message(STATUS "Found Prototipo_GZ_HLA: 0.0.0 (${Prototipo_GZ_HLA_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'Prototipo_GZ_HLA' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT Prototipo_GZ_HLA_DEPRECATED_QUIET)
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(Prototipo_GZ_HLA_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${Prototipo_GZ_HLA_DIR}/${_extra}")
endforeach()
