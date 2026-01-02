# vcpkg_setup.cmake - Common vcpkg configuration
# This file sets up vcpkg paths using environment variables

# Append vcpkg paths if available (using VCPKG_ROOT environment variable)
if(DEFINED ENV{VCPKG_ROOT})
  set(_VCPKG_ROOT "$ENV{VCPKG_ROOT}")
  
  # Auto-detect triplet if not set
  if(NOT DEFINED VCPKG_TARGET_TRIPLET)
    if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
      if(CMAKE_SIZEOF_VOID_P EQUAL 8)
        set(_VCPKG_TRIPLET "x64-linux")
      else()
        set(_VCPKG_TRIPLET "x86-linux")
      endif()
    elseif(CMAKE_SYSTEM_NAME STREQUAL "Windows")
      if(CMAKE_SIZEOF_VOID_P EQUAL 8)
        set(_VCPKG_TRIPLET "x64-windows")
      else()
        set(_VCPKG_TRIPLET "x86-windows")
      endif()
    elseif(CMAKE_SYSTEM_NAME STREQUAL "Darwin")
      set(_VCPKG_TRIPLET "x64-osx")
    endif()
  else()
    set(_VCPKG_TRIPLET "${VCPKG_TARGET_TRIPLET}")
  endif()

  set(VCPKG_INSTALLED_DIR "${_VCPKG_ROOT}/installed/${_VCPKG_TRIPLET}")
  
  if(EXISTS "${VCPKG_INSTALLED_DIR}")
    list(APPEND CMAKE_PREFIX_PATH "${VCPKG_INSTALLED_DIR}")
    list(APPEND CMAKE_MODULE_PATH "${VCPKG_INSTALLED_DIR}/share")
    message(STATUS "vcpkg: Using installed packages from ${VCPKG_INSTALLED_DIR}")
  else()
    message(WARNING "vcpkg: VCPKG_ROOT is set but installed directory not found: ${VCPKG_INSTALLED_DIR}")
  endif()
  
  unset(_VCPKG_ROOT)
  unset(_VCPKG_TRIPLET)
endif()

# gflags must use namespace target for glog compatibility (required by Ceres)
set(GFLAGS_USE_TARGET_NAMESPACE TRUE CACHE BOOL "Use gflags namespace targets" FORCE)

# OpenVINO setup - check environment variable or default paths
if(DEFINED ENV{INTEL_OPENVINO_DIR})
  set(_OPENVINO_DIR "$ENV{INTEL_OPENVINO_DIR}")
elseif(EXISTS "/opt/intel/openvino_2024")
  set(_OPENVINO_DIR "/opt/intel/openvino_2024")
elseif(EXISTS "/opt/intel/openvino")
  set(_OPENVINO_DIR "/opt/intel/openvino")
endif()

if(DEFINED _OPENVINO_DIR AND EXISTS "${_OPENVINO_DIR}/runtime/cmake")
  list(APPEND CMAKE_PREFIX_PATH "${_OPENVINO_DIR}/runtime/cmake")
  message(STATUS "OpenVINO: Using installation from ${_OPENVINO_DIR}")
  unset(_OPENVINO_DIR)
endif()
