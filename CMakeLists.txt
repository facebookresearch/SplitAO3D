cmake_minimum_required(VERSION 3.12 FATAL_ERROR)

project(SplitRenderingExperiments)

# Copied from apps/CMakeLists.txt
#
# Use C++17
set(CMAKE_CXX_STANDARD 17)

# This adds -fPIC compiler option which is required when making shared libs
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
# Global Windows customizations.
if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
  # Tell the MSVC compiler to define __cplusplus as something indicitive of the
  # actual version.
  add_definitions("/Zc:__cplusplus")
endif()

# Ask glog to not to try to find an external gflags
set(WITH_GFLAGS
    OFF
    CACHE BOOL "Build gflags with glog")
option(BUILD_TESTING "" OFF)
add_subdirectory("../../shared/third-party/glog/0.4.0/glog" glog)

add_subdirectory("../../shared/third-party/gflags/2.2.2/gflags" gflags)

option(WITH_RLR_STREAMING "" ON)
option(WITH_RLR_TCP "" ON)
add_subdirectory("../../shared/first-party/streaming" streaming)

add_subdirectory("../../split_rendering/research_framework" research_framework)

#add_subdirectory("../../split_rendering/samples" samples)

add_subdirectory("point_ao_split_rendering")