cmake_minimum_required(VERSION 3.12 FATAL_ERROR)

project(SplitRenderingExperiments)

# Copied from apps/CMakeLists.txt
#
# Use C++17
set(CMAKE_CXX_STANDARD 17)

add_subdirectory("point_ao_split_rendering")