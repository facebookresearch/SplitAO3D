# Copyright (c) Meta Platforms, Inc. and affiliates.
# All rights reserved.

# This source code is licensed under the license found in the
# LICENSE file in the root directory of this source tree.

cmake_minimum_required(VERSION 3.12 FATAL_ERROR)

project(SplitRenderingExperiments)

# Copied from apps/CMakeLists.txt
#
# Use C++17
set(CMAKE_CXX_STANDARD 17)

add_subdirectory("point_ao_split_rendering")
