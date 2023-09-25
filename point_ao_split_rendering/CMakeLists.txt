# Copyright (c) Meta Platforms, Inc. and affiliates.
# All rights reserved.

# This source code is licensed under the license found in the
# LICENSE file in the root directory of this source tree.

# This CMakeLists.txt is based on apps\LEm\CMakeLists.txt

# Use C++17 unless overridden by -DCMAKE_CXX_STANDARD on the command line
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

set(LZ4_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/../third_party/lz4_win64_v1_9_4")
set(LZ4_LIB_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../third_party/lz4_win64_v1_9_4/static/liblz4_static.lib")
set(ZSTD_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/../third_party/zstd-v1.5.2-win64")
set(ZSTD_LIB_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../third_party/zstd-v1.5.2-win64/static/libzstd_static.lib")

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../third_party/poisson_sampling/)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../third_party/nanoflann/)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../third_party/flip/)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../third_party/)
include_directories(${ZSTD_DIRECTORY}/include/)
include_directories(${LZ4_DIRECTORY}/include/)

	
set(SHADERS
  HashFunctionShared.slang
  PointCloudVisualizationPass.slang
  PointCellAllocationStage.cs.slang
  PointCellCreateNetworkBufferStage.cs.slang
  PointHashCreateNetworkBufferStage.cs.slang
  KDTree.slang
  OctEncoding.slang
  PointData.slang
  PointAOConstantsShared.slangh
  PointAOSamplingUtils.slang
  PointRTAO.rt.slang
  PointGen.rt.slang
  RTAO.rt.slang
  BlurPass.cs.slang
  DepthNormals.ps.slang
  FinalComposite.ps.slang
  FinalCompositePointAO.ps.slang
  PointCellUpdateStage.cs.slang
  PointHashUpdateStage.cs.slang
  CompositeClient.ps.slang
  ClientCompositePointAO.ps.slang
  VertexAnimationExport.cs.slang
  )


add_falcor_executable(FalcorServer)



target_sources(FalcorServer PRIVATE
  BinaryMessageType.h
  MeshPointGenerator.h
  MeshPointGenerator.cpp
  PointKDTreeGenerator.h
  PointKDTreeGenerator.cpp
  PointHashGenerator.h
  PointHashGenerator.cpp
  PointServerHashGenerator.h
  PointServerHashGenerator.cpp
  ServerMain.cpp
  NetworkServer.cpp
  NetworkServer.h
  NetworkCompressionBase.cpp
  NetworkCompressionBase.h
  ServerPointRenderer.cpp
  ServerPointRenderer.h
  FLIPScreenshotComparison.h
  ScreenshotCaptureHelper.h
  PointCloudVisualizationPass.cpp
  PointCloudVisualizationPass.h
  PointCellAllocationStage.cpp
  PointCellAllocationStage.h
  PointCellCreateNetworkBufferStage.cpp
  PointCellCreateNetworkBufferStage.h
  PointHashCreateNetworkBufferStage.cpp
  PointHashCreateNetworkBufferStage.h
  SSAO.cpp
  SSAO.h
  LZ4Compression.cpp
  LZ4Compression.h
  ZSTDCompression.cpp
  ZSTDCompression.h
  ${SHADERS}
)

target_link_libraries(FalcorServer PRIVATE ${LIBS} ${LZ4_LIB_PATH} ${ZSTD_LIB_PATH})

target_copy_shaders(FalcorServer Samples/FalcorServer)

target_source_group(FalcorServer "Samples")
