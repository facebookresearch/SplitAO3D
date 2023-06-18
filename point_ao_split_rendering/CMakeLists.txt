# This CMakeLists.txt is based on apps\LEm\CMakeLists.txt

# Use C++17 unless overridden by -DCMAKE_CXX_STANDARD on the command line
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

set(LZ4_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/../lz4_win64_v1_9_4")
set(LZ4_LIB_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../lz4_win64_v1_9_4/static/liblz4_static.lib")
set(ZSTD_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/../zstd-v1.5.2-win64")
set(ZSTD_LIB_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../zstd-v1.5.2-win64/static/libzstd_static.lib")

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../../split_rendering/)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../poisson_sampling/)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../nanoflann/)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../flip/)
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

#falcor_sample(
#  NAME
#  FalcorServer
#  SOURCES
#  BinaryMessageType.h
#  MeshPointGenerator.h
#  MeshPointGenerator.cpp
#  PointKDTreeGenerator.h
#  PointKDTreeGenerator.cpp
#  PointHashGenerator.h
#  PointHashGenerator.cpp
#  PointServerHashGenerator.h
#  PointServerHashGenerator.cpp
#  TCPNetworkBase.h
#  TCPNetworkBase.cpp
#  ServerMain.cpp
#  NetworkServer.cpp
#  NetworkServer.h
#  NetworkCompressionBase.cpp
#  NetworkCompressionBase.h
#  ServerScreenSpaceRenderer.cpp
#  ServerScreenSpaceRenderer.h
#  ServerPointRenderer.cpp
#  ServerPointRenderer.h
#  FLIPScreenshotComparison.h
#  ../nanoflann/nanoflann.hpp
#  ScreenshotCaptureHelper.h
#  PointCloudVisualizationPass.cpp
#  PointCloudVisualizationPass.h
#  PointCellAllocationStage.cpp
#  PointCellAllocationStage.h
#  PointCellCreateNetworkBufferStage.cpp
#  PointCellCreateNetworkBufferStage.h
#  PointHashCreateNetworkBufferStage.cpp
#  PointHashCreateNetworkBufferStage.h
#  SSAO.cpp
#  SSAO.h
#  LZ4Compression.cpp
#  LZ4Compression.h
#  ZSTDCompression.cpp
#  ZSTDCompression.h
#  ${SHADERS}
#  )

#target_link_libraries(FalcorServer PRIVATE ${LIBS} ${LZ4_LIB_PATH} ${ZSTD_LIB_PATH})

#falcor_sample(
#  NAME
#  FalcorClient
#  SOURCES
#  BinaryMessageType.h
#  TCPNetworkBase.h
#  TCPNetworkBase.cpp
#  ClientMain.cpp
#  NetworkClient.cpp
#  NetworkClient.h
#  NetworkCompressionBase.cpp
#  NetworkCompressionBase.h
#  ClientScreenSpaceRenderer.cpp
#  ClientScreenSpaceRenderer.h
#  ClientPointRenderer.cpp
#  ClientPointRenderer.h
#  ClientPointHashReceiver.h
#  ClientPointHashReceiver.cpp
#  LZ4Compression.cpp
#  LZ4Compression.h
#  ZSTDCompression.cpp
#  ZSTDCompression.h
#  ${SHADERS}
#  )

#target_link_libraries(FalcorClient PRIVATE ${LIBS} ${LZ4_LIB_PATH} ${ZSTD_LIB_PATH})


add_falcor_executable(FalcorServer)


#add_falcor_executable(HelloDXR)

target_sources(FalcorServer PRIVATE
  argparse.hpp
  BinaryMessageType.h
  MeshPointGenerator.h
  MeshPointGenerator.cpp
  PointKDTreeGenerator.h
  PointKDTreeGenerator.cpp
  PointHashGenerator.h
  PointHashGenerator.cpp
  PointServerHashGenerator.h
  PointServerHashGenerator.cpp
  #TCPNetworkBase.h
  #TCPNetworkBase.cpp
  ServerMain.cpp
  NetworkServer.cpp
  NetworkServer.h
  NetworkCompressionBase.cpp
  NetworkCompressionBase.h
#  ServerScreenSpaceRenderer.cpp
#  ServerScreenSpaceRenderer.h
  ServerPointRenderer.cpp
  ServerPointRenderer.h
  FLIPScreenshotComparison.h
#  ../nanoflann/nanoflann.hpp
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
