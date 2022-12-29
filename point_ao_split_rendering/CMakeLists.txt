# This CMakeLists.txt is based on apps\LEm\CMakeLists.txt

# Use C++17 unless overridden by -DCMAKE_CXX_STANDARD on the command line
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

set(FALCOR_ROOT_DIRECTORY
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../shared/third-party/Falcor/5.0/Falcor")
set(FALCOR_MOGWAILIBS
    ON
    CACHE BOOL "" FORCE)
add_subdirectory("${FALCOR_ROOT_DIRECTORY}" Falcor)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
set(CMAKE_MODULE_PATH "${FALCOR_ROOT_DIRECTORY}/cmake" ${CMAKE_MODULE_PATH})
set(LZ4_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/../../../shared/third-party/lz4/1.9.3#3/lz4")
set(LZ4_LIB_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../../../shared/third-party/lz4/1.9.3#3/lz4/lib/lz4.lib")
set(ZSTD_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/../zstd-v1.5.2-win64")
set(ZSTD_LIB_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../zstd-v1.5.2-win64/static/libzstd_static.lib")

include(FalcorUtility)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../../split_rendering/)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../poisson_sampling/)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../nanoflann/)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../flip/)
include_directories(${ZSTD_DIRECTORY}/include/)
include_directories(${LZ4_DIRECTORY}/include/)

if(WIN32)
  # In order to use just main(), not WinMain() on Windows
  # https://stackoverflow.com/questions/45724766/how-to-set-c-application-entry-point-to-main-on-windows-using-cmake
  set(CMAKE_EXE_LINKER_FLAGS
      "${CMAKE_EXE_LINKER_FLAGS} /SUBSYSTEM:WINDOWS /ENTRY:mainCRTStartup")
endif()
	
set(LIBS 
	rlr_tcp 
	research_framework
	rlr_streaming
	glog
	gflags
	)
	
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
  )

falcor_sample(
  NAME
  FalcorServer
  SOURCES
  BinaryMessageType.h
  MeshPointGenerator.h
  MeshPointGenerator.cpp
  PointKDTreeGenerator.h
  PointKDTreeGenerator.cpp
  PointHashGenerator.h
  PointHashGenerator.cpp
  PointServerHashGenerator.h
  PointServerHashGenerator.cpp
  TCPNetworkBase.h
  TCPNetworkBase.cpp
  ServerMain.cpp
  NetworkServer.cpp
  NetworkServer.h
  NetworkCompressionBase.cpp
  NetworkCompressionBase.h
  ServerScreenSpaceRenderer.cpp
  ServerScreenSpaceRenderer.h
  ServerPointRenderer.cpp
  ServerPointRenderer.h
  FLIPScreenshotComparison.h
  ../nanoflann/nanoflann.hpp
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

falcor_sample(
  NAME
  FalcorClient
  SOURCES
  BinaryMessageType.h
  TCPNetworkBase.h
  TCPNetworkBase.cpp
  ClientMain.cpp
  NetworkClient.cpp
  NetworkClient.h
  NetworkCompressionBase.cpp
  NetworkCompressionBase.h
  ClientScreenSpaceRenderer.cpp
  ClientScreenSpaceRenderer.h
  ClientPointRenderer.cpp
  ClientPointRenderer.h
  ClientPointHashReceiver.h
  ClientPointHashReceiver.cpp
  LZ4Compression.cpp
  LZ4Compression.h
  ZSTDCompression.cpp
  ZSTDCompression.h
  ${SHADERS}
  )

target_link_libraries(FalcorClient PRIVATE ${LIBS} ${LZ4_LIB_PATH} ${ZSTD_LIB_PATH})
