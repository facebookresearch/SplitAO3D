// (c) Meta Platforms, Inc. and affiliates. Confidential and proprietary.

// Example usage:
// clang-format off
// $ ./bin/Release/FalcorServer.exe -signaling_server_address 192.168.86.217 -signaling_server_port 8888 -ice_protocol udp -ice_port 8000
// clang-format on

// (c) Meta Platforms, Inc. and its affiliates. Confidential and proprietary.

//#include <rlr_streaming/transport/tcp/TcpServer.h>

//#include <gflags/gflags.h>
//#include <glog/logging.h>

#include "NetworkServer.h"
//#include "ServerScreenSpaceRenderer.h"
#include "ServerPointRenderer.h"
//#include "samples/Flags.h"

#include <Falcor.h>
//#include <glog/logging.h>
//#include <rlr_streaming/streaming/WebrtcStreamingApi.h>
//#include <rlr_streaming/transport/tcp/TcpServer.h>

#ifdef _WIN32
#include <windows.h>
#endif

#include <atomic>
#include <thread>

//using rlr_streaming::IceProtocol;
//using rlr_streaming::VideoCodec;
//using rlr_streaming::WebrtcPeerInit;
//using rlr_streaming::WebrtcStreamingApi;
using split_rendering::NetworkServer;
//using split_rendering::ServerScreenSpaceRenderer;
using split_rendering::ServerPointRenderer;


// (c) Meta Platforms, Inc. and affiliates. Confidential and proprietary.

// Based on
// shared\third-party\Falcor\4.1\Falcor\Source\Samples\HelloDXR\HelloDXR.h

#pragma once

#include "NetworkServer.h"
#include "PointCellAllocationStage.h"
#include "PointCloudVisualizationPass.h"

#include <Falcor.h>
#include "PointData.slang"
#include "Rendering/Lights/EnvMapLighting.h"
#include "ScreenshotCaptureHelper.h"

#include "MeshPointGenerator.h"
#include "PointHashGenerator.h"
#include "PointKDTreeGenerator.h"
#include "PointServerHashGenerator.h"

using namespace Falcor;

int main(int argc, char** argv) {
  //google::InitGoogleLogging(argv[0]);
  //google::InstallFailureSignalHandler();
  //gflags::ParseCommandLineFlags(&argc, &argv, true);

  //LOG(INFO) << "Starting ...";


  ServerEventHandler handler;

  //NetworkServer server(4001, handler);
  NetworkServer server;

  // Create Falcor GUI.
  const char* kFalcorGuiTitle = "Falcor Server";
  msgBoxTitle(kFalcorGuiTitle);
  //ServerScreenSpaceRenderer::UniquePtr falcorRenderer = std::make_unique<ServerScreenSpaceRenderer>(server);
  ServerPointRenderer::UniquePtr falcorRenderer =
      std::make_unique<ServerPointRenderer>(server);
  SampleConfig falcorSampleConfig;
  //falcorSampleConfig.timeScale = 0.0001f;
  falcorSampleConfig.deviceDesc.enableDebugLayer = false;
  falcorSampleConfig.deviceDesc.enableVsync = false;
  falcorSampleConfig.windowDesc.title = kFalcorGuiTitle;
  falcorSampleConfig.windowDesc.resizableWindow = true;
  //falcorSampleConfig.stereo = true;
  //server.startThreads();

  //while (server.getStatus() != rlr_streaming::TcpStatus::Connected) {
  //};

  Sample::run(falcorSampleConfig, falcorRenderer, 0, nullptr);

  //LOG(INFO) << "Existing ...";

  return 0;
}
