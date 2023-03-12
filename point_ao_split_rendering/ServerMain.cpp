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

// using rlr_streaming::IceProtocol;
// using rlr_streaming::VideoCodec;
// using rlr_streaming::WebrtcPeerInit;
// using rlr_streaming::WebrtcStreamingApi;
using split_rendering::NetworkServer;
// using split_rendering::ServerScreenSpaceRenderer;
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
#include "argparse.hpp"

using namespace Falcor;

int main(int argc, char** argv) {
  // google::InitGoogleLogging(argv[0]);
  // google::InstallFailureSignalHandler();
  // gflags::ParseCommandLineFlags(&argc, &argv, true);

  // LOG(INFO) << "Starting ...";

  argparse::ArgumentParser args("pbao_server_main");

  args.add_argument("--output_dir")
      .help("path to output directory (will be created if it doesn't exist) to store results")
      .default_value("");

  args.add_argument("--scene")
      .help("path to pyscene file")
      .default_value("test_scenes/sponza.pyscene");

  args.add_argument("--camera_path")
      .help("path to a camera trajectory (.bin) file")
      .default_value("");

  args.add_argument("--selected_renderer")
      .help("which renderer to selet (PBAO, RTAO, SSAO)")
      .default_value("PBAO");

  args.add_argument("--exit_after_camera_path")
      .help("whether or not to exit after camera path is done")
      .default_value(false).implicit_value(true);

  args.add_argument("--fullscreen")
      .help("whether or not to run in fullscreen")
      .default_value(false)
      .implicit_value(true);

  args.add_argument("--aoOnly")
      .help("whether or not to only output AO")
      .default_value(false)
      .implicit_value(true);

  args.add_argument("--export_images")
      .help("whether or not to export images to .pngs")
      .default_value(false)
      .implicit_value(true);

  args.add_argument("--aoRadius")
      .help("ao raytracing radius")
      .default_value(0.6f)
      .scan<'f', float>();
  args.add_argument("--simulatedLatencyMSec")
      .help("simulatedLatencyMSec")
      .default_value(0.0f)
      .scan<'f', float>();  
  args.add_argument("--raytracingFramerate")
      .help("rt framerate")
      .default_value(60)
      .scan<'d', int>();
  args.add_argument("--serverFramerate")
      .help("server framerate")
      .default_value(60)
      .scan<'d', int>();
  args.add_argument("--numSamplesPerUnitSquaredEliminated")
      .help("numSamplesPerUnitSquaredEliminated")
      .default_value(1024)
      .scan<'d', int>();
  args.add_argument("--minSamplesPerInstance")
      .help("minSamplesPerInstance")
      .default_value(512)
      .scan<'d', int>();
  args.add_argument("--width")
      .help("window/framebuffer width")
      .default_value(1920)
      .scan<'d', int>();
  args.add_argument("--height")
      .help("window/framebuffer height")
      .default_value(1080)
      .scan<'d', int>();

  try {
    args.parse_args(argc, argv);
  } catch (const std::runtime_error& err) {
    std::cerr << err.what() << std::endl;
    std::cerr << args;
    args.parse_known_args(argc, argv);
   // std::exit(1);
  }

  ServerEventHandler handler;

  // NetworkServer server(4001, handler);
  NetworkServer server;

  // Create Falcor GUI.
  const char* kFalcorGuiTitle = "Falcor Server";
  msgBoxTitle(kFalcorGuiTitle);
  // ServerScreenSpaceRenderer::UniquePtr falcorRenderer =
  // std::make_unique<ServerScreenSpaceRenderer>(server);
  ServerPointRenderer::UniquePtr falcorRenderer = std::make_unique<ServerPointRenderer>(server, args);
  SampleConfig falcorSampleConfig;
  // falcorSampleConfig.timeScale = 0.0001f;
  falcorSampleConfig.deviceDesc.enableDebugLayer = false;
  falcorSampleConfig.deviceDesc.enableVsync = false;
  falcorSampleConfig.windowDesc.title = kFalcorGuiTitle;
  falcorSampleConfig.windowDesc.resizableWindow = false;
  falcorSampleConfig.windowDesc.mode = args.get<bool>("--fullscreen") ? Window::WindowMode::Fullscreen : Window::WindowMode::Normal;
  falcorSampleConfig.windowDesc.width = args.get<int>("--width");
  falcorSampleConfig.windowDesc.height = args.get<int>("--height");

  // falcorSampleConfig.stereo = true;
  // server.startThreads();

  // while (server.getStatus() != rlr_streaming::TcpStatus::Connected) {
  //};
  // ServerPointRenderer test(server);
  Sample::run(falcorSampleConfig, falcorRenderer, 0, nullptr);

  // LOG(INFO) << "Existing ...";

  return 0;
}
