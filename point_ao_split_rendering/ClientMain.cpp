// (c) Meta Platforms, Inc. and affiliates. Confidential and proprietary.

// Example usage:
// clang-format off
// $ ./bin/Release/FalcorClient.exe -signaling_server_address 192.168.86.217 -signaling_server_port 8888 -peer_id_to_connect 1
// clang-format on

#include "NetworkClient.h"
#include "ClientPointRenderer.h"
#include "samples/Flags.h"

#include <Falcor.h>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <rlr_streaming/streaming/WebrtcStreamingApi.h>
#include <rlr_streaming/streaming/util/AtomicQueue.h>
#include <split_rendering/research_framework/Peer.h>

#ifdef _WIN32
#include <windows.h>
#endif

#include <thread>

using rlr_streaming::IceProtocol;
using rlr_streaming::VideoCodec;
using rlr_streaming::WebrtcPeerInit;
using rlr_streaming::WebrtcStreamingApi;
using split_rendering::NetworkClient;
using split_rendering::ClientPointRenderer;
using split_rendering::RunningState;
using split_rendering::StreamedMessage;

// (c) Meta Platforms, Inc. and its affiliates. Confidential and proprietary.

int main(int argc, char* argv[]) {
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  LOG(INFO) << "Starting ...";
  ClientEventHandler handler;

  NetworkClient client("::1", 4001, handler);

  // Create Falcor GUI.
  const char* kFalcorGuiTitle = "Falcor Client";
  msgBoxTitle(kFalcorGuiTitle);
  ClientPointRenderer::UniquePtr falcorRenderer = std::make_unique<ClientPointRenderer>(client);
  SampleConfig falcorSampleConfig;
  falcorSampleConfig.windowDesc.title = kFalcorGuiTitle;
  falcorSampleConfig.windowDesc.resizableWindow = true;
  falcorSampleConfig.stereo = false;
  Sample::run(falcorSampleConfig, falcorRenderer, 0, nullptr);

  LOG(INFO) << "Existing ...";

  return 0;
}
