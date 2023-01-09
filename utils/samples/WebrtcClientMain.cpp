// (c) Meta Platforms, Inc. and affiliates. Confidential and proprietary.

// Example usage:
// Assuming webrtc signaling server is running locally
// clang-format off
// $ samples/Release/WebrtcClient.exe --signaling_server_address 192.168.86.217 --signaling_server_port 8888 --peer_id_to_connect 1
// clang-format on

#include "Flags.h"
#include "KeyPress.h"
#include "SimpleClient.h"

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <rlr_streaming/streaming/WebrtcStreamingApi.h>

#ifdef _WIN32
#include <windows.h>
#endif

#include <thread>

using rlr_streaming::IceProtocol;
using rlr_streaming::VideoCodec;
using rlr_streaming::WebrtcPeerInit;
using rlr_streaming::WebrtcStreamingApi;
using split_rendering::KeyPress;
using split_rendering::RunningState;
using split_rendering::SimpleClient;

static bool sendLoopStopped{false};
static bool receiveLoopStopped{false};

int main(int argc, char* argv[]) {
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  LOG(INFO) << "Starting ...";
  LOG(INFO) << "Press 'q' to exit";

  if (FLAGS_signaling_server_address.empty() || FLAGS_signaling_server_port == 0 ||
      FLAGS_peer_id_to_connect <= 0) {
    LOG(FATAL) << "Missing required command line flag";
  }

  WebrtcStreamingApi streaming;
  WebrtcPeerInit init;
  strncpy(
      init.signalingServerAddress,
      FLAGS_signaling_server_address.c_str(),
      sizeof(init.signalingServerAddress));
  init.signalingServerAddress[sizeof(init.signalingServerAddress) - 1] = '\0';
  init.signalingServerPort = FLAGS_signaling_server_port;
  init.peerIdToConnect = FLAGS_peer_id_to_connect;
  // Use these default settings for now.
  init.iceProtocol = IceProtocol::None;
  init.minBitrateKbps = 7000;
  init.maxBitrateKbps = 8000;
  init.videoCodec = VideoCodec::Vp8;
  // Data channel for exchange arbitrary data with server.
  init.createDataChannel = true;
  streaming.init(init);

  SimpleClient client(streaming, FLAGS_send_interval_ms);
  std::thread clientSendThread([&client]() {
    client.sendLoop();
    sendLoopStopped = true;
  });
  std::thread clientReceiveThread([&client]() {
    client.receiveLoop();
    receiveLoopStopped = true;
  });

  KeyPress keyPress;
  keyPress.setup();

  while (!sendLoopStopped || !receiveLoopStopped) {
    // Do not exit this loop when 'q' is pressed because we need to keep process
    // message until WebRTC peer connection is properly closed to prevent it
    // from getting into a "stuck" state.
    if (keyPress.checkAndClear('q')) {
      client.stop();
    }
#ifdef _WIN32
    MSG msg;
    while (::PeekMessage(&msg, NULL, 0, 0, PM_REMOVE)) {
      ::TranslateMessage(&msg);
      ::DispatchMessage(&msg);
    }
#endif
  }

  keyPress.reset();

  clientSendThread.join();
  clientReceiveThread.join();

  LOG(INFO) << "Existing ...";

  return 0;
}
