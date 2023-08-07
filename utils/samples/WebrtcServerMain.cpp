// (c) Meta Platforms, Inc. and affiliates. Confidential and proprietary.

// Example usage:
// Assuming webrtc signaling server is running locally
// clang-format off
// $ samples/Release/WebrtcServer.exe --signaling_server_address 192.168.86.217 --signaling_server_port 8888 --ice_protocol udp --ice_port 8000 --rgb_file ~/media-files/CSGO_short_24.rgb --width 1920 --height 1080
// clang-format on

#include "Flags.h"
#include "KeyPress.h"
#include "SimpleServer.h"

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
using split_rendering::SimpleServer;

static bool sendLoopStopped{false};
static bool receiveLoopStopped{false};

int main(int argc, char* argv[]) {
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  LOG(INFO) << "Starting ...";
  LOG(INFO) << "Press 'q' to exit";

  LOG_IF(
      FATAL,
      FLAGS_signaling_server_address.empty() || FLAGS_signaling_server_port == 0 ||
          FLAGS_ice_protocol.empty() || FLAGS_ice_port == 0)
      << "Missing / Invalid required command line flag";

  LOG_IF(FATAL, FLAGS_rgb_file.empty() || FLAGS_width <= 0 || FLAGS_height <= 0)
      << "Missing / Invalid command line flag for RGB file";

  WebrtcStreamingApi streaming;
  WebrtcPeerInit init;
  strncpy(
      init.signalingServerAddress,
      FLAGS_signaling_server_address.c_str(),
      sizeof(init.signalingServerAddress));
  init.signalingServerAddress[sizeof(init.signalingServerAddress) - 1] = '\0';
  init.signalingServerPort = FLAGS_signaling_server_port;
  if (FLAGS_ice_protocol == "any") {
    init.iceProtocol = IceProtocol::Any;
  } else if (FLAGS_ice_protocol == "tcp") {
    init.iceProtocol = IceProtocol::Tcp;
  } else if (FLAGS_ice_protocol == "udp") {
    init.iceProtocol = IceProtocol::Udp;
  } else if (FLAGS_ice_protocol == "none") {
    init.iceProtocol = IceProtocol::None;
  } else {
    LOG(FATAL) << "Unrecognized ice protocol: " << FLAGS_ice_protocol;
  }
  init.icePort = FLAGS_ice_port;
  // Video track for sending video frames to client.
  init.createVideoTrack = true;
  streaming.init(init);

  SimpleServer server(streaming, FLAGS_send_interval_ms);
  server.setRgbFile(FLAGS_rgb_file, FLAGS_width, FLAGS_height);
  std::thread serverSendThread([&server]() {
    server.sendLoop();
    sendLoopStopped = true;
  });
  std::thread serverReceiveThread([&server]() {
    server.receiveLoop();
    receiveLoopStopped = true;
  });

  KeyPress keyPress;
  keyPress.setup();

  while (!sendLoopStopped || !receiveLoopStopped) {
    // Do not exit this loop when 'q' is pressed because we need to keep process
    // message until WebRTC peer connection is properly closed to prevent it
    // from getting into a "stuck" state.
    if (keyPress.checkAndClear('q')) {
      server.stop();
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

  serverSendThread.join();
  serverReceiveThread.join();

  LOG(INFO) << "Existing ...";

  return 0;
}
