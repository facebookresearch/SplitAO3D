// (c) Meta Platforms, Inc. and affiliates. Confidential and proprietary.

// Example usage:
// To just stream text message
// $ ./samples/Release/InProcessStreaming.exe
// To stream both text message and video frame
// clang-format off
// $ ./samples/Release/InProcessStreaming.exe --rgb_file ~/media-files/CSGO_short_24.rgb --width 1920 --height 1080
// clang-format on

#include "Flags.h"
#include "KeyPress.h"
#include "SimpleClient.h"
#include "SimpleServer.h"

#include <rlr_streaming/streaming/util/AtomicQueue.h>
#include <rlr_streaming/streaming/InMemoryStreamingApi.h>
#include <glog/logging.h>
#include <gflags/gflags.h>

#ifdef _WIN32
#include <windows.h>
#endif

#include <atomic>
#include <thread>

using rlr_streaming::AtomicQueue;
using rlr_streaming::InMemoryStreamingApi;
using rlr_streaming::QueueMessage;
using split_rendering::KeyPress;
using split_rendering::SimpleClient;
using split_rendering::SimpleServer;

int main(int argc, char* argv[]) {
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  LOG(INFO) << "Starting ...";
  LOG(INFO) << "Press 'q' to exit";

  AtomicQueue<QueueMessage> serverToClientQueue;
  AtomicQueue<QueueMessage> clientToServerQueue;

  InMemoryStreamingApi serverStreaming(
      serverToClientQueue /* sendQueue */, clientToServerQueue /* receiveQueue */);
  SimpleServer server(serverStreaming, FLAGS_send_interval_ms);
  if (!FLAGS_rgb_file.empty() && FLAGS_width > 0 && FLAGS_height > 0) {
    server.setRgbFile(FLAGS_rgb_file, FLAGS_width, FLAGS_height);
  }
  std::thread serverSendThread([&server]() { server.sendLoop(); });
  std::thread serverReceiveThread([&server]() { server.receiveLoop(); });
  std::thread serverStreamingThread([&serverStreaming]() {
    // For issuing callbacks for returning received messages.
    serverStreaming.run();
  });

  InMemoryStreamingApi clientStreaming(
      clientToServerQueue /* sendQueue */, serverToClientQueue /* receiveQueue */);
  SimpleClient client(clientStreaming, FLAGS_send_interval_ms);
  std::thread clientSendThread([&client]() { client.sendLoop(); });
  std::thread clientReceiveThread([&client]() { client.receiveLoop(); });
  std::thread clientStreamingThread([&clientStreaming]() {
    // For issuing callbacks for returning received messages.
    clientStreaming.run();
  });

  KeyPress keyPress;
  keyPress.setup();

  while (!keyPress.checkAndClear('q')) {
#ifdef _WIN32
    MSG msg;
    while (::PeekMessage(&msg, NULL, 0, 0, PM_REMOVE)) {
      ::TranslateMessage(&msg);
      ::DispatchMessage(&msg);
    }
#endif
  }

  keyPress.reset();

  server.stop();
  client.stop();

  serverSendThread.join();
  serverReceiveThread.join();
  serverStreamingThread.join();
  clientSendThread.join();
  clientReceiveThread.join();
  clientStreamingThread.join();

  LOG(INFO) << "Existing ...";

  return 0;
}
