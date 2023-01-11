// (c) Meta Platforms, Inc. and affiliates. Confidential and proprietary.

#include "SimpleClient.h"

#include <glog/logging.h>

#include <chrono>
#include <thread>

using rlr_streaming::Event;
using rlr_streaming::Message;
using rlr_streaming::MessageMetadata;
using rlr_streaming::MessageType;
using rlr_streaming::StreamingApi;

namespace split_rendering {

SimpleClient::SimpleClient(StreamingApi& streaming, int sendIntervalMs)
    : Peer(streaming), sendIntervalMs_(sendIntervalMs) {
  setName("SimpleClient");
  setOnEventCallback();
  setOnMessageCallback();
}

bool SimpleClient::sendLoop() {
  LOG(INFO) << name() << " send loop started";

  waitForChannelsOpen();

  while (!stopRequested()) {
    uint64_t sendStartMs = std::chrono::duration_cast<std::chrono::milliseconds>(
                               std::chrono::system_clock::now().time_since_epoch())
                               .count();

    // Send a text message.
    static int i = 0;
    std::string text = std::to_string(i) + " th message from " + name();
    Message message(text.data(), text.size());
    MessageMetadata metadata;
    metadata.type = MessageType::Binary;
    LOG(INFO) << name() << " sending text message: " << text;
    if (!send(message, metadata)) {
      LOG(ERROR) << name() << " failed to send text message";
    }
    i++;

    uint64_t sendEndMs = std::chrono::duration_cast<std::chrono::milliseconds>(
                             std::chrono::system_clock::now().time_since_epoch())
                             .count();
    uint64_t sleepMs = sendIntervalMs_ - (sendEndMs - sendStartMs);
    if (sleepMs > 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(sleepMs));
    }
  }

  LOG(INFO) << name() << " send loop finished";

  return true;
}

} // namespace split_rendering