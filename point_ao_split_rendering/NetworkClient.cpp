// (c) Meta Platforms, Inc. and affiliates. Confidential and proprietary.

#include "NetworkClient.h"

#include <glog/logging.h>
#include <rlr_streaming/streaming/StreamingApi.h>

#include <chrono>
#include <thread>

namespace split_rendering {
NetworkClient::~NetworkClient() {
  stopThreads();
}

int NetworkClient::establishConnection() {
  if (!client_.connect(address_, port_, retry_info_)) {
    LOG(ERROR) << "Error. Could not connect to server: " << address_ << " port: " << port_
               << std::endl;
    return -1;
  }

  return 0;
}
} // namespace split_rendering