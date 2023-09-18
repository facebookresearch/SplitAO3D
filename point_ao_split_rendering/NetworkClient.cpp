/*
 * Copyright (c) Meta Platforms, Inc. and affiliates.
 * All rights reserved.
 *
 * This source code is licensed under the license found in the
 * LICENSE file in the root directory of this source tree.
 */


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
