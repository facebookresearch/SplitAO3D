/*
 * Copyright (c) Meta Platforms, Inc. and affiliates.
 * All rights reserved.
 *
 * This source code is licensed under the license found in the
 * LICENSE file in the root directory of this source tree.
 */


#pragma once

#include <rlr_streaming/streaming/StreamingApi.h>
#include <split_rendering/research_framework/Peer.h>
#include "BinaryMessageType.h"

#include <rlr_streaming/transport/tcp/TcpClient.h>

#include "TCPNetworkBase.h"

#include <gflags/gflags.h>
#include <glog/logging.h>

class ClientEventHandler : public rlr_streaming::TcpEventHandler {
 public:
  ~ClientEventHandler() {}
  void onError(const std::string& message) {
    LOG(INFO) << "Client error: " << message;
  }

  void onConnect(const char* address, unsigned short port) {
    LOG(INFO) << "Client connected to " << address << " on port " << port;
  }

  void onDisconnect() {
    LOG(INFO) << "Client disconnected";
  }

  void onListening(unsigned short port) {
    LOG(INFO) << "Client doesn't listen, should never get this event";
  }
};
namespace split_rendering {
class NetworkClient : public TCPNetworkBase {
 public:
  NetworkClient(
      const char* address,
      int port,
      ClientEventHandler& handler,
      size_t numConnectRetries = 20,
      size_t retryIntervalMs = 2000)
      : TCPNetworkBase(port, client_),
        address_(address),
        client_(handler),
        retry_info_{numConnectRetries, retryIntervalMs} {}

  ~NetworkClient();

  int establishConnection() override;

 private:
  const char* address_;
  rlr_streaming::TcpClient client_;
  rlr_streaming::TcpRetry retry_info_;
};
} // namespace split_rendering
