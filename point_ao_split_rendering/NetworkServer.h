// (c) Meta Platforms, Inc. and affiliates. Confidential and proprietary.

#pragma once

#include <rlr_streaming/transport/tcp/TcpBase.h>
#include <rlr_streaming/transport/tcp/TcpServer.h>
#include "TCPNetworkBase.h"

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <thread>
#include "BinaryMessageType.h"

class ServerEventHandler : public rlr_streaming::TcpEventHandler {
 public:
  ~ServerEventHandler() {}

  void onError(const std::string& message) {
    LOG(INFO) << "Server error: " << message;
  }

  void onConnect(const char* address, unsigned short port) {
    LOG(INFO) << "Server accepted connection from client at: " << address << " on port: " << port;
  }

  void onDisconnect() {
    LOG(INFO) << "Server disconnected";
  }

  void onListening(unsigned short port) {
    LOG(INFO) << "Server is listening on port: " << port;
  }
};

namespace split_rendering {

class NetworkServer : public TCPNetworkBase {
 public:
  NetworkServer(int port, ServerEventHandler& handler)
      : TCPNetworkBase(port, server_),
        server_(handler) {}

  ~NetworkServer();

  int establishConnection() override;

 private:
  rlr_streaming::TcpServer server_;
};
} // namespace split_rendering