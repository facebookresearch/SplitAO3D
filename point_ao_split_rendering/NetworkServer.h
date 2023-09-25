/*
 * Copyright (c) Meta Platforms, Inc. and affiliates.
 * All rights reserved.
 *
 * This source code is licensed under the license found in the
 * LICENSE file in the root directory of this source tree.
 */

#pragma once

#include <thread>
#include "BinaryMessageType.h"

class ServerEventHandler 
{
 public:
  ~ServerEventHandler() {}

  void onError(const std::string& message) {
    //LOG(INFO) << "Server error: " << message;
  }

  void onConnect(const char* address, unsigned short port) {
    //LOG(INFO) << "Server accepted connection from client at: " << address << " on port: " << port;
  }

  void onDisconnect() {
    //LOG(INFO) << "Server disconnected";
  }

  void onListening(unsigned short port) {
    //LOG(INFO) << "Server is listening on port: " << port;
  }
};

namespace split_rendering {

class NetworkServer// : public TCPNetworkBase {
{
 public:
 //NetworkServer(int port, ServerEventHandler& handler)
  //    : TCPNetworkBase(port, server_),
  //      server_(handler) {}

  ~NetworkServer();

  //int establishConnection() override;

 private:
  int establishConnection();
  // rlr_streaming::TcpServer server_;
};
} // namespace split_rendering
