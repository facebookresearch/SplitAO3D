/*
 * Copyright (c) Meta Platforms, Inc. and affiliates.
 * All rights reserved.
 *
 * This source code is licensed under the license found in the
 * LICENSE file in the root directory of this source tree.
 */


#pragma once

//#include <rlr_streaming/streaming/util/AtomicQueue.h>
//#include <rlr_streaming/transport/tcp/TcpBase.h>

//#include <gflags/gflags.h>
//#include <glog/logging.h>
#include <thread>
#include <unordered_map>
#include "BinaryMessageType.h"

namespace split_rendering {

class TCPNetworkBase {
 public:
  TCPNetworkBase(int port, rlr_streaming::TcpBase& tcpConnection)
      : port_(port), tcpConnection_(tcpConnection), threadRunning_(false) {}

  virtual void setReceiveCallback(TCPMessageType type, std::function<void(TCPMessage&&)> callback);
  virtual void startThreads();
  virtual void stopThreads();
  virtual int establishConnection() = 0;

  virtual rlr_streaming::TcpStatus getStatus();
  virtual int send(TCPMessage& message);
  virtual bool tryPopFront(split_rendering::TCPMessage& messageOut);

 protected:
  int port_;
  rlr_streaming::TcpBase& tcpConnection_;
  std::thread receiverThread_;
  bool threadRunning_;
  rlr_streaming::AtomicQueue<split_rendering::TCPMessage> messageQueue_;
  std::unordered_map<TCPMessageType, std::function<void(TCPMessage&&)>> receiveCallbacks_;
};
} // namespace split_rendering
