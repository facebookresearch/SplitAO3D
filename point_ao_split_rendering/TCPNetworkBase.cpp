/*
 * Copyright (c) Meta Platforms, Inc. and affiliates.
 * All rights reserved.
 *
 * This source code is licensed under the license found in the
 * LICENSE file in the root directory of this source tree.
 */


#include "TCPNetworkBase.h"

namespace split_rendering {

void TCPNetworkBase::setReceiveCallback(
    TCPMessageType type,
    std::function<void(TCPMessage&&)> callback) {
  receiveCallbacks_[type] = callback;
}

void TCPNetworkBase::startThreads() {
  threadRunning_ = true;

  receiverThread_ = std::thread([&]() {
    establishConnection();

    while (tcpConnection_.getStatus() != rlr_streaming::TcpStatus::Connected) {
      if (threadRunning_ == false)
        return 0;

      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    while (tcpConnection_.getStatus() == rlr_streaming::TcpStatus::Connected) {
      if (threadRunning_ == false)
        return 0;

      split_rendering::TCPMessage message;

      if (!tcpConnection_.receive(&message.header, sizeof(message.header))) {
        LOG(ERROR) << "Failed to read message size";
        tcpConnection_.disconnect();
        return -1;
      }

      if ((uint32_t)message.header.type > (uint32_t)TCPMessageType::NumberOfMessageTypes) {
        LOG(ERROR) << "Invalid message ID in received message";
        tcpConnection_.disconnect();
        return -1;
      }

      message.data.resize(message.header.size);
      if (!tcpConnection_.receive(message.data.data(), message.header.size)) {
        LOG(ERROR) << "Failed to read message";
        tcpConnection_.disconnect();
        return -1;
      }

      // If a receive callback exists, we call it (e.g. when we only want to keep the newest
      // message. Otherwise we push to the queue for reliable messaging.
      if (receiveCallbacks_.find(message.header.type) != receiveCallbacks_.end()) {
        receiveCallbacks_[message.header.type](std::move(message));
      } else {
        messageQueue_.push(std::move(message));
      }
    }
  });
}

rlr_streaming::TcpStatus TCPNetworkBase::getStatus() {
  return tcpConnection_.getStatus();
}

void TCPNetworkBase::stopThreads() {
  threadRunning_ = false;
  tcpConnection_.disconnect();
  receiverThread_.join();
}

bool TCPNetworkBase::tryPopFront(split_rendering::TCPMessage& messageOut) {
  return messageQueue_.tryPopFront(messageOut);
}

int TCPNetworkBase::send(TCPMessage& message) {
  if (tcpConnection_.getStatus() != rlr_streaming::TcpStatus::Connected)
    return 0;

  // Send header
  if (!tcpConnection_.send(&message.header, sizeof(message.header))) {
    LOG(ERROR) << "Failed to send message size";
    tcpConnection_.disconnect();
    return -1;
  };
  LOG(INFO) << "Message size: " << message.header.size;

  // Send data
  if (!tcpConnection_.send(message.data.data(), message.header.size)) {
    LOG(ERROR) << "Failed to send message";
    tcpConnection_.disconnect();
    return -1;
  };

  return 0;
}

} // namespace split_rendering
