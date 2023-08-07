// (c) Meta Platforms, Inc. and affiliates. Confidential and proprietary.

#include "include/split_rendering/research_framework/Peer.h"

using rlr_streaming::Event;
using rlr_streaming::Message;
using rlr_streaming::MessageMetadata;
using rlr_streaming::MessageType;
using rlr_streaming::OnMessageCallback;

namespace split_rendering {

void Peer::setName(const std::string name) {
  name_ = name;
}

std::string Peer::name() const {
  return name_;
}

void Peer::setOnMessageCallback() {
  auto onMessageCallback = [this](Message message, MessageMetadata metadata) {
    StreamedMessage streamedMessage;
    streamedMessage.data.resize(message.size);
    memcpy(streamedMessage.data.data(), message.buffer, message.size);
    streamedMessage.metadata = metadata;
    if (metadata.type == MessageType::Binary) {
      binaryMessages_.push(streamedMessage);
    } else if (metadata.type == MessageType::ImageRgb8) {
      imageMessages_.push(streamedMessage);
    }
  };
  setOnMessageCallback(MessageType::Binary, onMessageCallback);
  setOnMessageCallback(MessageType::ImageRgb8, onMessageCallback);
}

void Peer::setOnMessageCallback(MessageType messageType, OnMessageCallback callback) {
  streaming_.setOnMessageCallback(messageType, callback);
}

void Peer::setOnEventCallback() {
  streaming_.setOnEventCallback([this](Event state) {
    bool notify = false;
    if (state == Event::VideoChannelOpen) {
      std::lock_guard<std::mutex> lock(channelStateMutex_);
      LOG(INFO) << "Video channel opened";
      videoChannelOpen_ = true;
      notify = videoChannelOpen_ && dataChannelOpen_;
    } else if (state == Event::DataChannelOpen) {
      std::lock_guard<std::mutex> lock(channelStateMutex_);
      LOG(INFO) << "Data channel opened";
      dataChannelOpen_ = true;
      notify = videoChannelOpen_ && dataChannelOpen_;
    }
    if (notify) {
      channelStateCv_.notify_all();
    }
  });
}

void Peer::waitForChannelsOpen() {
  std::unique_lock<std::mutex> lock(channelStateMutex_);
  channelStateCv_.wait(lock, [this] { return videoChannelOpen_ && dataChannelOpen_; });
}

bool Peer::areChannelsOpen() {
  std::lock_guard<std::mutex> lock(channelStateMutex_);
  return videoChannelOpen_ && dataChannelOpen_;
}

bool Peer::send(const Message& message, const MessageMetadata& metadata) {
  if (!areChannelsOpen()) {
    return false;
  }
  return streaming_.send(message, metadata);
}

bool Peer::sendLoop() {
  LOG(FATAL) << "Unimplemented";
}

bool Peer::receiveLoop() {
  LOG(INFO) << name_ << " receive loop started";

  waitForChannelsOpen();

  while (!stopRequested()) {
    // Receive binary message(s).
    StreamedMessage streamedMessage;
    while (binaryMessages_.tryPopFront(streamedMessage)) {
      std::string text(streamedMessage.data.size() + 1, '\0');
      memcpy(text.data(), streamedMessage.data.data(), streamedMessage.data.size());
      LOG(INFO) << name_ << " receives text: " << text;
    }

    // Receive video frame message(s).
    while (imageMessages_.tryPopFront(streamedMessage)) {
      MessageMetadata metadata = streamedMessage.metadata;
      LOG(INFO) << name_ << " receives image size " << streamedMessage.data.size() << " id "
                << metadata.id << " width " << metadata.width << " height " << metadata.height;
    }
  }

  LOG(INFO) << name_ << " receive loop finished";

  return true;
}

void Peer::stop() {
  streaming_.stop();
  std::lock_guard<std::mutex> lock(stopRequestedMutex_);
  stopRequested_ = true;
}

bool Peer::stopRequested() const {
  std::lock_guard<std::mutex> lock(stopRequestedMutex_);
  return stopRequested_;
}

} // namespace split_rendering