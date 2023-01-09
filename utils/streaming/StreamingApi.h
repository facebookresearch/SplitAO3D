// (c) Meta Platforms, Inc. and its affiliates. Confidential and proprietary.

#pragma once

#include <glog/logging.h>
#include <atomic>
#include <functional>
#include <mutex>

namespace rlr_streaming {

enum class MessageType : uint8_t {
  MessageTypeUnknown = 0,
  // Opaque blob data that is streamed as raw bytes without specialized processing.
  Binary,
  // Image data that has 8 bit depth and 3 channels, customized and optimized processing (encoding
  // and decoding) and streaming channel is expected for streaming implementation to delivery high
  // performance for streaming such message type.
  ImageRgb8,
  NumberOfMessageTypes // Keep this last.
};

// Metadata describing the message we send or receive.
struct MessageMetadata {
  MessageType type;
  // id is an identifier that Streaming API implementation **might** choose to propogate from sender
  // to receiver. Example use cases are: (1) Identify and associate source and received frame for
  // streaming quality evaluation or latency calculation (2) Sending different video and binary data
  // via different channels but need to have an association between each video frame with its
  // corresponding binary data for performing certain rendering operation on the receiver side.
  uint32_t id;
  size_t width; // Only used for image data message.
  size_t height; // Only used for image data message.
};

// Represent a message that is sent or received over the network.
struct Message {
  Message(const void* buffer, size_t size) {
    this->buffer = buffer;
    this->size = size;
  }
  const void* buffer;
  size_t size;
};

// Represent an event of the underlying communication channel.
// Connection events are modeled after https://w3c.github.io/webrtc-pc/#rtcpeerconnectionstate-enum
enum class Event : uint8_t {
  EventUnknown = 0, // Default unknown state.

  // State of underlying connection.
  ConnectionNew,
  ConnectionConnecting,
  ConnectionConnected,
  ConnectionDisconnected,
  ConnectionFailed,
  ConnectionClosed,

  // State of video streaming channel.
  VideoChannelConnecting,
  VideoChannelOpen,
  VideoChannelClosing,
  VideoChannelClosed,

  // State of data channel for sending arbitrary data.
  DataChannelConnecting,
  DataChannelOpen,
  DataChannelClosing,
  DataChannelClosed,
};

// Callback function for returning a |Message|.
using OnMessageCallback =
    std::function<void(const Message& message, const MessageMetadata& metadata)>;

// Callback function for returning an |Event|.
using OnEventCallback = std::function<void(const Event event)>;

// Streaming API developed by FRL Research Graphics team.
// Designed to support streaming of video, graphic data or arbitrary binary data.
class StreamingApi {
 public:
  StreamingApi() = default;
  virtual ~StreamingApi() {}

  // Override this to implement how to send messages to the other party synchronously.
  // Return whether the send operation was successful.
  virtual bool send(const Message& message, const MessageMetadata& metadata) {
    LOG(FATAL) << "Unimplemented";
    return true;
  }

  virtual void stop() {
    stop_.store(true, std::memory_order_release);
  }

  // Set callback for receiving an |Event| from streaming connection.
  virtual void setOnEventCallback(OnEventCallback callback) {
    const std::lock_guard<std::mutex> lock(onEventCallbackMutex_);
    onEventCallback_ = callback;
  }

  // Set callback for receiving a message of |type| from streaming connection.
  virtual void setOnMessageCallback(MessageType type, OnMessageCallback callback) {
    const std::lock_guard<std::mutex> lock(onMessageCallbacksMutex_);
    onMessageCallbacks_[static_cast<int>(type)] = callback;
  }

  StreamingApi(const StreamingApi&) = delete;
  StreamingApi& operator=(const StreamingApi&) = delete;

 protected:
  // An array of callbacks one for each |MessageType|, indexed by |MessageType|.
  OnMessageCallback onMessageCallbacks_[static_cast<int>(MessageType::NumberOfMessageTypes)];
  std::mutex onMessageCallbacksMutex_;
  // Single callback for returning an |Event|.
  OnEventCallback onEventCallback_;
  std::mutex onEventCallbackMutex_;

  std::atomic<bool> stop_{false};
};

} // namespace rlr_streaming
