// (c) Meta Platforms, Inc. and affiliates. Confidential and proprietary.

#pragma once

#include <rlr_streaming/streaming/util/AtomicQueue.h>
#include <rlr_streaming/streaming/StreamingApi.h>

#include <atomic>
#include <condition_variable>

namespace split_rendering {

struct StreamedMessage {
  std::vector<uint8_t> data;
  rlr_streaming::MessageMetadata metadata;
};

enum class RunningState : uint8_t { RunningStateUnknown = 0, Unstarted, Running, Stopped };

// Base class for Server and Client.
class Peer {
 public:
  Peer() = delete;
  virtual ~Peer() {}

  // Caller retain ownership of |streaming| and need to ensure it is valid
  // throughout the lifecycle of Peer.
  explicit Peer(rlr_streaming::StreamingApi& streaming) : streaming_(streaming) {}

  // Name for identifying this class, e.g., used in logging.
  virtual std::string name() const;

  virtual bool send(
      const rlr_streaming::Message& message,
      const rlr_streaming::MessageMetadata& metadata);

  // Looping for processing, e.g., send / receive.
  virtual bool sendLoop();
  virtual bool receiveLoop();

  // Request to stop processing.
  virtual void stop();

  Peer(const Peer&) = delete;
  Peer& operator=(const Peer&) = delete;

 protected:
  virtual void setName(const std::string name);

  // Set callbacks.
  virtual void setOnEventCallback();
  virtual void setOnMessageCallback();
  virtual void setOnMessageCallback(
      rlr_streaming::MessageType messageType,
      rlr_streaming::OnMessageCallback callback);

  // Will block the thread waiting for all the channels are open.
  virtual void waitForChannelsOpen();

  virtual bool areChannelsOpen();

  virtual bool stopRequested() const;

  // Received messages.
  rlr_streaming::AtomicQueue<StreamedMessage> binaryMessages_;
  rlr_streaming::AtomicQueue<StreamedMessage> imageMessages_;

 private:
  // Name assigned to the class or its sub-classes.
  std::string name_;

  rlr_streaming::StreamingApi& streaming_;

  // Make sure channelStateMutex_ is locked when updating any channel state
  // related variables below.
  bool videoChannelOpen_{false};
  bool dataChannelOpen_{false};
  std::mutex channelStateMutex_;
  std::condition_variable channelStateCv_;

  bool stopRequested_{false};
  mutable std::mutex stopRequestedMutex_;
};

} // namespace split_rendering