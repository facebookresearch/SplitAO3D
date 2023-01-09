// (c) Meta Platforms, Inc. and its affiliates. Confidential and proprietary.

#pragma once

#include "StreamingApi.h"
#include "util/AtomicQueue.h"

#include <glog/logging.h>
#include <atomic>

namespace rlr_streaming {

struct QueueMessage {
  std::vector<uint8_t> data;
  MessageMetadata metadata;
};

// In memory implementation of the streaming API
class InMemoryStreamingApi final : public StreamingApi {
 public:
  InMemoryStreamingApi() = delete;
  virtual ~InMemoryStreamingApi() {}

  InMemoryStreamingApi(
      AtomicQueue<QueueMessage>& sendQueue,
      AtomicQueue<QueueMessage>& receiveQueue)
      : sendQueue_(sendQueue), receiveQueue_(receiveQueue) {}

  bool send(const Message& message, const MessageMetadata& metadata) override;

  // Monitor internal queue and issue callbacks for returning messages back to app.
  void run();

  InMemoryStreamingApi(const InMemoryStreamingApi&) = delete;
  InMemoryStreamingApi& operator=(const InMemoryStreamingApi&) = delete;

 private:
  // Queue for outgoing messages.
  AtomicQueue<QueueMessage>& sendQueue_;
  // Queue for incoming messages.
  AtomicQueue<QueueMessage>& receiveQueue_;
};

} // namespace rlr_streaming