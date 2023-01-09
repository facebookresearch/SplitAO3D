// (c) Meta Platforms, Inc. and affiliates. Confidential and proprietary.

#pragma once

#include <split_rendering/research_framework/Peer.h>
#include <rlr_streaming/streaming/StreamingApi.h>

namespace split_rendering {

// A very simple client implementation.
class SimpleClient : public Peer {
 public:
  SimpleClient() = delete;
  virtual ~SimpleClient() {}

  explicit SimpleClient(rlr_streaming::StreamingApi& streaming, int sendIntervalMs);

  virtual bool sendLoop() override;

  SimpleClient(const SimpleClient&) = delete;
  SimpleClient& operator=(const SimpleClient&) = delete;

 private:
  int sendIntervalMs_;
};

} // namespace split_rendering