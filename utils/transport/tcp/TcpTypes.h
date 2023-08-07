// (c) Meta Platforms, Inc. and its affiliates. Confidential and proprietary.

#pragma once

#include <cstddef>
#include <string>

namespace rlr_streaming {

enum struct TcpStatus : int {
  Unknown = 0,
  Connected,
  Disconnected,
};

class TcpRetry {
 public:
  TcpRetry(size_t numOfRetries, size_t intervalMs)
      : numOfRetries(numOfRetries), intervalMs(intervalMs) {}

  bool operator==(const TcpRetry& b) const {
    return numOfRetries == b.numOfRetries && intervalMs == b.intervalMs;
  }

  size_t numOfRetries;
  size_t intervalMs;
};

class TcpEventHandler {
 public:
  virtual void onError(const std::string& message) = 0;
  virtual void onConnect(const char* address, unsigned short port) = 0;
  virtual void onListening(unsigned short port) = 0;
  virtual void onDisconnect() = 0;
};

} // namespace rlr_streaming