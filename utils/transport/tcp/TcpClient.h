// (c) Meta Platforms, Inc. and its affiliates. Confidential and proprietary.

#pragma once

#include "TcpBase.h"

#include <memory>

namespace rlr_streaming {

class TcpClient : public TcpBase {
 public:
  TcpClient() = delete;
  explicit TcpClient(TcpEventHandler& handler);
  virtual ~TcpClient();
  TcpClient(TcpClient&&) = delete;
  TcpClient(const TcpClient&) = delete;
  TcpClient& operator=(TcpClient&&) = delete;
  TcpClient& operator=(const TcpClient&) = delete;

  // Override methods in TcpBase.
  TcpStatus getStatus() override;
  bool send(const void* buffer, size_t size) override;
  bool receive(void* buffer, size_t size) override;
  void setBlockSize(size_t blockSize) override;
  bool disconnect() override;

  // TcpClient unqiue method.
  bool connect(const char* address, unsigned short port, TcpRetry retry);

 private:
  class TcpClientImpl;
  std::unique_ptr<TcpClientImpl> impl_;
};

} // namespace rlr_streaming
