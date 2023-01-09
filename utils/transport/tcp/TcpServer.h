// (c) Meta Platforms, Inc. and its affiliates. Confidential and proprietary.

#pragma once

#include "TcpBase.h"

#include <memory>

namespace rlr_streaming {

class TcpServer : public TcpBase {
 public:
  TcpServer() = delete;
  explicit TcpServer(TcpEventHandler& handler);
  virtual ~TcpServer();
  TcpServer(TcpServer&&) = delete;
  TcpServer(const TcpServer&) = delete;
  TcpServer& operator=(TcpServer&&) = delete;
  TcpServer& operator=(const TcpServer&) = delete;

  // Override methods in TcpBase.
  TcpStatus getStatus() override;
  bool send(const void* buffer, size_t size) override;
  bool receive(void* buffer, size_t size) override;
  void setBlockSize(size_t blockSize) override;
  bool disconnect() override;

  // TcpServer unqiue method.
  bool acceptConnection(unsigned short port);

 private:
  class TcpServerImpl;
  std::unique_ptr<TcpServerImpl> impl_;
};

} // namespace rlr_streaming
