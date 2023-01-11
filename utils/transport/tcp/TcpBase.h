// (c) Meta Platforms, Inc. and its affiliates. Confidential and proprietary.

#pragma once

#include "TcpTypes.h"

#include <stddef.h>

namespace rlr_streaming {

class TcpBase {
 public:
  TcpBase() = default;
  virtual ~TcpBase() = default;
  TcpBase(TcpBase&&) = delete;
  TcpBase(const TcpBase&) = delete;
  TcpBase& operator=(TcpBase&&) = delete;
  TcpBase& operator=(const TcpBase&) = delete;

  virtual TcpStatus getStatus() = 0;
  virtual bool send(const void* buffer, size_t size) = 0;
  virtual bool receive(void* buffer, size_t size) = 0;
  virtual void setBlockSize(size_t blockSize) = 0;
  virtual bool disconnect() = 0;
};

} // namespace rlr_streaming