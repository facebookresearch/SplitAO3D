/*
 * Copyright (c) Meta Platforms, Inc. and affiliates.
 * All rights reserved.
 *
 * This source code is licensed under the license found in the
 * LICENSE file in the root directory of this source tree.
 */

#pragma once
#include "NetworkCompressionBase.h"
#include "lz4.h"

class LZ4Compression : public NetworkCompressionBase {
 public:
  virtual int compressData(
      const void* uncompressedData,
      std::vector<uint8_t>& compressedData,
      uint32_t numUncompressedBytes);

  virtual int decompressData(
      const std::vector<uint8_t>& compressedData,
      std::vector<uint8_t>& decompressedData);

  virtual NetworkCompressionID getID() {
    return NetworkCompressionID::LZ4;
  }

};
