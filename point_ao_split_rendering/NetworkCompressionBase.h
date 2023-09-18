/*
 * Copyright (c) Meta Platforms, Inc. and affiliates.
 * All rights reserved.
 *
 * This source code is licensed under the license found in the
 * LICENSE file in the root directory of this source tree.
 */

#pragma once
#include <stdint.h>
#include <memory>
#include <stdexcept>
#include <vector>

enum class NetworkCompressionID { LZ4 = 0, ZSTD, NUM_IDS };

class NetworkCompressionBase {
 public:
  // This method returns the number of compressed bytes, which are <= the size of the compressedData
  // to prevent a copy.
  virtual int compressData(
      const void* uncompressedData,
      std::vector<uint8_t>& compressedData,
      uint32_t numUncompressedBytes) = 0;

  virtual int decompressData(
      const std::vector<uint8_t>& compressedData,
      std::vector<uint8_t>& decompressedData) = 0;

  virtual NetworkCompressionID getID() = 0;

  static std::unique_ptr<NetworkCompressionBase> getDerived(NetworkCompressionID id);
};
