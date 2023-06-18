// (c) Meta Platforms, Inc. and its affiliates
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
