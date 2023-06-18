// (c) Meta Platforms, Inc. and its affiliates
#pragma once
#include "NetworkCompressionBase.h"
#include "zstd.h"

class ZSTDCompression : public NetworkCompressionBase {
 public:
  ZSTDCompression() {
    initContext(20, 4, 0);
  }

  ZSTDCompression(
      uint32_t compressionLevel,
      uint32_t compressionStrategy,
      uint32_t numWorkers = 0) {
    initContext(compressionLevel, compressionStrategy, numWorkers);
  }

  ~ZSTDCompression() {
    ZSTD_freeCCtx(zstdCompressionContext_);
  }

  virtual int compressData(
      const void* uncompressedData,
      std::vector<uint8_t>& compressedData,
      uint32_t numUncompressedBytes);

  virtual int decompressData(
      const std::vector<uint8_t>& compressedData,
      std::vector<uint8_t>& decompressedData);

  virtual NetworkCompressionID getID() {
    return NetworkCompressionID::ZSTD;
  }

 private:
  void initContext(uint32_t compressionLevel, uint32_t compressionStrategy, uint32_t numWorkers) {
    zstdCompressionContext_ = ZSTD_createCCtx();
    zstdDecompressionContext_ = ZSTD_createDCtx();
    ZSTD_CCtx_setParameter(zstdCompressionContext_, ZSTD_c_nbWorkers, numWorkers);
    ZSTD_CCtx_setParameter(zstdCompressionContext_, ZSTD_c_compressionLevel, compressionLevel);
    ZSTD_CCtx_setParameter(zstdCompressionContext_, ZSTD_c_strategy, compressionStrategy);
  }

  ZSTD_CCtx* zstdCompressionContext_;
  ZSTD_DCtx* zstdDecompressionContext_;
};
