// (c) Meta Platforms, Inc. and its affiliates
#include "LZ4Compression.h"

int LZ4Compression::compressData(
    const void* uncompressedData,
    std::vector<uint8_t>& compressedData,
    uint32_t numUncompressedBytes) {
  uint32_t maxCompressedBytes = LZ4_compressBound(numUncompressedBytes);
  compressedData.resize(maxCompressedBytes);
  return LZ4_compress_default(
      (const char*)uncompressedData,
      (char*)compressedData.data(),
      numUncompressedBytes,
      maxCompressedBytes);
}

int LZ4Compression::decompressData(
    const std::vector<uint8_t>& compressedData,
    std::vector<uint8_t>& decompressedData) {
  return LZ4_decompress_safe(
      (const char*)compressedData.data(),
      (char*)decompressedData.data(),
      compressedData.size(),
      decompressedData.size());
}
