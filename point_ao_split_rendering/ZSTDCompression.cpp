/*
 * Copyright (c) Meta Platforms, Inc. and affiliates.
 * All rights reserved.
 *
 * This source code is licensed under the license found in the
 * LICENSE file in the root directory of this source tree.
 */

#include "ZSTDCompression.h"

int ZSTDCompression::compressData(
    const void* uncompressedData,
    std::vector<uint8_t>& compressedData,
    uint32_t numUncompressedBytes) {
  uint32_t maxCompressedBytes = ZSTD_compressBound(numUncompressedBytes);

  compressedData.resize(maxCompressedBytes);

  return ZSTD_compress2(
      zstdCompressionContext_,
      compressedData.data(),
      maxCompressedBytes,
      uncompressedData,
      numUncompressedBytes);
}

int ZSTDCompression::decompressData(
    const std::vector<uint8_t>& compressedData,
    std::vector<uint8_t>& decompressedData) {
  // ZSTD decompress
  return ZSTD_decompressDCtx(zstdDecompressionContext_,
      decompressedData.data(),
      decompressedData.size(),
      compressedData.data(),
      compressedData.size());
}
