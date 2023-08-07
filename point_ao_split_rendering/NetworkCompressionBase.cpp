// (c) Meta Platforms, Inc. and its affiliates
#include "NetworkCompressionBase.h"
#include "LZ4Compression.h"
#include "ZSTDCompression.h"

std::unique_ptr<NetworkCompressionBase> NetworkCompressionBase::getDerived(
    NetworkCompressionID id) {
  switch (id) {
    case NetworkCompressionID::LZ4:
      return std::make_unique<LZ4Compression>();
    case NetworkCompressionID::ZSTD:
      return std::make_unique<ZSTDCompression>();
    default:
      throw std::runtime_error("Error in NetworkCompressionBase::getDerived(): ID unknown!");
  }
}
