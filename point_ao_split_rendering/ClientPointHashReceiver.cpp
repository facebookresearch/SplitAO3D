/*
 * Copyright (c) Meta Platforms, Inc. and affiliates.
 * All rights reserved.
 *
 * This source code is licensed under the license found in the
 * LICENSE file in the root directory of this source tree.
 */

#include "ClientPointHashReceiver.h"
#include <algorithm>
#include <execution>
#include "lz4.h"
#include "zstd.h"

namespace split_rendering {

void ClientPointHashReceiver::init() {
  cellComputePass_ =
      Falcor::ComputePass::create("Samples/FalcorServer/PointCellUpdateStage.cs.slang");
  cellComputePass_->getProgram()->setGenerateDebugInfoEnabled(true);

  hashComputePass_ =
      Falcor::ComputePass::create("Samples/FalcorServer/PointHashUpdateStage.cs.slang");
  hashComputePass_->getProgram()->setGenerateDebugInfoEnabled(true);

  // TODO: use some sort of config file for this
  constexpr uint32_t kMaxNumCellUpdates = 1000000;
  cellUpdateBuffer_ = Falcor::Buffer::createStructured(
      sizeof(Falcor::CellUpdateInfo),
      kMaxNumCellUpdates,
      Falcor::ResourceBindFlags::ShaderResource | Falcor::ResourceBindFlags::UnorderedAccess,
      Falcor::Buffer::CpuAccess::None);

  hashUpdateBuffer_ = Falcor::Buffer::createStructured(
      sizeof(Falcor::HashUpdateInfo),
      kMaxNumCellUpdates,
      Falcor::ResourceBindFlags::ShaderResource | Falcor::ResourceBindFlags::UnorderedAccess,
      Falcor::Buffer::CpuAccess::None);
}

void ClientPointHashReceiver::receive(TCPMessage& message, Falcor::RenderContext* renderContext) {
  FALCOR_PROFILE("receive");

  const auto receive_vector_message = [](TCPMessage& message, auto& gpuBuffer, uint32_t typeSize) {
    gpuBuffer = Falcor::Buffer::createStructured(
        typeSize,
        message.data.size() / typeSize,
        Falcor::ResourceBindFlags::ShaderResource | Falcor::ResourceBindFlags::UnorderedAccess,
        Falcor::Buffer::CpuAccess::None,
        message.data.data());
  };

  switch (message.header.type) {
    case TCPMessageType::PAOInstancePointInfo:
      receive_vector_message(message, gpuInstancePointInfo_, sizeof(Falcor::InstancePointInfo));
      break;
    case TCPMessageType::PAOServerInstanceHashInfo:
      receive_vector_message(message, gpuInstanceHashInfo_, sizeof(Falcor::InstanceHashInfo));
      break;
    case TCPMessageType::PAOInstanceToPoissonRadius:
      receive_vector_message(message, gpuPoissonDiskRadius_, sizeof(float));
      break;
    case TCPMessageType::PAOServerHashToPointCell:
      receive_vector_message(message, gpuHashToPointCell_, sizeof(Falcor::CompactHashToCellInfo));
      break;
    case TCPMessageType::PAOCompressedClientAOPoints:
      receive_vector_message(
          message, gpuCompressedClientPointCells_, sizeof(Falcor::CompressedClientPointData));
      break;
    case TCPMessageType::PAOEndOfInit:
      initDone_ = true;
      break;
      // no default, we skip if the message doesn't match any of these
  }

  // This is just to make the profiling in Falcor work more nicely.
  updateCells(message, renderContext);
  updateHash(message, renderContext);
}

void ClientPointHashReceiver::updateCells(
    TCPMessage& message,
    Falcor::RenderContext* renderContext) {
  FALCOR_PROFILE("updateCells");

  if (message.header.type != TCPMessageType::PAOPointCellUpdate)
    return;

  uint32_t numUpdates = message.header.size / sizeof(Falcor::CellUpdateInfo);

  if (message.header.decompressedSize != message.header.size) {
    decompressedData_.resize(message.header.decompressedSize);

    // If we don't know the compression type yet, figure it out
    if (!networkCompression_.get()) {
      networkCompression_ =
          NetworkCompressionBase::getDerived((NetworkCompressionID)message.header.width);
    }

    int numDecompressedBytes = networkCompression_->decompressData(message.data, decompressedData_);

    if (numDecompressedBytes <= 0) {
      // This should never happen.
      throw std::runtime_error("Number of decompressed bytes <= 0, error in decompression!");
    }

    cellUpdateBuffer_->setBlob(decompressedData_.data(), 0, numDecompressedBytes);

    // We need a different number of updates for when we have compression
    numUpdates = decompressedData_.size() / sizeof(Falcor::CellUpdateInfo);
  } else {
    cellUpdateBuffer_->setBlob(message.data.data(), 0, message.data.size());
  }

  auto vars = cellComputePass_->getVars();

  vars["compressedClientAOPoints"] = gpuCompressedClientPointCells_;
  vars["cellUpdateInfos"] = cellUpdateBuffer_;

  auto cb = vars["perFrameConstantBuffer"];
  cb["numUpdates"] = numUpdates;

  cellComputePass_->execute(renderContext, Falcor::uint3(numUpdates, 1, 1));
}

void ClientPointHashReceiver::updateHash(
    TCPMessage& message,
    Falcor::RenderContext* renderContext) {
  FALCOR_PROFILE("updateHash");

  if (message.header.type != TCPMessageType::PAOHashUpdate)
    return;

  hashUpdateBuffer_->setBlob(message.data.data(), 0, message.header.size);

  auto vars = hashComputePass_->getVars();

  vars["serverHashToPointCell"] = gpuHashToPointCell_;
  vars["hashUpdateInfos"] = hashUpdateBuffer_;

  uint32_t numUpdates = message.header.size / sizeof(Falcor::HashUpdateInfo);

  auto cb = vars["perFrameConstantBuffer"];
  cb["numUpdates"] = numUpdates;

  hashComputePass_->execute(renderContext, Falcor::uint3(numUpdates, 1, 1));
}

} // namespace split_rendering
