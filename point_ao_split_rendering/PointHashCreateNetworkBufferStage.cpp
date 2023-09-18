/*
 * Copyright (c) Meta Platforms, Inc. and affiliates.
 * All rights reserved.
 *
 * This source code is licensed under the license found in the
 * LICENSE file in the root directory of this source tree.
 */

#include "PointHashCreateNetworkBufferStage.h"
#include "PointData.slang"
using namespace Falcor;

namespace split_rendering {
void PointHashCreateNetworkBufferStage::init(PointServerHashGenerator& serverHashGen) {
  hashUpdateComputePass_ =
      ComputePass::create("Samples/FalcorServer/PointHashCreateNetworkBufferStage.cs.slang");
  hashUpdateComputePass_->getProgram()->setGenerateDebugInfoEnabled(true);

  // NOTE/TODO: This will break if we need to update more than 65535 cells in a frame, as
  //            the d3d12 spec mentions this limit.
  Falcor::IndirectDispatchArgs args_init{0, 1, 1};

  hashIndirectArgsBuffer_ = Falcor::Buffer::createStructured(
      sizeof(Falcor::IndirectDispatchArgs),
      1,
      Falcor::ResourceBindFlags::ShaderResource | Falcor::ResourceBindFlags::UnorderedAccess,
      Falcor::Buffer::CpuAccess::None,
      &args_init);

  hashUpdateBuffer_ = Falcor::Buffer::createStructured(
      sizeof(Falcor::HashUpdateInfo),
      serverHashGen.getCPUHashToPointCell().size() / FIXED_HASH_BUCKET_SIZE,
      Falcor::ResourceBindFlags::ShaderResource | Falcor::ResourceBindFlags::UnorderedAccess,
      Falcor::Buffer::CpuAccess::None);

  dirtyHashInfoBuffer_ = Falcor::Buffer::createStructured(
      sizeof(uint32_t),
      serverHashGen.getCPUHashToPointCell().size() / FIXED_POINTS_PER_CELL,
      Falcor::ResourceBindFlags::ShaderResource | Falcor::ResourceBindFlags::UnorderedAccess,
      Falcor::Buffer::CpuAccess::None);
}

void PointHashCreateNetworkBufferStage::execute(
    RenderContext* renderContext,
    PointServerHashGenerator& serverHashGen,
    MeshPointGenerator& pointGen) {
  FALCOR_PROFILE("PointHashCreateNetworkBufferStage");

  auto vars = hashUpdateComputePass_->getVars();

  //vars["serverAOPoints"] = serverHashGen.getGPUPointCells();
  vars["serverAOPositions"] = serverHashGen.gpuPositions_;
  vars["serverAONormals"] = serverHashGen.gpuNormals_;
  vars["serverAOTangents"] = serverHashGen.gpuTangents_;
  vars["serverAOBarycentrics"] = serverHashGen.gpuBarycentrics_;
  vars["serverAOInstanceTriangleIDs"] = serverHashGen.gpuInstanceTriangleIDs_;
  vars["serverAOInstanceIDs"] = serverHashGen.gpuInstanceIDs_;
  vars["serverAOValues"] = serverHashGen.gpuValues_;

  vars["compressedClientAOPoints"] = serverHashGen.getGPUCompressedClientPointCells();
  vars["instancePointInfo"] = serverHashGen.getGPUInstancePointInfo();
  vars["instanceToDiskRadius"] = pointGen.getGPUDiskRadiusPerInstance();
  vars["serverInstanceHashInfo"] = serverHashGen.getGPUInstanceHashInfo();
  vars["serverHashToPointCell"] = serverHashGen.getGPUHashToPointCell();
  vars["hashNumBuckets"] = serverHashGen.getGPUHashNumBuckets();
  vars["cellDirtyFlags"] = serverHashGen.getGPUCellDirtyFlags();
  vars["hashDirtyInfos"] = dirtyHashInfoBuffer_;
  vars["hashUpdateInfos"] = hashUpdateBuffer_;

  hashUpdateComputePass_->executeIndirect(renderContext, hashIndirectArgsBuffer_.get());
  Falcor::IndirectDispatchArgs* args =
      (Falcor::IndirectDispatchArgs*)hashIndirectArgsBuffer_->map(Falcor::Buffer::MapType::Read);

  numHashUpdates_ = args->x;
  hashIndirectArgsBuffer_->unmap();

  Falcor::IndirectDispatchArgs args_init{0, 1, 1};
  renderContext->updateBuffer(hashIndirectArgsBuffer_.get(), &args_init, 0, 4);
}

std::vector<Falcor::HashUpdateInfo> PointHashCreateNetworkBufferStage::getNetworkHashUpdateInfo(
    Falcor::RenderContext* renderContext) {
  if (numHashUpdates_ == 0) {
    return {};
  }

  Falcor::HashUpdateInfo* hud =
      (Falcor::HashUpdateInfo*)hashUpdateBuffer_->map(Falcor::Buffer::MapType::Read);

  std::vector<Falcor::HashUpdateInfo> hashUpdates_(numHashUpdates_);

  std::memcpy(hashUpdates_.data(), hud, numHashUpdates_ * sizeof(Falcor::HashUpdateInfo));

  hashUpdateBuffer_->unmap();

  numHashUpdates_ = 0;
  return hashUpdates_;
}
} // namespace split_rendering
