// (c) Meta Platforms, Inc. and its affiliates
#include "PointCellCreateNetworkBufferStage.h"
#include "PointData.slang"
using namespace Falcor;

namespace split_rendering {
void PointCellCreateNetworkBufferStage::init(PointServerHashGenerator& serverHashGen) {
  cellUpdateComputePass_ =
      ComputePass::create("Samples/FalcorServer/PointCellCreateNetworkBufferStage.cs.slang");
  cellUpdateComputePass_->getProgram()->setGenerateDebugInfoEnabled(true);

  // NOTE/TODO: This will break if we need to update more than 65535 cells in a frame, as
  //            the d3d12 spec mentions this limit.
  Falcor::IndirectDispatchArgs args_init{0, 1, 1};
  cellIndirectArgsBuffer_ = Falcor::Buffer::createStructured(
      sizeof(Falcor::IndirectDispatchArgs),
      1,
      Falcor::ResourceBindFlags::ShaderResource | Falcor::ResourceBindFlags::UnorderedAccess,
      Falcor::Buffer::CpuAccess::None,
      &args_init);

  dirtyCellInfoBuffer_ = Falcor::Buffer::createStructured(
      sizeof(uint32_t),
      serverHashGen.getCPUPointCells().size() / FIXED_POINTS_PER_CELL,
      Falcor::ResourceBindFlags::ShaderResource | Falcor::ResourceBindFlags::UnorderedAccess,
      Falcor::Buffer::CpuAccess::None);

  cellUpdateBuffer_ = Falcor::Buffer::createStructured(
      sizeof(Falcor::CellUpdateInfo),
      serverHashGen.getCPUPointCells().size() / FIXED_POINTS_PER_CELL,
      Falcor::ResourceBindFlags::ShaderResource | Falcor::ResourceBindFlags::UnorderedAccess,
      Falcor::Buffer::CpuAccess::None);

  cellUpdateDeltaBuffer_ = Falcor::Buffer::createStructured(
      sizeof(Falcor::CellUpdateInfo),
      serverHashGen.getCPUPointCells().size() / FIXED_POINTS_PER_CELL,
      Falcor::ResourceBindFlags::ShaderResource | Falcor::ResourceBindFlags::UnorderedAccess,
      Falcor::Buffer::CpuAccess::None);
}

void PointCellCreateNetworkBufferStage::execute(
    RenderContext* renderContext,
    PointServerHashGenerator& serverHashGen,
    MeshPointGenerator& pointGen) {
  FALCOR_PROFILE("PointCellCreateNetworkBufferStage");

  auto vars = cellUpdateComputePass_->getVars();

  //vars["serverAOPoints"] = serverHashGen.getGPUPointCells();

  vars["serverAOPositions"] = serverHashGen.gpuPositions_;
  vars["serverAONormals"] = serverHashGen.gpuNormals_;
  vars["serverAOTangents"] = serverHashGen.gpuTangents_;
  vars["serverAOBarycentrics"] = serverHashGen.gpuBarycentrics_;
  vars["serverAOInstanceTriangleIDs"] = serverHashGen.gpuInstanceTriangleIDs_;
  vars["serverAOInstanceIDs"] = serverHashGen.gpuInstanceIDs_;
  vars["serverAOValues"] = serverHashGen.gpuValues_;
  vars["compressedClientAOPoints"] = serverHashGen.getGPUCompressedClientPointCells();
  vars["previousCompressedClientAOPoints"] =
      serverHashGen.getGPUPreviousCompressedClientPointCells();
  vars["instancePointInfo"] = serverHashGen.getGPUInstancePointInfo();
  vars["instanceToDiskRadius"] = pointGen.getGPUDiskRadiusPerInstance();
  vars["serverInstanceHashInfo"] = serverHashGen.getGPUInstanceHashInfo();
  vars["serverHashToPointCell"] = serverHashGen.getGPUHashToPointCell();
  vars["hashNumBuckets"] = serverHashGen.getGPUHashNumBuckets();
  vars["cellDirtyFlags"] = serverHashGen.getGPUCellDirtyFlags();
  vars["cellDirtyInfos"] = dirtyCellInfoBuffer_;
  vars["cellUpdateInfos"] = cellUpdateBuffer_;
  vars["cellUpdateDeltaInfos"] = cellUpdateDeltaBuffer_;

  cellUpdateComputePass_->executeIndirect(renderContext, cellIndirectArgsBuffer_.get());
  Falcor::IndirectDispatchArgs* args =
      (Falcor::IndirectDispatchArgs*)cellIndirectArgsBuffer_->map(Falcor::Buffer::MapType::Read);

  numCellUpdates_ = args->x;
  cellIndirectArgsBuffer_->unmap();

  Falcor::IndirectDispatchArgs args_init{0, 1, 1};
  renderContext->updateBuffer(cellIndirectArgsBuffer_.get(), &args_init, 0, 4);
}

std::vector<Falcor::CellUpdateInfo> PointCellCreateNetworkBufferStage::getNetworkCellUpdateInfo(
    Falcor::RenderContext* renderContext) {
  if (numCellUpdates_ == 0) {
    return {};
  }

  Falcor::CellUpdateInfo* cudDelta =
      (Falcor::CellUpdateInfo*)cellUpdateDeltaBuffer_->map(Falcor::Buffer::MapType::Read);

  std::vector<Falcor::CellUpdateInfo> cellUpdates(numCellUpdates_);

  std::memcpy(cellUpdates.data(), cudDelta, numCellUpdates_ * sizeof(Falcor::CellUpdateInfo));

  cellUpdateDeltaBuffer_->unmap();
  numCellUpdates_ = 0;
  return cellUpdates;
}
} // namespace split_rendering
