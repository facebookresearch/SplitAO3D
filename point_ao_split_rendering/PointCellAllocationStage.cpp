// (c) Meta Platforms, Inc. and its affiliates
#include "PointCellAllocationStage.h"
#include "PointData.slang"
using namespace Falcor;

namespace split_rendering {
void PointCellAllocationStage::init() {
  computePass_ = ComputePass::create("Samples/FalcorServer/PointCellAllocationStage.cs.slang");
  computePass_->getProgram()->setGenerateDebugInfoEnabled(true);

  // NOTE/TODO: This will break if we need to update more than 65535 cells in a frame, as
  //            the d3d12 spec mentions this limit.
  const Falcor::IndirectDispatchArgs kArgsInit{0, 1, 1};
  gpuIndirectArgsBuffer_ = Falcor::Buffer::createStructured(
      sizeof(Falcor::IndirectDispatchArgs),
      1,
      Falcor::ResourceBindFlags::ShaderResource | Falcor::ResourceBindFlags::UnorderedAccess,
      Falcor::Buffer::CpuAccess::None,
      &kArgsInit);
}

void PointCellAllocationStage::execute(
    RenderContext* renderContext,
    PointServerHashGenerator& serverHashGen,
    MeshPointGenerator& pointGen) {
  FALCOR_PROFILE("PointAllocationStage");
  auto vars = computePass_->getVars();

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
  vars["pointUpdateData"] = serverHashGen.getGPUPointUpdateData();
  vars["serverHashToPointCell"] = serverHashGen.getGPUHashToPointCell();
  vars["hashNumBuckets"] = serverHashGen.getGPUHashNumBuckets();

  computePass_->executeIndirect(renderContext, gpuIndirectArgsBuffer_.get());

  IndirectDispatchArgs args = {0, 1, 1};
  renderContext->updateBuffer(gpuIndirectArgsBuffer_.get(), &args);
}
} // namespace split_rendering
