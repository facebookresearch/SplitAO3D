// Copyright (c) Facebook
#pragma once
#include <Falcor.h>
#include "PointServerHashGenerator.h"

namespace split_rendering {
class PointCellCreateNetworkBufferStage {
 public:
  void init(PointServerHashGenerator& serverHashGen);
  void execute(
      Falcor::RenderContext* renderContext,
      PointServerHashGenerator& serverHashGen,
      MeshPointGenerator& pointGen);

  std::vector<Falcor::CellUpdateInfo> getNetworkCellUpdateInfo(
      Falcor::RenderContext* renderContext);

  Falcor::Buffer::SharedPtr& getCellIndirectBuffer() {
    return cellIndirectArgsBuffer_;
  }

  Falcor::Buffer::SharedPtr& getDirtyCellInfoBuffer() {
    return dirtyCellInfoBuffer_;
  }

 private:
  Falcor::ComputePass::SharedPtr cellUpdateComputePass_;
  Falcor::Buffer::SharedPtr cellIndirectArgsBuffer_;
  Falcor::Buffer::SharedPtr dirtyCellInfoBuffer_;
  Falcor::Buffer::SharedPtr cellUpdateBuffer_;
  Falcor::Buffer::SharedPtr cellUpdateDeltaBuffer_;
  uint32_t numCellUpdates_;
};
} // namespace split_rendering