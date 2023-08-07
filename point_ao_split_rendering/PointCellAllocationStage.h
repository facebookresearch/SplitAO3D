// (c) Meta Platforms, Inc. and its affiliates
#pragma once
#include <Falcor.h>
#include "PointServerHashGenerator.h"

namespace split_rendering {
class PointCellAllocationStage {
 public:
  void init();
  void execute(
      Falcor::RenderContext* renderContext,
      PointServerHashGenerator& serverHashGen,
      MeshPointGenerator& pointGen);

  Falcor::Buffer::SharedPtr& getIndirectBuffer() {
    return gpuIndirectArgsBuffer_;
  }

 private:
  Falcor::ComputePass::SharedPtr computePass_;
  Falcor::Buffer::SharedPtr gpuIndirectArgsBuffer_;
};
} // namespace split_rendering
