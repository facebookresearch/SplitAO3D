// (c) Meta Platforms, Inc. and its affiliates
#pragma once
#include <Falcor.h>
#include "PointServerHashGenerator.h"

namespace split_rendering {
class PointHashCreateNetworkBufferStage {
 public:
  void init(PointServerHashGenerator& serverHashGen);
  void execute(
      Falcor::RenderContext* renderContext,
      PointServerHashGenerator& serverHashGen,
      MeshPointGenerator& pointGen);

  std::vector<Falcor::HashUpdateInfo> getNetworkHashUpdateInfo(
      Falcor::RenderContext* renderContext);

  Falcor::Buffer::SharedPtr& getHashIndirectBuffer() {
    return hashIndirectArgsBuffer_;
  }

  Falcor::Buffer::SharedPtr& getDirtyHashInfoBuffer() {
    return dirtyHashInfoBuffer_;
  }

 private:
  Falcor::ComputePass::SharedPtr hashUpdateComputePass_;
  Falcor::Buffer::SharedPtr hashIndirectArgsBuffer_;
  Falcor::Buffer::SharedPtr dirtyHashInfoBuffer_;
  Falcor::Buffer::SharedPtr hashUpdateBuffer_;
  uint32_t numHashUpdates_;
};
} // namespace split_rendering
