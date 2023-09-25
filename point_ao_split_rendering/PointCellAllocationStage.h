/*
 * Copyright (c) Meta Platforms, Inc. and affiliates.
 * All rights reserved.
 *
 * This source code is licensed under the license found in the
 * LICENSE file in the root directory of this source tree.
 */

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
