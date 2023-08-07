// (c) Meta Platforms, Inc. and its affiliates
#pragma once

#include <Falcor.h>
#include "PointData.slang"
#include "MeshPointGenerator.h"

namespace split_rendering {

class PointKDTreeGenerator {
 public:
  // Generates linearized kd-tree buffers for use in shaders
  void generate(
      Falcor::Scene::SharedPtr& scene,
      const MeshPointGenerator& pointGen,
      std::vector<Falcor::PointData>& pointCells,
      std::vector<Falcor::InstancePointInfo>& instancePointInfos);

  Falcor::Buffer::SharedPtr& getGPUKDTree() {
    return gpuKdTree_;
  }

  Falcor::Buffer::SharedPtr& getGPUKDTreeIndex() {
    return gpuKdTreeIndex_;
  }

  Falcor::Buffer::SharedPtr& getGPUInstanceKDTreeOffset() {
    return gpuInstanceKdTreeOffset_;
  }

  Falcor::Buffer::SharedPtr& getGPUInstanceKDTreeIndexOffset() {
    return gpuInstanceKdTreeIndexOffset_;
  }

 private:
  static constexpr Falcor::uint kKdTreeMaxPointsLeaf = 16;
  Falcor::Buffer::SharedPtr gpuKdTree_;
  Falcor::Buffer::SharedPtr gpuKdTreeIndex_;
  Falcor::Buffer::SharedPtr gpuInstanceKdTreeIndexOffset_;
  Falcor::Buffer::SharedPtr gpuInstanceKdTreeOffset_;
};

} // namespace split_rendering
