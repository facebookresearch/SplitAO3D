// (c) Meta Platforms, Inc. and its affiliates
#pragma once

#include <Falcor.h>
#include "PointData.slang"
#include "MeshPointGenerator.h"
#include "HashFunctionShared.slang"

namespace split_rendering {

class PointHashGenerator {
 public:
  void generate(
      Falcor::Scene::SharedPtr& scene,
      const MeshPointGenerator& pointGen,
      std::vector<Falcor::PointData>& pointCells,
      std::vector<Falcor::InstancePointInfo>& instancePointInfos);

  Falcor::Buffer::SharedPtr& getGPUHashToBucket() {
    return gpuHashToBucket_;
  }

  Falcor::Buffer::SharedPtr& getGPUHashBucketToPointCell() {
    return gpuHashBucketToPointCell_;
  }

  Falcor::Buffer::SharedPtr& getGPUInstanceHashInfo() {
    return gpuInstanceHashInfo_;
  }

  Falcor::Buffer::SharedPtr& getGPUPointCells() {
    return gpuPointCells_;
  }

 private:
  Falcor::Buffer::SharedPtr gpuHashToBucket_;
  Falcor::Buffer::SharedPtr gpuHashBucketToPointCell_;
  Falcor::Buffer::SharedPtr gpuInstanceHashInfo_;
  Falcor::Buffer::SharedPtr gpuPointCells_;
  std::vector<Falcor::InstanceHashInfo> instanceHashInfo_;
  std::vector<uint32_t> pointCells_;
  std::vector<int32_t> hashToBucket_;
  std::vector<Falcor::HashBucketInfo> hashBucketToPointCell_;
  std::vector<Falcor::PointsAABB> instanceAABBs_;
};

} // namespace split_rendering
