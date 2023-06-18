// (c) Meta Platforms, Inc. and its affiliates
#pragma once

#include <Falcor.h>
#include "HashFunctionShared.slang"
#include "MeshPointGenerator.h"
#include "PointData.slang"

namespace split_rendering {

// This class manages a hash table with 2 levels:
// 1: Hash table with fixed number of buckets, linear in memory. Each bucket is an offset into the
// second level. 2: Point cells with a fixed (maximum) number of points per cell, linear in memory.
//
// After initial generation, it can manage point updates from the GPU (if position/normal/value of
// points change) and prepare updates for sending to the client (insert, update, delete). We could
// also skip the preparation of client updates on the CPU and prepare them all on the GPU - could be
// relevant in the future.
class PointServerHashGenerator {
 public:
  // Generates linearized kd-tree buffers for use in shaders
  void generate(Falcor::Scene::SharedPtr& scene, const MeshPointGenerator& pointGen);

  Falcor::Buffer::SharedPtr& getGPUHashToPointCell() {
    return gpuHashToPointCell_;
  }

  Falcor::Buffer::SharedPtr& getGPUHashNumBuckets() {
    return gpuHashNumBuckets_;
  }

  Falcor::Buffer::SharedPtr& getGPUInstanceHashInfo() {
    return gpuInstanceHashInfo_;
  }

  Falcor::Buffer::SharedPtr& getGPUInstancePointInfo() {
    return gpuInstancePointInfo_;
  }

  //Falcor::Buffer::SharedPtr& getGPUPointCells() {
  //  return gpuPointCells_;
  //}

  Falcor::Buffer::SharedPtr& getGPUCompressedClientPointCells() {
    return gpuCompressedClientPointCells_;
  }

  Falcor::Buffer::SharedPtr& getGPUPreviousCompressedClientPointCells() {
    return gpuPreviousCompressedClientPointCells_;
  }

  Falcor::Buffer::SharedPtr& getGPUPointUpdateData() {
    return gpuPointUpdateData_;
  }

  Falcor::Buffer::SharedPtr& getGPUCellDirtyFlags() {
    return mGPUCellDirtyFlags;
  }

  std::vector<Falcor::InstancePointInfo>& getCPUInstancePointInfo() {
    return instancePointInfo_;
  }

  std::vector<Falcor::InstanceHashInfo>& getCPUInstanceHashInfo() {
    return instanceHashInfo_;
  }

  std::vector<Falcor::PointData>& getCPUPointCells() {
    return pointCells_;
  }

  std::vector<Falcor::CompressedClientPointData>& getCPUCompressedClientPointCells() {
    return compressedClientPointCells_;
  }

  std::vector<Falcor::HashToCellInfo>& getCPUHashToPointCell() {
    return hashToPointCell_;
  }

  std::vector<Falcor::CompactHashToCellInfo>& getCPUCompactHashToPointCell() {
    return compactHashToPointCell_;
  }

  Falcor::Buffer::SharedPtr gpuPositions_;
  Falcor::Buffer::SharedPtr gpuNormals_;
  Falcor::Buffer::SharedPtr gpuTangents_;
  Falcor::Buffer::SharedPtr gpuBarycentrics_;
  Falcor::Buffer::SharedPtr gpuInstanceTriangleIDs_;
  Falcor::Buffer::SharedPtr gpuInstanceIDs_;
  Falcor::Buffer::SharedPtr gpuValues_;

 private:
  Falcor::Buffer::SharedPtr gpuHashToPointCell_;
  Falcor::Buffer::SharedPtr gpuHashNumBuckets_;
  Falcor::Buffer::SharedPtr gpuInstanceHashInfo_;
  Falcor::Buffer::SharedPtr gpuInstancePointInfo_;
  Falcor::Buffer::SharedPtr gpuPointCells_;
  




  // float3 position;
  // float3 normal;
  // float3 tangent;
  // float2 barycentrics;
  // uint instanceTriangleId;
  // uint instanceId;
  // float value;


  Falcor::Buffer::SharedPtr gpuCompressedClientPointCells_;
  Falcor::Buffer::SharedPtr gpuPreviousCompressedClientPointCells_;
  Falcor::Buffer::SharedPtr gpuPointUpdateData_;
  Falcor::Buffer::SharedPtr mGPUCellDirtyFlags;
  std::vector<Falcor::InstanceHashInfo> instanceHashInfo_;
  std::vector<Falcor::InstancePointInfo> instancePointInfo_;
  std::vector<Falcor::PointData> pointCells_;
  std::vector<Falcor::CompressedClientPointData> compressedClientPointCells_;
  std::vector<Falcor::HashToCellInfo> hashToPointCell_;
  std::vector<Falcor::CompactHashToCellInfo> compactHashToPointCell_;

  uint32_t hashToPointCellSize_ = 0;
  uint32_t pointCellsSize_ = 0;
};

} // namespace split_rendering
