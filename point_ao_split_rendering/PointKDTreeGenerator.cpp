/*
 * Copyright (c) Meta Platforms, Inc. and affiliates.
 * All rights reserved.
 *
 * This source code is licensed under the license found in the
 * LICENSE file in the root directory of this source tree.
 */

#include "PointKDTreeGenerator.h"
#include <glm/gtx/matrix_decompose.hpp>
#include <algorithm>
#include <execution>
#include "../poisson_sampling/cySampleElim.h"
#include "nanoflann.hpp"

namespace split_rendering {

// These structs were mostly adapted from the nanoflann code base.
struct PointCloud {
  PointCloud(std::vector<Falcor::PointData>& p) : pts(p) {}

  std::vector<Falcor::PointData>& pts;

  // Must return the number of data points
  inline size_t kdTreeGetPointCount() const {
    return pts.size();
  }

  // Returns the dim'th component of the idx'th point in the class:
  // Since this is inlined and the "dim" argument is typically an immediate
  // value, the
  //  "if/else's" are actually solved at compile time.
  inline float kdTreeGetPt(const size_t idx, const size_t dim) const {
    if (dim == 0)
      return pts[idx].position.x;
    else if (dim == 1)
      return pts[idx].position.y;
    else
      return pts[idx].position.z;
  }

  // Optional bounding-box computation: return false to default to a standard
  // bbox computation loop.
  //   Return true if the BBOX was already computed by the class and returned
  //   in "bb" so it can be avoided to redo it again. Look at bb.size() to
  //   find out the expected dimensionality (e.g. 2 or 3 for point clouds)
  template <class BBOX>
  bool kdTreeGetBbox(BBOX& /* bb */) const {
    return false;
  }
};

struct PointCloudRange {
  PointCloudRange(const std::vector<Falcor::PointData>& p, uint32_t start, uint32_t size)
      : pts(p), start(start), size(size) {}

  const std::vector<Falcor::PointData>& pts;
  uint32_t start;
  uint32_t size;

  // Must return the number of data points
  inline size_t kdTreeGetPointCount() const {
    return size;
  }

  // Returns the dim'th component of the idx'th point in the class:
  // Since this is inlined and the "dim" argument is typically an immediate
  // value, the
  //  "if/else's" are actually solved at compile time.
  inline float kdTreeGetPt(const size_t idx, const size_t dim) const {
    if (dim == 0)
      return pts[idx + start].position.x;
    else if (dim == 1)
      return pts[idx + start].position.y;
    else
      return pts[idx + start].position.z;
  }

  // Optional bounding-box computation: return false to default to a standard
  // bbox computation loop.
  //   Return true if the BBOX was already computed by the class and returned
  //   in "bb" so it can be avoided to redo it again. Look at bb.size() to
  //   find out the expected dimensionality (e.g. 2 or 3 for point clouds)
  template <class BBOX>
  bool kdTreeGetBbox(BBOX& /* bb */) const {
    return false;
  }
};

// This is used when building a kd-tree from point cells of fixed sizes
// These point cells can have invalid/unused points in them which we need to skip
// The kdTreeIndexMap needs to be mapped to the GPU later as well to get the correct lookup.
struct SparsePointCloudRange {
  SparsePointCloudRange(
      const std::vector<Falcor::PointData>& p,
      uint32_t start,
      uint32_t sparseSize)
      : pts(p), start(start), sparseSize(sparseSize) {
    for (uint32_t i = start; i < start + sparseSize; i++) {
      if (pts[i].value >= 0) {
        kdTreeIndexMap.push_back(i);
      }
    }
  }

  const std::vector<Falcor::PointData>& pts;
  std::vector<uint32_t> kdTreeIndexMap;
  uint32_t start;
  uint32_t sparseSize;

  // Must return the number of data points
  inline size_t kdTreeGetPointCount() const {
    return kdTreeIndexMap.size();
  }

  // Returns the dim'th component of the idx'th point in the class:
  // Since this is inlined and the "dim" argument is typically an immediate
  // value, the
  //  "if/else's" are actually solved at compile time.
  inline float kdTreeGetPt(const size_t idx, const size_t dim) const {
    if (dim == 0)
      return pts[kdTreeIndexMap[idx]].position.x;
    else if (dim == 1)
      return pts[kdTreeIndexMap[idx]].position.y;
    else
      return pts[kdTreeIndexMap[idx]].position.z;
  }

  // Optional bounding-box computation: return false to default to a standard
  // bbox computation loop.
  //   Return true if the BBOX was already computed by the class and returned
  //   in "bb" so it can be avoided to redo it again. Look at bb.size() to
  //   find out the expected dimensionality (e.g. 2 or 3 for point clouds)
  template <class BBOX>
  bool kdTreeGetBbox(BBOX& /* bb */) const {
    return false;
  }
};

void PointKDTreeGenerator::generate(
    Falcor::Scene::SharedPtr& scene,
    const MeshPointGenerator& pointGen,
    std::vector<Falcor::PointData>& pointCells,
    std::vector<Falcor::InstancePointInfo>& instancePointInfos) {
  const auto& numFinalSamplesPerInstance = pointGen.getNumSamplesPerInstance();
  const auto& sampleOffsetPerInstance = pointGen.getSampleOffsetPerInstance();
  const auto& cpuPointsData = pointCells;

  std::vector<Falcor::KDTreeGPUNode> linearizedKdTrees;
  std::vector<uint32_t> linearizedKdTreeIndices;
  std::vector<uint32_t> instanceIdToKdTree;
  std::vector<uint32_t> instanceIdToKdTreeIndexOffset;

  instanceIdToKdTree.push_back(0);

  for (uint32_t instanceId = 0; instanceId < scene->getGeometryInstanceCount(); instanceId++) {
    const auto& ipi = instancePointInfos[instanceId];

    SparsePointCloudRange pc = SparsePointCloudRange(
        cpuPointsData, ipi.pointCellOffset, ipi.numAllocatedCells * FIXED_POINTS_PER_CELL);

    // construct a kd-tree index:
    using my_kd_tree_t = nanoflann::KDTreeSingleIndexAdaptor<
        nanoflann::L2_Simple_Adaptor<float, SparsePointCloudRange>,
        SparsePointCloudRange,
        3>;

    my_kd_tree_t index(3 /*dim*/, pc, {kKdTreeMaxPointsLeaf /* max leaf */});

    auto kdTree = index.getLinearizedTree();
    auto kdTreeIndex = index.vAcc;

    for (auto& idx : kdTreeIndex) {
      idx = pc.kdTreeIndexMap[idx];
    }

    linearizedKdTrees.insert(
        std::end(linearizedKdTrees), std::begin(kdTree), std::end(kdTree));

    instanceIdToKdTreeIndexOffset.push_back(linearizedKdTreeIndices.size());

    linearizedKdTreeIndices.insert(
        std::end(linearizedKdTreeIndices), std::begin(kdTreeIndex), std::end(kdTreeIndex));

    if (instanceId < scene->getGeometryInstanceCount() - 1) {
      instanceIdToKdTree.push_back(instanceIdToKdTree.back() + kdTree.size());
    }
  }

  gpuKdTree_ = Falcor::Buffer::createStructured(
      sizeof(Falcor::KDTreeGPUNode),
      linearizedKdTrees.size(),
      Falcor::ResourceBindFlags::ShaderResource | Falcor::ResourceBindFlags::UnorderedAccess,
      Falcor::Buffer::CpuAccess::None,
      linearizedKdTrees.data());

  gpuKdTreeIndex_ = Falcor::Buffer::createStructured(
      sizeof(uint32_t),
      linearizedKdTreeIndices.size(),
      Falcor::ResourceBindFlags::ShaderResource | Falcor::ResourceBindFlags::UnorderedAccess,
      Falcor::Buffer::CpuAccess::None,
      linearizedKdTreeIndices.data());

  gpuInstanceKdTreeOffset_ = Falcor::Buffer::createStructured(
      sizeof(uint32_t),
      instanceIdToKdTree.size(),
      Falcor::ResourceBindFlags::ShaderResource | Falcor::ResourceBindFlags::UnorderedAccess,
      Falcor::Buffer::CpuAccess::None,
      instanceIdToKdTree.data());

  gpuInstanceKdTreeIndexOffset_ = Falcor::Buffer::createStructured(
      sizeof(uint32_t),
      instanceIdToKdTreeIndexOffset.size(),
      Falcor::ResourceBindFlags::ShaderResource | Falcor::ResourceBindFlags::UnorderedAccess,
      Falcor::Buffer::CpuAccess::None,
      instanceIdToKdTreeIndexOffset.data());
}

} // namespace split_rendering
