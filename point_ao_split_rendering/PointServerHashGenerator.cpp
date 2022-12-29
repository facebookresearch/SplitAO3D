#include "PointServerHashGenerator.h"
#include <algorithm>
#include <execution>

namespace split_rendering {

int compressHashToCellInfoIndex(int pointCellIndex, uint32_t numPoints, uint32_t numCells) {
  return pointCellIndex;
}

void PointServerHashGenerator::generate(
    Falcor::Scene::SharedPtr& scene,
    const MeshPointGenerator& pointGen) {
  const auto& numFinalSamplesPerInstance = pointGen.getNumSamplesPerInstance();
  const auto& sampleOffsetPerInstance = pointGen.getSampleOffsetPerInstance();
  const auto& diskRadiusPerInstance = pointGen.getDiskRadiusPerInstance();
  const auto& cpuPointsData = pointGen.getCPUPointData();

  std::vector<Falcor::HashNumBuckets> hashNumBuckets;

  // Hash Table Setup
  for (uint32_t instanceId = 0; instanceId < scene->getGeometryInstanceCount(); instanceId++) {
    // NOTE: sometimes the hash table size can be a bit too small, and ideally we want to configure
    // it and/or figure out dynamically how much we need
    uint32_t hashTableSize = std::exp2(
        std::ceil(std::log2((numFinalSamplesPerInstance[instanceId] / FIXED_HASH_BUCKET_SIZE))) +
        HASH_LOG2_SIZE_FACTOR);

    // Conservatively use 4x the average number of cells to preallocate memory for cells
    uint32_t numCells = (numFinalSamplesPerInstance[instanceId] / DISK_RADIUS_FACTOR) *
        NUM_CELLS_PREALLOCATION_FACTOR;

    if (scene->getGeometryInstance(instanceId).hasDynamicData()) {
      // More memory for dynamic instances - this will make things easier when updating (= adding
      // more cells) later
      numCells *= 4;
    }

    instanceHashInfo_.push_back({});
    instancePointInfo_.push_back({});
    auto& ihi = instanceHashInfo_.back();
    auto& ipi = instancePointInfo_.back();

    ihi.hashToBucketOffset = hashToPointCellSize_;
    ipi.pointCellOffset = pointCellsSize_;
    ihi.hashToBucketSize = hashTableSize;
    ipi.maxNumPoints = numCells * FIXED_POINTS_PER_CELL;

    hashNumBuckets.resize(hashToPointCellSize_ + hashTableSize);

    hashToPointCell_.resize(hashToPointCellSize_ + hashTableSize * FIXED_HASH_BUCKET_SIZE);
    hashToPointCellSize_ = hashToPointCell_.size();
    pointCells_.resize(pointCellsSize_ + numCells * FIXED_POINTS_PER_CELL);

    compressedClientPointCells_.resize(pointCellsSize_ + numCells * FIXED_POINTS_PER_CELL);
    pointCellsSize_ = pointCells_.size();

    uint32_t& numAllocatedCells = ipi.numAllocatedCells;
    numAllocatedCells = 0;

    Falcor::float3& aabbMin = ipi.aabbMin;
    Falcor::float3& aabbMax = ipi.aabbMax;
    aabbMin = glm::float3(
        std::numeric_limits<float>::max(),
        std::numeric_limits<float>::max(),
        std::numeric_limits<float>::max());
    aabbMax = glm::float3(
        -std::numeric_limits<float>::max(),
        -std::numeric_limits<float>::max(),
        -std::numeric_limits<float>::max());

    for (uint32_t pointId = sampleOffsetPerInstance[instanceId];
         pointId < sampleOffsetPerInstance[instanceId] + numFinalSamplesPerInstance[instanceId];
         pointId++) {
      const auto& point = cpuPointsData[pointId];

      aabbMin = glm::min(aabbMin, point.position);
      aabbMax = glm::max(aabbMax, point.position);
    }

    glm::float3 aabbSizeInitial = aabbMax - aabbMin;

    aabbMax += 0.5f * aabbSizeInitial;

    aabbMin -= 0.5f * aabbSizeInitial;

    ihi.aabbMin = aabbMin;
    ihi.aabbMax = aabbMax;

    float diskRadius = DISK_RADIUS_FACTOR * diskRadiusPerInstance[instanceId];
    glm::float3 aabbSize = aabbMax - aabbMin;

    glm::uvec3 gridDim = {
        ((aabbSize.x / diskRadius)),
        ((aabbSize.y / diskRadius)),
        ((aabbSize.z / diskRadius))};

    gridDim.x = glm::max(glm::max(gridDim.x, gridDim.y), gridDim.z);
    gridDim.y = gridDim.x;
    gridDim.z = gridDim.x;
    
    ipi.gridDim = gridDim;

    ihi.gridDim = gridDim;

    for (uint32_t pointId = sampleOffsetPerInstance[instanceId];
         pointId < sampleOffsetPerInstance[instanceId] + numFinalSamplesPerInstance[instanceId];
         pointId++) {
      const auto& cpuPoint = cpuPointsData[pointId];
      const auto& pointPos = cpuPoint.position;

      // Get grid index for point given grid size == poisson disk radius
      glm::ivec3 coords = ((pointPos - aabbMin)) / diskRadius;

      Falcor::HashData hd = Falcor::getHash(coords, gridDim, hashTableSize, 0);

      // We store the buckets linearly in memory so we need the extra multiplication here
      hd.hashBase *= FIXED_HASH_BUCKET_SIZE;

      bool found = false;

      // Check if we already have an existing entry that matches the rawCellId, otherwise
      // keep track of first non-negative index

      int firstFreeIndex = -1;
      bool foundCell = false;

      for (int hashInfoIndex = 0; hashInfoIndex < FIXED_HASH_BUCKET_SIZE; hashInfoIndex++) {
        auto& hashInfo = hashToPointCell_[hd.hashBase + ihi.hashToBucketOffset + hashInfoIndex];

        // if a matching cell already exists
        if (hashInfo.pointCellIndex >= 0 && hashInfo.rawCellId == hd.rawCellId) {
          // Find first free entry in the point cell
          foundCell = true;
          for (uint32_t localPointCellOffset = 0; localPointCellOffset < FIXED_POINTS_PER_CELL;
               localPointCellOffset++) {
            auto& pointCellPoint =
                pointCells_[hashInfo.pointCellIndex + ipi.pointCellOffset + localPointCellOffset];

            if (pointCellPoint.value < 0) {
              pointCellPoint = cpuPoint;
              pointCellPoint.value = UNINITIALIZED_VALUE;
              hashInfo.numPoints++;
              compressedClientPointCells_
                  [hashInfo.pointCellIndex + ipi.pointCellOffset + localPointCellOffset] =
                      compressClientData(pointCellPoint, diskRadius, ipi.aabbMin);

              break;
            }

            // if we don't find anything, we just skip/ignore.
          }

          // break out as we already found a cell
          break;

        } else if (hashInfo.pointCellIndex < 0 && firstFreeIndex < 0) {
          firstFreeIndex = hashInfoIndex;
        }
      }

      // If we successfully found a cell for the given point, go to next.
      // Note that this even happens if the cell is full, which we ignore.
      if (foundCell)
        continue;

      // If the hash bucket for this given hash is full, we also skip/ignore
      if (firstFreeIndex < 0)
        continue;

      // We reach this part of the code if we need to allocate a new point cell within this hash
      // info and found an empty hash info in the array
      auto& hashInfo = hashToPointCell_[hd.hashBase + ihi.hashToBucketOffset + firstFreeIndex];
      hashInfo.rawCellId = hd.rawCellId;
      hashInfo.numPoints = 1;
      hashInfo.pointCellIndex = (numAllocatedCells * FIXED_POINTS_PER_CELL);
      numAllocatedCells++;

      if (hashInfo.pointCellIndex >= (numCells * FIXED_POINTS_PER_CELL)) {
        // If this happens, we run out of preallocated memory. This ideally should not happen.
        continue;
      }
      hashNumBuckets
          [hd.hashBase / FIXED_HASH_BUCKET_SIZE + ihi.hashToBucketOffset / FIXED_HASH_BUCKET_SIZE]
              .numBuckets++;
      auto& pointCell = pointCells_[hashInfo.pointCellIndex + ipi.pointCellOffset];
      // As this is a new cell, we simply add the point as the first entry and set it to valid

      pointCell = cpuPoint;
      pointCell.value = UNINITIALIZED_VALUE;
      compressedClientPointCells_
          [hashInfo.pointCellIndex + ipi.pointCellOffset] =
          compressClientData(pointCell, diskRadius, ipi.aabbMin);
    }
  }

  // Compress hash table entries

  for (const auto& hashEntry : hashToPointCell_) {
    Falcor::CompactHashToCellInfo chtci;

    chtci.rawCellId = hashEntry.rawCellId;

    chtci.encodedIndex = compressHashToCellInfoIndex(
        hashEntry.pointCellIndex, hashEntry.numPoints, hashEntry.numCells);

    if (hashEntry.pointCellIndex < 0) {
      chtci.encodedIndex = INVALID_CELL;
      chtci.rawCellId = INVALID_CELL;
    }

    compactHashToPointCell_.push_back(chtci);
  }

  // After all instances have been processed, we can generate the GPU buffers
  gpuHashToPointCell_ = Falcor::Buffer::createStructured(
      sizeof(Falcor::CompactHashToCellInfo),
      compactHashToPointCell_.size(),
      Falcor::ResourceBindFlags::ShaderResource | Falcor::ResourceBindFlags::UnorderedAccess,
      Falcor::Buffer::CpuAccess::None,
      compactHashToPointCell_.data());

  gpuHashNumBuckets_ = Falcor::Buffer::createStructured(
      sizeof(Falcor::HashNumBuckets),
      hashNumBuckets.size(),
      Falcor::ResourceBindFlags::ShaderResource | Falcor::ResourceBindFlags::UnorderedAccess,
      Falcor::Buffer::CpuAccess::None,
      hashNumBuckets.data());

  gpuPointCells_ = Falcor::Buffer::createStructured(
      sizeof(Falcor::PointData),
      pointCells_.size(),
      Falcor::ResourceBindFlags::ShaderResource | Falcor::ResourceBindFlags::UnorderedAccess,
      Falcor::Buffer::CpuAccess::None,
      pointCells_.data());


  mGPUCellDirtyFlags = Falcor::Buffer::createStructured(
      sizeof(uint32_t),
      pointCells_.size() / FIXED_POINTS_PER_CELL,
      Falcor::ResourceBindFlags::ShaderResource | Falcor::ResourceBindFlags::UnorderedAccess,
      Falcor::Buffer::CpuAccess::None);

  gpuCompressedClientPointCells_ = Falcor::Buffer::createStructured(
      sizeof(Falcor::CompressedClientPointData),
      compressedClientPointCells_.size(),
      Falcor::ResourceBindFlags::ShaderResource | Falcor::ResourceBindFlags::UnorderedAccess,
      Falcor::Buffer::CpuAccess::None,
      compressedClientPointCells_.data());

  gpuPreviousCompressedClientPointCells_ = Falcor::Buffer::createStructured(
      sizeof(Falcor::CompressedClientPointData),
      compressedClientPointCells_.size(),
      Falcor::ResourceBindFlags::ShaderResource | Falcor::ResourceBindFlags::UnorderedAccess,
      Falcor::Buffer::CpuAccess::None,
      compressedClientPointCells_.data());

  gpuInstanceHashInfo_ = Falcor::Buffer::createStructured(
      sizeof(Falcor::InstanceHashInfo),
      instanceHashInfo_.size(),
      Falcor::ResourceBindFlags::ShaderResource | Falcor::ResourceBindFlags::UnorderedAccess,
      Falcor::Buffer::CpuAccess::None,
      instanceHashInfo_.data());

  gpuInstancePointInfo_ = Falcor::Buffer::createStructured(
      sizeof(Falcor::InstancePointInfo),
      instancePointInfo_.size(),
      Falcor::ResourceBindFlags::ShaderResource | Falcor::ResourceBindFlags::UnorderedAccess,
      Falcor::Buffer::CpuAccess::None,
      instancePointInfo_.data());

  // Generate point update data and staging buffer
  const uint32_t kNumMaxUpdates = pointCells_.size();
  gpuPointUpdateData_ = Falcor::Buffer::createStructured(
      sizeof(Falcor::PointUpdateData),
      kNumMaxUpdates,
      Falcor::ResourceBindFlags::ShaderResource | Falcor::ResourceBindFlags::UnorderedAccess,
      Falcor::Buffer::CpuAccess::None);
}

} // namespace split_rendering