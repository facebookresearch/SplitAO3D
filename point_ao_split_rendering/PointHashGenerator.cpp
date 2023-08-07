// (c) Meta Platforms, Inc. and its affiliates
#include "PointHashGenerator.h"
#include <algorithm>
#include <execution>

namespace split_rendering {

void PointHashGenerator::generate(
    Falcor::Scene::SharedPtr& scene,
    const MeshPointGenerator& pointGen,
    std::vector<Falcor::PointData>& pointCells,
    std::vector<Falcor::InstancePointInfo>& instancePointInfos) {
  const auto& numFinalSamplesPerInstance = pointGen.getNumSamplesPerInstance();
  const auto& diskRadiusPerInstance = pointGen.getDiskRadiusPerInstance();
  const auto& cpuPointsData = pointCells;

  // Hash Table Setup
  for (uint32_t instanceId = 0; instanceId < scene->getGeometryInstanceCount(); instanceId++) {
    // NOTE: sometimes the hash table size can be a bit too small, and ideally we want to configure
    // it and/or figure out dynamically how much we need
    uint32_t hashTableSize = (uint32_t)std::exp2(
        std::ceil(std::log2((numFinalSamplesPerInstance[instanceId] / FIXED_HASH_BUCKET_SIZE))) +
        HASH_LOG2_SIZE_FACTOR);

    
    hashTableSize = std::max(hashTableSize, (uint32_t)FIXED_HASH_BUCKET_SIZE);

    const auto& ipi = instancePointInfos[instanceId];

    std::vector<int32_t> instanceHashTable(hashTableSize, -1);

    std::vector<std::vector<Falcor::HashBucketInfo>> instanceHashTableBuckets;
    uint32_t numAllocatedBuckets = 0;
    uint32_t numAllocatedCells = 0;

    struct CPUPointCell {
      int rawCellId;
      std::vector<int> pointIndices;
    };

    std::vector<CPUPointCell> instancePointCells;

    instanceAABBs_.push_back({});
    auto& instanceAABB = instanceAABBs_.back();
    instanceAABB.min = glm::float3(
        std::numeric_limits<float>::max(),
        std::numeric_limits<float>::max(),
        std::numeric_limits<float>::max());
    instanceAABB.max = glm::float3(
        -std::numeric_limits<float>::max(),
        -std::numeric_limits<float>::max(),
        -std::numeric_limits<float>::max());

    for (uint32_t pointId = ipi.pointCellOffset;
         pointId < ipi.pointCellOffset + ipi.numAllocatedCells * FIXED_POINTS_PER_CELL;
         pointId++) {
      const auto& point = cpuPointsData[pointId];

      if (point.value < 0)
        continue;

      instanceAABB.min = glm::min(instanceAABB.min, point.position);
      instanceAABB.max = glm::max(instanceAABB.max, point.position);
    }

    float diskRadius = DISK_RADIUS_FACTOR * diskRadiusPerInstance[instanceId];
    glm::float3 aabbSize = instanceAABB.max - instanceAABB.min;

    glm::uvec3 gridDim = ipi.gridDim;

    for (uint32_t pointId = ipi.pointCellOffset;
         pointId < ipi.pointCellOffset + ipi.numAllocatedCells * FIXED_POINTS_PER_CELL;
         pointId++) {
      if (cpuPointsData[pointId].value < 0)
        continue;

      const auto& point = cpuPointsData[pointId].position;

      // Get grid index for point given grid size == poisson disk radius

      glm::ivec3 coords = ((point - instanceAABB.min)) / diskRadius;

      Falcor::HashData hd = Falcor::getHash(coords, gridDim, hashTableSize, 0);

      bool found = false;

      // Check if we already have a point cell matching this raw cell id at this coord

      int bucketIndex = instanceHashTable[hd.hashBase];

      if (bucketIndex >= 0) {
        for (uint32_t i = 0; i < instanceHashTableBuckets[bucketIndex].size(); i++) {
          if (instanceHashTableBuckets[bucketIndex][i].rawCellId == hd.rawCellId) {
            // If we found a matching cell already, append this point
            instancePointCells[instanceHashTableBuckets[bucketIndex][i].pointCellIndex]
                .pointIndices.push_back(pointId);
            found = true;
            break;
          }
        }
      }

      // If we found a cell, we're done
      if (found)
        continue;

      // else we need to allocate a new cell and store the index in the hash table
      // Check if we already have a bucket
      if (instanceHashTable[hd.hashBase] < 0) {
        instanceHashTable[hd.hashBase] = numAllocatedBuckets;
        instanceHashTableBuckets.push_back({});
        numAllocatedBuckets++;
      }

      // Otherwise create a bucket/cell
      bucketIndex = instanceHashTable[hd.hashBase];
      instanceHashTableBuckets[bucketIndex].push_back({});
      instanceHashTableBuckets[bucketIndex].back().rawCellId = hd.rawCellId;
      instanceHashTableBuckets[bucketIndex].back().pointCellIndex = numAllocatedCells;

      instancePointCells.push_back({});
      instancePointCells[numAllocatedCells].rawCellId = hd.rawCellId;
      instancePointCells[numAllocatedCells].pointIndices.push_back(pointId);
      numAllocatedCells++;
    }

    // Try out a NN search with the hash table

    // Compress / copy into final buffers
    // First copy the cells into a large buffer, and adjust the offsets to go directly into that
    // So we iterate through the buckets, find each cell, copy it, and store the correct offset
    // then we do the same for the buckets / main hash
    // If we choose a memory management scheme that is easier to update, we likely have fixed
    // bucket/grid cell capacities.

    std::vector<uint32_t> compactCells;
    std::vector<Falcor::HashBucketInfo> compactBuckets;

    for (uint32_t bucketId = 0; bucketId < instanceHashTableBuckets.size(); bucketId++) {
      auto& bucket = instanceHashTableBuckets[bucketId];

      for (uint32_t cellIndex = 0; cellIndex < bucket.size(); cellIndex++) {
        const auto& cell = instancePointCells[bucket[cellIndex].pointCellIndex];

        uint32_t copyOffset = compactCells.size();
        compactCells.resize(compactCells.size() + cell.pointIndices.size());

        // Copy cells into buffer
        std::copy(
            cell.pointIndices.begin(), cell.pointIndices.end(), compactCells.begin() + copyOffset);

        // Overwrite bucket offset and store number of elements in bucket as well
        // We won't need it anymore
        bucket[cellIndex].pointCellIndex =
            copyOffset + (cell.pointIndices.size() << COMPACT_HASH_INDEX_BITS);
      }
    }

    for (uint32_t hashId = 0; hashId < instanceHashTable.size(); hashId++) {
      int32_t& bucketId = instanceHashTable[hashId];

      if (bucketId < 0)
        continue;

      // Loop over all buckets and compact them, adjusting the pointers in the hash table
      const auto& bucket = instanceHashTableBuckets[bucketId];

      uint32_t copyOffset = compactBuckets.size();
      compactBuckets.resize(compactBuckets.size() + bucket.size());

      std::copy(bucket.begin(), bucket.end(), compactBuckets.begin() + copyOffset);

      // Overwrite hash table offset to contain number of elements as well
      bucketId = copyOffset + (bucket.size() << COMPACT_HASH_INDEX_BITS);
    }

    // Copy everything into large (cpu) buffers with offsets
    Falcor::InstanceHashInfo ihf;

    ihf.aabbMax = instanceAABB.max;
    ihf.aabbMin = instanceAABB.min;
    ihf.hashToBucketOffset = hashToBucket_.size();

    ihf.hashToBucketSize = instanceHashTable.size();
    ihf.hashBucketToPointCellOffset = hashBucketToPointCell_.size();
    ihf.pointCellOffset = pointCells_.size();
    ihf.gridDim = gridDim;

    hashToBucket_.resize(hashToBucket_.size() + instanceHashTable.size());
    std::copy(
        instanceHashTable.begin(),
        instanceHashTable.end(),
        hashToBucket_.begin() + ihf.hashToBucketOffset);

    hashBucketToPointCell_.resize(hashBucketToPointCell_.size() + compactBuckets.size());
    std::copy(
        compactBuckets.begin(),
        compactBuckets.end(),
        hashBucketToPointCell_.begin() + ihf.hashBucketToPointCellOffset);

    pointCells_.resize(pointCells_.size() + compactCells.size());
    std::copy(compactCells.begin(), compactCells.end(), pointCells_.begin() + ihf.pointCellOffset);

    instanceHashInfo_.push_back(ihf);
  }

  // After all instances have been processed, we can generate the GPU buffers
  gpuHashToBucket_ = Falcor::Buffer::createStructured(
      sizeof(int),
      hashToBucket_.size(),
      Falcor::ResourceBindFlags::ShaderResource | Falcor::ResourceBindFlags::UnorderedAccess,
      Falcor::Buffer::CpuAccess::None,
      hashToBucket_.data());

  gpuHashBucketToPointCell_ = Falcor::Buffer::createStructured(
      sizeof(Falcor::HashBucketInfo),
      hashBucketToPointCell_.size(),
      Falcor::ResourceBindFlags::ShaderResource | Falcor::ResourceBindFlags::UnorderedAccess,
      Falcor::Buffer::CpuAccess::None,
      hashBucketToPointCell_.data());

  gpuPointCells_ = Falcor::Buffer::createStructured(
      sizeof(uint32_t),
      pointCells_.size(),
      Falcor::ResourceBindFlags::ShaderResource | Falcor::ResourceBindFlags::UnorderedAccess,
      Falcor::Buffer::CpuAccess::None,
      pointCells_.data());

  gpuInstanceHashInfo_ = Falcor::Buffer::createStructured(
      sizeof(Falcor::InstanceHashInfo),
      instanceHashInfo_.size(),
      Falcor::ResourceBindFlags::ShaderResource | Falcor::ResourceBindFlags::UnorderedAccess,
      Falcor::Buffer::CpuAccess::None,
      instanceHashInfo_.data());
}

} // namespace split_rendering
