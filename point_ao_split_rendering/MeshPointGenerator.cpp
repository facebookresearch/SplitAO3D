#include "MeshPointGenerator.h"
#include <glm/gtx/matrix_decompose.hpp>
#include <algorithm>
#include <execution>
#include "../poisson_sampling/cySampleElim.h"

namespace split_rendering {

uint32_t MeshPointGenerator::samplePoissonDiskOnMeshOffsets(
    const Falcor::PointData* uniformPointData,
    uint32_t uniformDataOffset,
    uint32_t numTriangles,
    uint32_t numSamplesUniform,
    uint32_t numSamplesAfterElimination,
    float totalSurfaceArea,
    bool isDoubleSided,
    float& instanceDiskRadius,
    uint32_t outputOffset,
    std::vector<Falcor::PointData>& output) {
  // Split into front-side and back-side points if we have a double-sided material,
  // otherwise the poisson disk sampling considers points on the opposing side for elimination
  if (isDoubleSided) {
    // this is kind of a hack, but can be handled differently later.
    std::vector<Falcor::float3> frontInputPoints;
    std::vector<uint32_t> frontInputIds;
    std::vector<Falcor::float3> backInputPoints;
    std::vector<uint32_t> backInputIds;
    std::vector<uint32_t> frontOutputIds((numSamplesAfterElimination + 1) / 2);
    std::vector<uint32_t> backOutputIds((numSamplesAfterElimination + 1) / 2);

    cy::WeightedSampleElimination<Falcor::float3, float, 3, uint32_t> wse;
    float diameterMaxFull =
        2.0f * wse.GetMaxPoissonDiskRadius(2, numSamplesAfterElimination, totalSurfaceArea);
    instanceDiskRadius = diameterMaxFull / 2.0f;
    float diameterMax = 2.0f *
        wse.GetMaxPoissonDiskRadius(2, (numSamplesAfterElimination + 1) / 2, totalSurfaceArea);

    for (uint32_t i = 0; i < numSamplesUniform; i++) {
      bool frontSided = uniformPointData[uniformDataOffset + i].value > 0;

      if (frontSided) {
        frontInputPoints.push_back(Falcor::float3(
            uniformPointData[uniformDataOffset + i].position.x,
            uniformPointData[uniformDataOffset + i].position.y,
            uniformPointData[uniformDataOffset + i].position.z));
        frontInputIds.push_back(i);
      } else {
        backInputPoints.push_back(Falcor::float3(
            uniformPointData[uniformDataOffset + i].position.x,
            uniformPointData[uniformDataOffset + i].position.y,
            uniformPointData[uniformDataOffset + i].position.z));
        backInputIds.push_back(i);
      }
    }

    wse.EliminateID(
        frontInputPoints.data(),
        frontInputPoints.size(),
        frontOutputIds.data(),
        frontOutputIds.size(),
        diameterMax,
        2);

    wse.EliminateID(
        backInputPoints.data(),
        backInputPoints.size(),
        backOutputIds.data(),
        backOutputIds.size(),
        diameterMax,
        2);

    uint32_t outputId = 0;

    uint32_t num = numSamplesAfterElimination / 2;
    auto* upd = uniformPointData + uniformDataOffset;
    for (uint32_t i = 0, j = outputOffset; i < num; ++i) {
      output[j++] = upd[frontInputIds[frontOutputIds[i]]];
      output[j++] = upd[backInputIds[backOutputIds[i]]];
    }

  } else {
    std::vector<uint32_t> outputIds(numSamplesAfterElimination);

    std::vector<Falcor::float3> inputPoints(numSamplesUniform);
    cy::WeightedSampleElimination<Falcor::float3, float, 3, uint32_t> wse;
    float diameterMax = 2 * wse.GetMaxPoissonDiskRadius(2, outputIds.size(), totalSurfaceArea);

    instanceDiskRadius = diameterMax / 2;

    for (uint32_t i = 0; i < numSamplesUniform; i++) {
      inputPoints[i] = Falcor::float3(
          uniformPointData[uniformDataOffset + i].position.x,
          uniformPointData[uniformDataOffset + i].position.y,
          uniformPointData[uniformDataOffset + i].position.z);
    }
    wse.EliminateID(
        inputPoints.data(), inputPoints.size(), outputIds.data(), outputIds.size(), diameterMax, 2);

    for (uint32_t i = 0; i < outputIds.size(); i++) {
      output[i + outputOffset] = uniformPointData[uniformDataOffset + outputIds[i]];
    }
  }

  return numSamplesAfterElimination;
}

float MeshPointGenerator::getTotalSurfaceArea(
    const std::vector<uint32_t>& indexBuffer,
    std::vector<Falcor::PackedStaticVertexData>& vertexBuffer,
    uint32_t indexOffset,
    uint32_t vertexOffset,
    uint32_t numTriangles,
    bool use16BitIndices,
    std::vector<float>& outTriangleAreas) {
  double triangleAreaTotal = 0.0f;
  uint32_t numPlacedSamples = 0;
  uint32_t instanceTriangleIndex = 0;

  outTriangleAreas.resize(numTriangles);

  // NOTE: This could be easily parallelized
  for (uint32_t idxStart = indexOffset; idxStart < indexOffset + numTriangles * 3; idxStart += 3) {
    Falcor::float3 vertices[3] = {
        vertexBuffer[vertexOffset + getIndex(indexBuffer, idxStart, use16BitIndices)].position,
        vertexBuffer[vertexOffset + getIndex(indexBuffer, idxStart + 1, use16BitIndices)].position,
        vertexBuffer[vertexOffset + getIndex(indexBuffer, idxStart + 2, use16BitIndices)].position};

    Falcor::float3 crossVec = glm::cross((vertices[0] - vertices[2]), (vertices[1] - vertices[2]));

    float triangleArea = 0.5f * glm::length(crossVec);
    outTriangleAreas[instanceTriangleIndex++] = triangleArea;

    triangleAreaTotal += triangleArea;
  }

  return triangleAreaTotal;
}

std::vector<uint32_t> MeshPointGenerator::getNumSamplesPerTriangle(
    const std::vector<uint32_t>& indexBuffer,
    std::vector<Falcor::PackedStaticVertexData>& vertexBuffer,
    uint32_t indexOffset,
    uint32_t vertexOffset,
    uint32_t numTriangles,
    uint32_t numSamples,
    const std::vector<float>& triangleAreas,
    bool use16BitIndices,
    bool isDoubleSided,
    float totalSurfaceArea,
    uint32_t* numPlacedSamples) {
  static SeededRandom seededRandom;
  std::uniform_real_distribution<float> uniformRand(0.0, 1.0);

  std::vector<uint32_t> triangleSamples(numTriangles);

  uint32_t instanceTriangleIndex = 0;

  // Spawn at least three (uniform) samples per triangle, more if we have budget -> this helps cover
  // the area better for highly non-uniform meshes or lots of transparency. This also helps
  // fine/intricate geometry that has a very small area (plants / trees / foliage).
  float remainderTotal = 0.0f;
  uint32_t actualSamples = 0;
  constexpr uint32_t kMinSamplesPerTriangle = 3;
  const float rcpTriAreaTotal = numSamples / totalSurfaceArea;

  // NOTE: this loop could be parallelized
  for (uint32_t tri_idx = 0; tri_idx < triangleSamples.size(); tri_idx++) {
    triangleSamples[tri_idx] = kMinSamplesPerTriangle;
    actualSamples += kMinSamplesPerTriangle;

    if (isDoubleSided) {
      triangleSamples[tri_idx] += kMinSamplesPerTriangle;
      actualSamples += kMinSamplesPerTriangle;
    }

    float weightedProbability = triangleAreas[tri_idx] * rcpTriAreaTotal;
    float intPart;

    remainderTotal += std::modf(weightedProbability, &intPart);
    triangleSamples[tri_idx] += (uint32_t) intPart;
    actualSamples += (uint32_t) intPart;
  }

  while (actualSamples < numSamples) {
    uint32_t randomIdx = (uint32_t)(uniformRand(seededRandom.random_engine) * numTriangles);
    triangleSamples[randomIdx]++;
    actualSamples++;
  }

  if (numPlacedSamples)
    *numPlacedSamples = actualSamples;

  return triangleSamples;
}

void MeshPointGenerator::computeSampleCounts(
    Falcor::Scene::SharedPtr& scene,
    std::vector<float>& totalSurfaceAreas,
    std::vector<std::vector<uint32_t>>& triangleSampleCountsPerInstance,
    std::vector<std::vector<uint32_t>>& triangleSampleOffsetsPerInstance,
    std::vector<uint32_t>& uniformSamplesCount,
    std::vector<uint32_t>& uniformSamplesOffset,
    uint32_t& numSamplesTotal,
    uint32_t& numUniformSamplesTotal) {
  constexpr uint32_t kMinUniformSamplesPerInstance =
      kMinSamplesPerInstance * kSamplesEliminatedFactor;
  constexpr uint32_t kNumSamplesPerUnitSquaredUniform =
      kNumSamplesPerUnitSquaredEliminated * kSamplesEliminatedFactor;

  auto& sceneData = scene->getSceneData();
  auto& meshStaticData = sceneData.meshStaticData;
  const auto& meshIndexData = sceneData.meshIndexData;
  const auto& globalTransforms = scene->getAnimationController()->getGlobalMatrices();

  const auto instanceCount = scene->getGeometryInstanceCount();

  triangleSampleCountsPerInstance.resize(instanceCount);
  triangleSampleOffsetsPerInstance.resize(instanceCount);

  uint32_t instanceTriangleOffset = 0;
  uint32_t prevPlacedSamples = 0;
  // Based on the total surface area, we can compute the number of samples that we want to have
  // If we have that, we can parallelize the generation
  for (uint32_t instanceId = 0; instanceId < instanceCount; instanceId++) {
    const auto& instance = scene->getGeometryInstance(instanceId);
    bool use16Bit = (instance.flags & (uint32_t)Falcor::GeometryInstanceFlags::Use16BitIndices) > 0;

    uint32_t numPlacedSamples = 0;

    const auto& meshDesc = scene->getMesh(Falcor::MeshID{instance.geometryID});
    std::vector<float> instanceTriangleAreas;

    instanceTriangleOffset += meshDesc.getTriangleCount();

    totalSurfaceAreas.push_back(getTotalSurfaceArea(
        meshIndexData,
        meshStaticData,
        instance.ibOffset * (use16Bit ? 2 : 1),
        instance.vbOffset,
        meshDesc.indexCount / 3,
        use16Bit,
        instanceTriangleAreas));

    if (sampleOffsetPerInstance_.empty()) {
      sampleOffsetPerInstance_.push_back(0);
    } else {
      sampleOffsetPerInstance_.push_back(
          sampleOffsetPerInstance_.back() + numSamplesPerInstance_.back());
    }

    // We keep the surface areas in "unit space" for the poisson disk sampling and poisson disk
    // radius, but increase/decrease the number of samples according to scale
    const auto& localToWorld = globalTransforms[instance.globalMatrixID];
    glm::vec3 scale;
    glm::quat rotation;
    glm::vec3 translation;
    glm::vec3 skew;
    glm::vec4 perspective;
    decompose(localToWorld, scale, rotation, translation, skew, perspective);
    float scaleFactor = glm::max(glm::max(scale.x, scale.y), scale.z);

    numSamplesPerInstance_.push_back(std::max(
        (uint32_t)std::ceil(
            totalSurfaceAreas.back() * kNumSamplesPerUnitSquaredEliminated * scaleFactor),
        kMinSamplesPerInstance));

    // Increase number of samples for double-sided materials
    const bool isDoubleSided =
        scene->getMaterial(Falcor::MaterialID{instance.materialID})->isDoubleSided();
    if (isDoubleSided) {
      numSamplesPerInstance_.back() *= 2;
    }

    triangleSampleCountsPerInstance[instanceId] = getNumSamplesPerTriangle(
        meshIndexData,
        meshStaticData,
        instance.ibOffset * (use16Bit ? 2 : 1),
        instance.vbOffset,
        meshDesc.indexCount / 3,
        std::max(
            numSamplesPerInstance_[instanceId] * kSamplesEliminatedFactor,
            kMinUniformSamplesPerInstance),
        instanceTriangleAreas,
        use16Bit,
        isDoubleSided,
        totalSurfaceAreas.back(),
        &numPlacedSamples);

    numSamplesTotal += numSamplesPerInstance_.back();
    numUniformSamplesTotal += numPlacedSamples;
    uniformSamplesCount.push_back(numPlacedSamples);

    if (uniformSamplesOffset.empty()) {
      uniformSamplesOffset.push_back(0);
    } else {
      uniformSamplesOffset.push_back(uniformSamplesOffset.back() + prevPlacedSamples);
    }

    prevPlacedSamples = numPlacedSamples;

    // Compute the offsets of all triangles so we don't even need atomics
    // NOTE: This could be simplyfied by only computing this for each unique combination of instance
    // and its transform as we only need this for each unique combination
    triangleSampleOffsetsPerInstance[instanceId].push_back(0);
    for (uint32_t sampleId = 0; sampleId < triangleSampleCountsPerInstance[instanceId].size() - 1;
         sampleId++) {
      triangleSampleOffsetsPerInstance[instanceId].push_back(
          triangleSampleOffsetsPerInstance[instanceId].back() +
          triangleSampleCountsPerInstance[instanceId][sampleId]);
    }
  }
}

void MeshPointGenerator::computeUniformSamples(
    Falcor::Scene::SharedPtr& scene,
    Falcor::RenderContext* renderContext,
    Falcor::Buffer::SharedPtr& uniformPointsGpu,
    std::vector<uint32_t>& instanceIds,
    std::vector<uint32_t>& uniformSamplesOffset,
    std::vector<std::vector<uint32_t>>& triangleSampleCountsPerInstance,
    std::vector<std::vector<uint32_t>>& triangleSampleOffsetsPerInstance) {
  // NOTE: this could use dispenso::for_each which might have better performance
  std::for_each(
      std::execution::seq, instanceIds.begin(), instanceIds.end(), [&](uint32_t& instanceId) {
        // From the precomputed sample counts (and precomputed offsets), we can start a compute/RT
        // shader for each instance and generate the pointData_ data

        // This could be replaced by a simple compute shader, but the RT shader already binds all
        // the things that we need, so that is more convenient

        const auto& instance = scene->getGeometryInstance(instanceId);
        const auto& meshDesc = scene->getMesh(Falcor::MeshID{instance.geometryID});
        uint32_t numTriangles = meshDesc.indexCount / 3;

        auto constantBuffer = pointGenRtVars_["perFrameConstantBuffer"];
        constantBuffer["instanceId"] = instanceId;
        constantBuffer["uniformSamplesOffset"] = uniformSamplesOffset[instanceId];
        constantBuffer["numTriangles"] = numTriangles;

        // Generating this for each instance is also not great, should be generated once and indexed
        // with instance offsets / flattened
        Falcor::Buffer::SharedPtr triangleSampleCountsGpu = Falcor::Buffer::createStructured(
            sizeof(uint32_t),
            triangleSampleCountsPerInstance[instanceId].size(),
            Falcor::ResourceBindFlags::ShaderResource | Falcor::ResourceBindFlags::UnorderedAccess,
            Falcor::Buffer::CpuAccess::None,
            triangleSampleCountsPerInstance[instanceId].data());

        Falcor::Buffer::SharedPtr triangleSampleOffsetsGpu = Falcor::Buffer::createStructured(
            sizeof(uint32_t),
            triangleSampleOffsetsPerInstance[instanceId].size(),
            Falcor::ResourceBindFlags::ShaderResource | Falcor::ResourceBindFlags::UnorderedAccess,
            Falcor::Buffer::CpuAccess::None,
            triangleSampleOffsetsPerInstance[instanceId].data());

        // These flushes should be appropriate barriers, but it's during setup, so it doesn't matter
        // too much.
        renderContext->flush(true);

        pointGenRtVars_->setBuffer("triangleSampleCountsPerInstance", triangleSampleCountsGpu);
        pointGenRtVars_->setBuffer("triangleSampleOffsetsPerInstance", triangleSampleOffsetsGpu);
        pointGenRtVars_->setBuffer("uniformPoints", uniformPointsGpu);

        // We "trace" a ray for each triangle, and each triangle generates its points
        scene->raytrace(
            renderContext,
            pointGenRtProgram_.get(),
            pointGenRtVars_,
            Falcor::uint3(numTriangles, 1, 1));

        renderContext->flush(true);
      });
}

void MeshPointGenerator::generatePointsInScene(
    Falcor::Scene::SharedPtr& scene,
    Falcor::RenderContext* renderContext) {
  setupGPUUniformPointGenerationPipeline(scene);

  pointData_.clear();

  uint32_t instanceOffset = 0;

  uint32_t meshCount = scene->getMeshCount();

  const auto instanceCount = scene->getGeometryInstanceCount();

  diskRadiusPerInstance_.resize(instanceCount);

  std::vector<float> totalSurfaceAreas;

  std::vector<uint32_t> uniformSamplesOffset;
  std::vector<uint32_t> uniformSamplesCount;

  uint32_t numSamplesTotal = 0;
  uint32_t numUniformSamplesTotal = 0;

  std::vector<std::vector<uint32_t>> triangleSampleCountsPerInstance;
  std::vector<std::vector<uint32_t>> triangleSampleOffsetsPerInstance;

  // Compute and preallocate sample counts for pushing to the GPU uniform sample generation
  computeSampleCounts(
      scene,
      totalSurfaceAreas,
      triangleSampleCountsPerInstance,
      triangleSampleOffsetsPerInstance,
      uniformSamplesCount,
      uniformSamplesOffset,
      numSamplesTotal,
      numUniformSamplesTotal);

  // Prepare buffer that will hold all of the generated points on the GPU
  Falcor::Buffer::SharedPtr uniformPointsGpu = Falcor::Buffer::createStructured(
      sizeof(Falcor::PointData),
      numUniformSamplesTotal,
      Falcor::ResourceBindFlags::ShaderResource | Falcor::ResourceBindFlags::UnorderedAccess,
      Falcor::Buffer::CpuAccess::None);

  // Prepare indices for parallel for
  std::vector<uint32_t> instanceIds(instanceCount);
  for (uint32_t instanceId = 0; instanceId < instanceCount; instanceId++) {
    instanceIds[instanceId] = instanceId;
  }

  // Compute uniform samples on the GPU
  computeUniformSamples(
      scene,
      renderContext,
      uniformPointsGpu,
      instanceIds,
      uniformSamplesOffset,
      triangleSampleCountsPerInstance,
      triangleSampleOffsetsPerInstance);

  renderContext->flush(true);

  // Map the uniform points and pass them to the poisson disk sampling
  const Falcor::PointData* uniformPointData =
      (const Falcor::PointData*)uniformPointsGpu->map(Falcor::Buffer::MapType::Read);

  // Pre-allocate memory that will hold the resulting CPU poisson rejected points
  pointData_.resize(numSamplesTotal);

  // NOTE: this could use dispenso::for_each which might have better performance
  std::for_each(
      std::execution::par, instanceIds.begin(), instanceIds.end(), [&](uint32_t& instanceId) {
        const auto& instance = scene->getGeometryInstance(instanceId);

        bool use16Bit =
            (instance.flags & (uint32_t)Falcor::GeometryInstanceFlags::Use16BitIndices) > 0;

        uint32_t numPlacedSamples = 0;

        const auto& meshDesc = scene->getMesh(Falcor::MeshID{instance.geometryID});

        float instanceDiskRadius = 0.0f;

        bool isDoubleSided =
            scene->getMaterial(Falcor::MaterialID{instance.materialID})->isDoubleSided();

        numPlacedSamples = samplePoissonDiskOnMeshOffsets(
            uniformPointData,
            uniformSamplesOffset[instanceId],
            meshDesc.indexCount / 3,
            uniformSamplesCount[instanceId],
            numSamplesPerInstance_[instanceId],
            totalSurfaceAreas[instanceId],
            isDoubleSided,
            instanceDiskRadius,
            sampleOffsetPerInstance_[instanceId],
            pointData_);

        diskRadiusPerInstance_[instanceId] = instanceDiskRadius;
      });

  gpuDiskRadiusPerInstance_ = Falcor::Buffer::createStructured(
      sizeof(float),
      diskRadiusPerInstance_.size(),
      Falcor::ResourceBindFlags::ShaderResource | Falcor::ResourceBindFlags::UnorderedAccess,
      Falcor::Buffer::CpuAccess::None,
      diskRadiusPerInstance_.data());
}

void MeshPointGenerator::setupGPUUniformPointGenerationPipeline(
    Falcor::Scene::SharedPtr& scene) {
  // Uniform point generation RT pipeline (using RT/ray gen shader because it already has all the
  // scene things setup...)
 
  Falcor::RtProgram::Desc rtPointGenProgDesc;

  rtPointGenProgDesc.addShaderModules(scene->getShaderModules());
  rtPointGenProgDesc.addShaderLibrary("Samples/FalcorServer/PointGen.rt.slang");

  // We're not actually tracing rays.
  rtPointGenProgDesc.setMaxTraceRecursionDepth(0);
  rtPointGenProgDesc.setMaxPayloadSize(0);

  Falcor::RtBindingTable::SharedPtr pointGenSbt =
      Falcor::RtBindingTable::create(0, 0, scene->getGeometryCount());
  pointGenSbt->setRayGen(rtPointGenProgDesc.addRayGen("rayGen"));

  pointGenRtProgram_ = Falcor::RtProgram::create(rtPointGenProgDesc, scene->getSceneDefines());
  pointGenRtProgram_->setTypeConformances(scene->getTypeConformances());
  pointGenRtVars_ = Falcor::RtProgramVars::create(pointGenRtProgram_, pointGenSbt);
}

} // namespace split_rendering
