#include "MeshPointGenerator.h"
#include <glm/gtx/matrix_decompose.hpp>
#include <algorithm>
#include <execution>
#include "../poisson_sampling/cySampleElim.h"
#include "Core/API/Device.h"
#include <filesystem>
#include <fstream>

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

    // TODO: check if file exists and load, otherwise don't load
    // need file path that consists of num samples and is stored alongside the mesh/scene somewhere


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
  constexpr uint32_t kMinSamplesPerTriangle = 1;
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
    float scaleFactor = glm::abs(glm::max(glm::max(scale.x, scale.y), scale.z));

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

        if (checkIfPoissonSamplesExist(scene, instanceId))
          return;

        const auto& meshDesc = scene->getMesh(Falcor::MeshID{instance.geometryID});
        uint32_t numTriangles = meshDesc.indexCount / 3;

        auto constantBuffer = pointGenRtVars_["perFrameConstantBuffer"];
        constantBuffer["instanceId"] = instanceId;
        constantBuffer["uniformSamplesOffset"] = uniformSamplesOffset[instanceId];
        constantBuffer["numTriangles"] = numTriangles;

        std::cout << "Creating triangle GPU buffers for uniform sampling "
          << "(" << instanceId << " / " << instanceIds.size() << "): num tris: "
          << numTriangles << ", "
          << triangleSampleCountsPerInstance[instanceId].size() * sizeof(uint32_t) << ", "
          << triangleSampleOffsetsPerInstance[instanceId].size() * sizeof(uint32_t) << "\n";

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

        std::cout << "Uniform point gen done:  "
          << "(" << instanceId << " / " << instanceIds.size() << ")\n";

        renderContext->flush(true);
        
      });
}

std::string MeshPointGenerator::getPoissonSampleFileName(Falcor::Scene::SharedPtr& scene, uint32_t instanceID)
{
  const auto& instance = scene->getGeometryInstance(instanceID);

  // TODO: we actually want to create a filename based on name and a hash of the bbox / matrix / whatever else
  std::string full_unique_id_suffix;

  const auto& animController = scene->getAnimationController();
  const auto& globalMatrices = animController->getGlobalMatrices();
  const auto& mesh = scene->getMesh(Falcor::MeshID{ instance.geometryID });

  std::string geometry_str_for_hash;

  for (uint32_t i = 0; i < 16; i++)
  {
    geometry_str_for_hash += std::to_string(globalMatrices[instance.globalMatrixID].data()[i]);
  }

  geometry_str_for_hash += std::to_string(mesh.indexCount) + std::to_string(mesh.vertexCount) + std::to_string(mesh.isDynamic()) + std::to_string(mesh.isDisplaced());

  // Get some vertices for the hash too
  auto& sceneData = scene->getSceneData();
  auto& meshVertex0 = sceneData.meshStaticData[instance.vbOffset];
  auto& meshVertex1 = sceneData.meshStaticData[instance.vbOffset + 1];

  geometry_str_for_hash += std::to_string(meshVertex0.position.x) + std::to_string(meshVertex0.texCrd.y) + std::to_string(meshVertex1.position.z) + std::to_string(meshVertex1.texCrd.y);


  std::hash<std::string> stringHasher;

  //full_unique_id_suffix += std::to_string(instance.materialID) + "_" + std::to_string(stringHasher(matrix_str));
  full_unique_id_suffix += std::to_string(stringHasher(geometry_str_for_hash));

  std::string meshInstanceName = "mesh_" + scene->getMaterial(Falcor::MaterialID{ instance.materialID })->getName() + "_" + full_unique_id_suffix;
  std::string scenePath = "poisson_caches";//scene->getPath().parent_path().string();
  std::string sceneName = scene->getPath().filename().replace_extension("").string();
  std::string poissonPointsPath = scenePath + "/poisson_points_cache_" + sceneName + "_" + std::to_string(kNumSamplesPerUnitSquaredEliminated)
    + "_" + std::to_string(kMinSamplesPerInstance) + "_" + std::to_string(kSamplesEliminatedFactor) + "/" + meshInstanceName + ".poisson";

  return poissonPointsPath;
}

bool MeshPointGenerator::checkIfPoissonSamplesExist(Falcor::Scene::SharedPtr& scene, uint32_t instanceID)
{
  return std::filesystem::exists(getPoissonSampleFileName(scene, instanceID));
}

void MeshPointGenerator::loadPoissonSamples(std::string filePath, std::vector<Falcor::PointData>& output, uint32_t outputOffset)
{
  std::ifstream file(filePath, std::ios::binary);
  file.unsetf(std::ios::skipws);

  //get length of file
  file.seekg(0, std::ios::end);
  size_t length = file.tellg();
  file.seekg(0, std::ios::beg);

  file.read((char*) &output[outputOffset], length);
  //output.insert(output.begin() + outputOffset, std::istream_iterator<Falcor::PointData>(file), std::istream_iterator<Falcor::PointData>());
}

void MeshPointGenerator::savePoissonSamples(std::string filePath, std::vector<Falcor::PointData>& output, uint32_t outputOffset, uint32_t numSamples)
{
  std::ofstream file(filePath, std::ios::binary);
  file.write((const char*) &output[outputOffset], sizeof(Falcor::PointData) * numSamples);
}

void MeshPointGenerator::generatePointsInScene(
    Falcor::Scene::SharedPtr& scene,
    Falcor::RenderContext* renderContext) {

  auto rng = std::default_random_engine{};

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
  //int dummy_int_count = 100;
  std::vector<uint32_t> instanceIds(instanceCount);
  for (uint32_t instanceId = 0; instanceId < instanceIds.size(); instanceId++) {
    instanceIds[instanceId] = instanceId;
  }

  // Create directory for poisson point caching
  auto poissonFilename = getPoissonSampleFileName(scene, 0);
  std::filesystem::create_directories(std::filesystem::path{ poissonFilename }.remove_filename());

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

  std::mutex poissonCacheFileMutex;

  // NOTE: this could use dispenso::for_each which might have better performance
  std::for_each(
      std::execution::par, instanceIds.begin(), instanceIds.end(), [&](uint32_t& instanceId) {
        const auto& instance = scene->getGeometryInstance(instanceId);

        //if (instanceId == 90)
        //  return;

        bool use16Bit =
            (instance.flags & (uint32_t)Falcor::GeometryInstanceFlags::Use16BitIndices) > 0;

        uint32_t numPlacedSamples = 0;

        const auto& meshDesc = scene->getMesh(Falcor::MeshID{instance.geometryID});

        float instanceDiskRadius = 0.0f;

        bool isDoubleSided =
            scene->getMaterial(Falcor::MaterialID{instance.materialID})->isDoubleSided();


        std::string poissonFilename = getPoissonSampleFileName(scene, instanceId);

        poissonCacheFileMutex.lock();
        bool fileExists = checkIfPoissonSamplesExist(scene, instanceId);
        poissonCacheFileMutex.unlock();

        if (fileExists)
        {
          // Set disk radius
          cy::WeightedSampleElimination<Falcor::float3, float, 3, uint32_t> wse;
          float diameterMax = 2.0f * wse.GetMaxPoissonDiskRadius(2, numSamplesPerInstance_[instanceId], totalSurfaceAreas[instanceId]);
          diskRadiusPerInstance_[instanceId] = diameterMax / 2;

          // Load file into output
          poissonCacheFileMutex.lock();
          loadPoissonSamples(poissonFilename, pointData_, sampleOffsetPerInstance_[instanceId]);
          poissonCacheFileMutex.unlock();

          std::cout << "Loaded poisson disk samples done from " << poissonFilename << " ( " << instanceId << " / " << instanceIds.size() - 1 << " )\n";
        }
        else
        {

          std::string exception_mesh = "mesh_tile_floor_rubber_mat_5591710445552534511.poisson";
          if (poissonFilename.compare(poissonFilename.length() - exception_mesh.length(), exception_mesh.length(), exception_mesh) == 0)
          {
            // Do random sampling of the uniform samples... this mesh takes ages to optimize otherwise.
            std::vector<uint32_t> randIndices(uniformSamplesCount[instanceId]);

            for (uint32_t i = 0; i < randIndices.size(); i++)
              randIndices[i] = i;

            std::shuffle(std::begin(randIndices), std::end(randIndices), rng);

            for (uint32_t outputIndex = 0; outputIndex < numSamplesPerInstance_[instanceId]; outputIndex++)
            {
              pointData_[outputIndex + sampleOffsetPerInstance_[instanceId]] = uniformPointData[randIndices[outputIndex] + uniformSamplesOffset[instanceId]];
            }

            // Set disk radius
            cy::WeightedSampleElimination<Falcor::float3, float, 3, uint32_t> wse;
            float diameterMax = 2.0f * wse.GetMaxPoissonDiskRadius(2, numSamplesPerInstance_[instanceId], totalSurfaceAreas[instanceId]);
            diskRadiusPerInstance_[instanceId] = diameterMax / 2;
            numPlacedSamples = numSamplesPerInstance_[instanceId];

          }
          else
          {
            std::cout << "Poisson disk sampling start for " << poissonFilename << " " << instanceId << " / " << instanceIds.size() - 1 << "\n";
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

          }


          // Save points into file
          poissonCacheFileMutex.lock();

          if (checkIfPoissonSamplesExist(scene, instanceId))
          {
            int a = 3;
            getPoissonSampleFileName(scene, instanceId);
          }
          savePoissonSamples(poissonFilename, pointData_, sampleOffsetPerInstance_[instanceId], numPlacedSamples);
          poissonCacheFileMutex.unlock();

          std::cout << "Poisson disk sampling done for " << poissonFilename << " " << instanceId << " / " << instanceIds.size() - 1 << "\n";
        }

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
