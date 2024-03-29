/*
 * Copyright (c) Meta Platforms, Inc. and affiliates.
 * All rights reserved.
 *
 * This source code is licensed under the license found in the
 * LICENSE file in the root directory of this source tree.
 */

import Scene.Raytracing;
import Utils.Sampling.UniformSampleGenerator;
import Rendering.Lights.LightHelpers;
import Utils.Math.MathHelpers;
import PointData;
import HashFunctionShared;
import Utils.Math.MatrixUtils;

#include "PointAOConstantsShared.slangh"

RWStructuredBuffer<uint> triangleVisibilityDataPerFrame;
RWStructuredBuffer<uint> triangleVisibilityOffsetData;

cbuffer perFrameConstantBuffer
{
  float4x4 extendedViewProjectionMatrix;
  uint sampleIndex;
  float aoRadius;
  int aoSamples;
  uint raytracePoints;
  float updateDeltaCosThreshold;
  float updateDeltaPosFactor;
  float updateDeltaValFactor;
};


struct AORayData
{
	float hitT;
};

/*
float3 randomSampleCosineHemisphere(PointData pd, float2 rng)
{
	float pdf;
	float3 relativeVec = normalize(sample_cosine_hemisphere_concentric(rng, pdf));
  
  float3 bitangent = cross(pd.normal, pd.tangent);
  return normalize(relativeVec.x * pd.tangent + relativeVec.y * bitangent + relativeVec.z * pd.normal);
}*/

float3 randomSampleCosineHemisphereSD(ShadingData sd, float2 rng)
{
	float pdf;
	float3 relativeVec = normalize(sample_cosine_hemisphere_concentric(rng, pdf));
	
  return normalize(sd.fromLocal(relativeVec));
}

[shader("miss")]
void aoMiss(inout AORayData hitData)
{
	hitData.hitT = aoRadius;
}

[shader("closesthit")]
void aoClosestHit(inout AORayData hitData, in BuiltInTriangleIntersectionAttributes attribs)
{
  hitData.hitT = RayTCurrent();
}

[shader("anyhit")]
void aoAnyHit(
    inout AORayData rayData,
    BuiltInTriangleIntersectionAttributes attribs)
{
  // Alpha test for non-opaque geometry.
  GeometryInstanceID instanceID = getGeometryInstanceID();
  VertexData v = getVertexData(instanceID, PrimitiveIndex(), attribs);
  uint materialID = gScene.getMaterialID(instanceID);
  if (gScene.materials.alphaTest(v, materialID, 0.f))
  {
    IgnoreHit();
  }
}

float raytraceAO(ShadingData sd, UniformSampleGenerator sg)
{
  float aoTerm = 0.0f;
  
  // Main sample loop for averaging the raytraced AO samples
  for (int i = 0; i < aoSamples; i++)
  {
    float3 direction = randomSampleCosineHemisphereSD(sd, float2(sampleNext1D(sg), sampleNext1D(sg)));

    RayDesc ray;
    ray.Origin = sd.posW + sd.N * 0.01f;
    ray.Direction = normalize(direction);
    ray.TMin = 0;
    ray.TMax = aoRadius;
      
    AORayData rayData;
    
    TraceRay(gScene.rtAccel, 0 , 0xFF, 0 , rayTypeCount, 0, ray, rayData);
    
    aoTerm += rayData.hitT / aoRadius;
  }
    
  aoTerm /= (float) aoSamples;
  
  return aoTerm;
}


[shader("raygeneration")]
void rayGen()
{
  // This shader traces rays from each point and is responsible for determining the changes (value, normal, position) of points. 

  uint3 launchIndex = DispatchRaysIndex();
  
  if (serverAOValues[launchIndex.x] < 0)
    return;
  
  //PointData pt = serverAOPoints[launchIndex.x];

  uint pt_instanceId = serverAOInstanceIDs[launchIndex.x];
  uint pt_instanceTriangleId = serverAOInstanceTriangleIDs[launchIndex.x];
  float2 pt_barycentrics = serverAOBarycentrics[launchIndex.x];
  float3 pt_normal = serverAONormals[launchIndex.x];

  //if (triangleVisibilityDataPerFrame[triangleVisibilityOffsetData[pt_instanceId] + pt_instanceTriangleId] == 0)
  //  return;
  
  const GeometryInstanceID instanceID = { pt_instanceId };
  
  const GeometryInstanceData instance = gScene.getGeometryInstance(instanceID);
  float4x4 worldMat = gScene.loadWorldMatrix(instance.globalMatrixID);

  float4x4 shadingWorldMat = gScene.loadWorldMatrix(instance.globalMatrixID);
  float3x3 worldInvTransposeMat = gScene.loadInverseTransposeWorldMatrix(instance.globalMatrixID);
  
  // We use the barycentrics to get the new point location with the current animation state for the given triangle id.
  const float3 barycentrics = {pt_barycentrics.x , pt_barycentrics.y, 1.0 - pt_barycentrics.x - pt_barycentrics.y };
  
  VertexData v = gScene.getVertexData(instanceID, pt_instanceTriangleId, barycentrics);
  uint materialID = gScene.getMaterialID(instanceID);
  
  let lod = ExplicitLodTextureSampler(0.f);
  
  pt_normal = normalize(mul(gScene.getInverseTransposeWorldMatrix(instanceID), pt_normal));
  ShadingData sd = gScene.materials.prepareShadingData(v, materialID, pt_normal, lod);

  if (!sd.frontFacing && sd.mtl.isDoubleSided())
  {
      sd.N = -sd.N;
  }

  // Transform world space pos with extended view projection matrix

  float4 posNDCExtended = mul(extendedViewProjectionMatrix, float4(sd.posW, 1.0f));

  posNDCExtended /= posNDCExtended.w;

  // check if any coordinate is outside [-1,-1] <-> [1, 1] NDC
  bool insideExtendedView = true;

  if (posNDCExtended.x < -1 || posNDCExtended.x > 1 || posNDCExtended.y < -1 || posNDCExtended.y > 1 || posNDCExtended.z < -1 || posNDCExtended.z > 1)
    insideExtendedView = false;


  if(!insideExtendedView)
  {
    return;
  }

  UniformSampleGenerator sg = UniformSampleGenerator(launchIndex.x, pt_instanceId);

  //float aoTerm = serverAOPoints[launchIndex.x].value;
  float aoTerm = serverAOValues[launchIndex.x];

  // Only raytrace points if they are rasterized in extended FOV - can do this directly here
  if(raytracePoints && insideExtendedView)
    aoTerm = raytraceAO(sd, sg);

  float diskRadius = DISK_RADIUS_FACTOR * instanceToDiskRadius[pt_instanceId];
  
  InstancePointInfo ipi = instancePointInfo[pt_instanceId];
  InstanceHashInfo ihf = serverInstanceHashInfo[pt_instanceId];
  float3 aabbSize = ipi.aabbMax - ipi.aabbMin;
  
  // We store the old position, normal and value to compare them 
  //float3 oldPosition = serverAOPoints[launchIndex.x].position;
  //float3 oldNormal = serverAOPoints[launchIndex.x].normal;
  //float oldValue = serverAOPoints[launchIndex.x].value;

  float3 oldPosition = serverAOPositions[launchIndex.x];
  float3 oldNormal = serverAONormals[launchIndex.x];
  float oldValue = serverAOValues[launchIndex.x];

  float3 newPosition = mul(sd.posW - extractTranslate(shadingWorldMat), worldInvTransposeMat);
  float3 newNormal = normalize(mul(sd.N, worldInvTransposeMat));
  float newValue = aoTerm;
  
  
  float3 posDelta = newPosition - oldPosition;
  float posDist = length(posDelta);
  float valDist = abs(newValue - oldValue);
  float cosN = dot(newNormal, oldNormal);
  
  const float cosThreshold = updateDeltaCosThreshold;
  const float posThreshold = updateDeltaPosFactor * diskRadius * ONE_OVER_POINT_POS_MAX;
  const float valThreshold = updateDeltaValFactor * ONE_OVER_POINT_VAL_MAX;
  
  int3 newCoords = ((newPosition - ipi.aabbMin)) / diskRadius;
  int3 oldCoords = ((oldPosition - ipi.aabbMin)) / diskRadius;
    
  HashData oldHashData = getHash(oldCoords, ipi.gridDim, ihf.hashToBucketSize, 0);
  HashData newHashData = getHash(newCoords, ipi.gridDim, ihf.hashToBucketSize, 0);
  
  oldHashData.hashBase *= FIXED_HASH_BUCKET_SIZE;
  newHashData.hashBase *= FIXED_HASH_BUCKET_SIZE;

  if(newHashData.rawCellId == oldHashData.rawCellId && cosN > cosThreshold && posDist < posThreshold && valDist < valThreshold)
  {
    // If the cell IDs are identical (and no change in value, normal or position was detected), we can early out - no memory management or cell updates necessary
    return;
  }

  
  // Update server points
  serverAOValues[launchIndex.x] = newValue;
 // serverAOPoints[launchIndex.x].value = newValue;
  
  serverAOPositions[launchIndex.x] = newPosition;
  serverAONormals[launchIndex.x] = newNormal;
  // Tangent is used for visualization only
  //serverAOPoints[launchIndex.x].tangent = //normalize(mul(sd.T, worldInvTransposeMat)); 
  
  InterlockedAdd(frameUpdateInfo[0].numChangedPoints, 1);
  
  uint cellDirtyRange = 1;
  uint2 dirtyCellOffsets;
  
  uint baseHashOffset = newHashData.hashBase + ihf.hashToBucketOffset;
  uint oldBaseHashOffset = oldHashData.hashBase + ihf.hashToBucketOffset;

  if(newHashData.rawCellId != oldHashData.rawCellId)
  {
    cellDirtyRange = 2;
 
    // First: disable the old points and generate data for the Allocation pass
    // Disabling (setting is_valid flag to false) can be done here without sync issues
      
    // Write out buffer for point alloc
    uint updateId = 0;
    
    // Update indirect dispatch args for point cell alloc stage.
    InterlockedAdd(cellAllocIndirectDispatchArgs[0].x, 1, updateId);

    pointUpdateData[updateId].newServerPointData.position = serverAOPositions[launchIndex.x];
    pointUpdateData[updateId].newServerPointData.normal = serverAONormals[launchIndex.x];
    pointUpdateData[updateId].newServerPointData.tangent = serverAOTangents[launchIndex.x];
    pointUpdateData[updateId].newServerPointData.barycentrics = serverAOBarycentrics[launchIndex.x];
    pointUpdateData[updateId].newServerPointData.instanceTriangleId = serverAOInstanceTriangleIDs[launchIndex.x];
    pointUpdateData[updateId].newServerPointData.instanceId = serverAOInstanceIDs[launchIndex.x];
    pointUpdateData[updateId].newServerPointData.value = serverAOValues[launchIndex.x];

    //pointUpdateData[updateId].newServerPointData = serverAOPoints[launchIndex.x];
    pointUpdateData[updateId].newHashData = newHashData;
    
    // Disable old point (will potentially be overwritten in the next pass)
    serverAOValues[launchIndex.x] = -1.0f;
    //serverAOPoints[launchIndex.x].value = -1.0f;
    compressedClientAOPoints[launchIndex.x].posNormVal = INVALID_CELL;
      
    // If the cell wasn't allocated yet, we allocate it and append its info to the hash table
    // At this point we ensured that only one thread per cell does this
    // Afterwards, we use a separate pass to generate/delete points into these cells
        
    int found = 0;
        
    // Check to see if we need to allocate a new cell or if one already exists
    for (uint hashBucketOffset = 0; hashBucketOffset < FIXED_HASH_BUCKET_SIZE; hashBucketOffset++) {
          
      uint oldHashEntry = 0;
         
      InterlockedCompareExchange(serverHashToPointCell[baseHashOffset + hashBucketOffset].rawCellId, INVALID_CELL, newHashData.rawCellId, oldHashEntry);
    
      if (oldHashEntry == INVALID_CELL)
      {
        // We found an empty cell in the hash table and can allocate memory in the cell buffer for it
        uint cellAllocOffset = 0;
        InterlockedAdd(instancePointInfo[pt_instanceId].numAllocatedCells, 1, cellAllocOffset);
        
        uint dummy = 0;
        InterlockedExchange(frameUpdateInfo[0].numAllocatedCells, instancePointInfo[pt_instanceId].numAllocatedCells, dummy);
        
        cellAllocOffset *= FIXED_POINTS_PER_CELL;
            
        uint numCells = 0;
        InterlockedExchange(serverHashToPointCell[baseHashOffset + hashBucketOffset].encodedIndex, cellAllocOffset, dummy);
      
        // Add to the number of buckets. 
        InterlockedAdd(hashNumBuckets[baseHashOffset / FIXED_HASH_BUCKET_SIZE].numBuckets, 1);
      
        found = 1;
        dirtyCellOffsets[1] = cellAllocOffset;
        
        
        // Also write out the hash table update buffer so we can prepare to send the updated hash entry to the client
        uint hashUpdateId = 0;
        InterlockedAdd(hashNetworkBufferIndirectDispatchArgs[0].x, 1, hashUpdateId);
      
        // Write cell dirty info - subsequent pass will prepare the network buffer
        // We need a subsequent pass because at this point, not all the points have been (de-)allocated yet.
        hashDirtyInfos[hashUpdateId] = baseHashOffset;
        
        break;
      }
      else if(oldHashEntry == newHashData.rawCellId)
      {
        // If we find a cell that contains our cell ID already, we can safely return, as this means that either another thread has already allocated it or it was preallocated initially
        dirtyCellOffsets[1] = serverHashToPointCell[baseHashOffset + hashBucketOffset].encodedIndex;
        break;
      }
    }
  }
  else
  {
    // If the raw cell ID matches, we can simply update the compressed point data - the client update will send the full cell later.
    compressedClientAOPoints[launchIndex.x] = compressClientData(serverAOPositions[launchIndex.x], serverAONormals[launchIndex.x], serverAOValues[launchIndex.x], diskRadius, ipi.aabbMin);
  }
  
  // Find old cell for the dirty offset
  for (uint hashBucketOffset = 0; hashBucketOffset < FIXED_HASH_BUCKET_SIZE; hashBucketOffset++)
  {
    if(serverHashToPointCell[oldBaseHashOffset + hashBucketOffset].rawCellId == oldHashData.rawCellId)
    {
      dirtyCellOffsets[0] = serverHashToPointCell[oldBaseHashOffset + hashBucketOffset].encodedIndex;
      break;
    }
  }
  
  // Finally, after creating the new cell (if it didn't exist), we can mark both the old and new cell as dirty
  // If we have a change in normals / positions / values *or* a change in cell ID, we mark the affected cells
  // we write out a cell update buffer using an atomic cell dirty flag
  // after the points were deallocated/allocated on the server, a final stage prepares the cell updates
  // for network transmission
  
  for(uint i = 0; i < cellDirtyRange; i++)
  {
    uint oldCellStatus;
    InterlockedCompareExchange(cellDirtyFlags[(dirtyCellOffsets[i] + ipi.pointCellOffset) / FIXED_POINTS_PER_CELL], CELL_NOT_DIRTY, CELL_DIRTY, oldCellStatus);
    
    if (oldCellStatus == CELL_NOT_DIRTY)
    {
      // If the cell was not dirty, we push the relevant info (global cell offset) into a buffer, and 
      // update the indirect dispatch args for the subsequent compute pass
      uint cellUpdateId = 0;
      InterlockedAdd(cellNetworkBufferIndirectDispatchArgs[0].x, 1, cellUpdateId);
      
      // Write cell dirty info - subsequent pass will prepare the network buffer
      // We need a subsequent pass because at this point, not all the points have been (de-)allocated yet.
      cellDirtyInfos[cellUpdateId] = dirtyCellOffsets[i] + ipi.pointCellOffset;
    }
  }
}
