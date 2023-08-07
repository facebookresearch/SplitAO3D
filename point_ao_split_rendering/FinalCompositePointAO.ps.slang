#include "Utils/Math/MathConstants.slangh"
#include "PointAOConstantsShared.slangh"

import Scene.Shading;
import Scene.Raster;
import Utils.Sampling.TinyUniformSampleGenerator;
import Utils.Geometry.GeometryHelpers;
import Rendering.Lights.LightHelpers;
import Rendering.Lights.EnvMapLighting;
import PointData;
import HashFunctionShared;
import KDTree;
import PointAOSamplingUtils;

cbuffer perFrameConstantBuffer
{
  bool aoOnly;
  bool colorOnly;
  int aoType;
  float aoRadius;
  float interpolationRadiusFactor;
  float cosNormalThreshold;
  float cosDeltaThreshold;
}

SamplerState mainSampler;
EnvMapLighting envMapLighting;

Texture2D aoTex;
Texture2D ssaoTex;

// Returns the point cell index for the densely packed (non-dynamic) hash table implementation
int getPointCellIndex(uint hashBase, uint rawCellId, InstanceHashInfo ihf)
{
  int bucketIndex = hashToBucket[hashBase + ihf.hashToBucketOffset];
  
  if(bucketIndex < 0)
    return -1;
  
  uint numBuckets = bucketIndex >> COMPACT_HASH_INDEX_BITS;
  bucketIndex &= COMPACT_HASH_INDEX_MASK;
  
  for (int hashBucketOffset = 0; hashBucketOffset < numBuckets; hashBucketOffset++) {
    
    HashBucketInfo hbi = hashBucketToPointCell[bucketIndex + hashBucketOffset + ihf.hashBucketToPointCellOffset];
    
    if (hbi.rawCellId == rawCellId) {
      // If we found a matching cell, we can return
      return hbi.pointCellIndex;
    }
  }
  
  return -1;
}


// Takes the base index in the hash table (and the offset for the given instance) and returns the point cell index and number of points
// It looks for any cell that matches the given rawCellId, which is computed from the hash function.
int getPointCellIndexUpdate(uint hashBase, uint hashBucketBase, uint rawCellId, uint hashToBucketOffset, out uint numPoints)
{
  uint baseHashOffset = hashBase + hashToBucketOffset;
  
  // TODO: handle division differently - for power-of-two this could be a shift instead, or we could store the offset already divided.
  uint numCells = hashNumBuckets[hashBucketBase + hashToBucketOffset / FIXED_HASH_BUCKET_SIZE].numBuckets;
  
  for (uint hashBucketOffset = 0; hashBucketOffset < numCells; hashBucketOffset++) {
      
      CompactHashToCellInfo htci = serverHashToPointCell[baseHashOffset + hashBucketOffset];
    
      if (htci.rawCellId == rawCellId) {
        return htci.encodedIndex;
      }
    }
  
  return -1;
}


// Sample an AO value from the compact (= non-dynamic) hash table
// Keeps track of the 16 nearest neighbors instead of blending between all of them
float samplePointAOHashNN(VSOut vsOut, uint triangleIndex, ShadingData sd, float3 barycentrics)
{
  AOShadingPoint sp = getInstanceShadingPointPreviousFrame(sd, vsOut, interpolationRadiusFactor, barycentrics, triangleIndex);
  
  InstanceHashInfo ihf = instanceHashInfo[vsOut.instanceID.index];
  
  float3 floatCoords = ((sp.posW - ihf.aabbMin)) / sp.diskRadius;
  int3 intBaseCoords = floatCoords - 0.5f;
  
  AOSamplingWeight samplingWeight = initSamplingWeight();
  
  ResultSet resultSet;
  resultSet.count = 0;
  
  for(int i = 0; i < NUM_KNN_NEIGHBORS; i++)
  {
    resultSet.dists[i] = 9999999.9f;
    resultSet.cosDists[i] = 9999999.9f;
  }
  
  resultSet.maxDist = resultSet.dists[NUM_KNN_NEIGHBORS - 1];
  
  for (int x = 0; x <= 1; x++)
  {
    for (int y = 0; y <= 1; y++)
    {
      for (int z = 0; z <= 1; z++)
      {
        HashData hd = getHash(intBaseCoords + int3(x, y, z), ihf.gridDim, ihf.hashToBucketSize, 0);
        
        int pointCellIndex = getPointCellIndex(hd.hashBase, hd.rawCellId, ihf);  
        
        if (pointCellIndex < 0)
          continue;
        
        uint numPoints = pointCellIndex >> COMPACT_HASH_INDEX_BITS;
        pointCellIndex &= COMPACT_HASH_INDEX_MASK;
        
        // Iterate over point cell and get all contributions        
        for (uint pointOffset = 0; pointOffset < numPoints; pointOffset++)
        {
          int otherIndex = pointCells[pointCellIndex + ihf.pointCellOffset + pointOffset];
          
          float otherValue = serverAOPoints[otherIndex].value;
          
          float l2 = 0.0f;
          float cosDist = cosineHemisphereDist(sp, serverAOPoints[otherIndex], l2);
          
          if(cosDist < resultSet.cosDists[NUM_KNN_NEIGHBORS - 1] && l2 < sp.searchRadius)
          {
            addPointCos(l2, cosDist, otherIndex, resultSet);
            resultSet.maxDist = resultSet.dists[NUM_KNN_NEIGHBORS - 1];
          }
          
        }
      }
    }
  }
  
  for(int i = 0; i < resultSet.count; i++)
  {
    uint offsetIndex = resultSet.indices[i];
    ClientPointData cpd;
    cpd.position = serverAOPoints[offsetIndex].position;
    cpd.value = serverAOPoints[offsetIndex].value;
    cpd.normal = serverAOPoints[offsetIndex].normal;
    
    applySamplingWeight(samplingWeight, sp, cpd, cosDeltaThreshold, cosNormalThreshold);
  }
  
  if(samplingWeight.weightSum == 0.0f)
    return clamp(samplingWeight.fallbackValueSum / samplingWeight.fallbackWeightSum, 0.0f, 1.0f);
  
  return clamp(samplingWeight.valueSum / samplingWeight.weightSum, 0.0f, 1.0f);
}

float samplePointAOHash(VSOut vsOut, uint triangleIndex, ShadingData sd, float3 barycentrics)
{
  AOShadingPoint sp = getInstanceShadingPointPreviousFrame(sd, vsOut, interpolationRadiusFactor, barycentrics, triangleIndex);
  
  InstanceHashInfo ihf = instanceHashInfo[vsOut.instanceID.index];
  
  float3 floatCoords = ((sp.posW - ihf.aabbMin)) / sp.diskRadius;
  int3 intBaseCoords = floatCoords - 0.5f;
  
  AOSamplingWeight samplingWeight = initSamplingWeight();
  
  for (int x = 0; x <= 1; x++)
  {
    for (int y = 0; y <= 1; y++)
    {
      for (int z = 0; z <= 1; z++)
      {
        // Get hash entry and raw ID for coord
        HashData hd = getHash(intBaseCoords + int3(x, y, z), ihf.gridDim, ihf.hashToBucketSize, 0);
        
        int pointCellIndex = getPointCellIndex(hd.hashBase, hd.rawCellId, ihf);  
        
        if (pointCellIndex < 0)
          continue;
        
        uint numPoints = pointCellIndex >> COMPACT_HASH_INDEX_BITS;
        pointCellIndex &= COMPACT_HASH_INDEX_MASK;
        
        // Iterate over point cell and get all contributions
        for (uint pointOffset = 0; pointOffset < numPoints; pointOffset++)
        {
          int offsetIndex = pointCells[pointCellIndex + ihf.pointCellOffset + pointOffset];
          
          if(serverAOPoints[offsetIndex].value < 0)
            continue;
          
          ClientPointData cpd;
          cpd.position = serverAOPoints[offsetIndex].position;
          cpd.value = serverAOPoints[offsetIndex].value;
          cpd.normal = serverAOPoints[offsetIndex].normal;
    
          applySamplingWeight(samplingWeight, sp, cpd, cosDeltaThreshold, cosNormalThreshold);
        }
      }
    }
  }
  
  if(samplingWeight.weightSum == 0.0f)
    return clamp(samplingWeight.fallbackValueSum / samplingWeight.fallbackWeightSum, 0.0f, 1.0f);
  
  return clamp(samplingWeight.valueSum / samplingWeight.weightSum, 0.0f, 1.0f);
}

float samplePointAOHashUpdate(VSOut vsOut, uint triangleIndex, ShadingData sd, float3 barycentrics)
{
  const uint shadingInstanceId = vsOut.instanceID.index;
  
  AOShadingPoint sp = getInstanceShadingPointPreviousFrame(sd, vsOut, interpolationRadiusFactor, barycentrics, triangleIndex);
  
  InstanceHashInfo ihf = serverInstanceHashInfo[shadingInstanceId];
  InstancePointInfo ipi = instancePointInfo[shadingInstanceId];
  
  float3 floatCoords = ((sp.posW - ipi.aabbMin)) / sp.diskRadius;
  int3 intBaseCoords = floatCoords - 0.5f;
  
  AOSamplingWeight samplingWeight = initSamplingWeight();
  
  for (int x = 0; x <= 1; x++)
  {
    for (int y = 0; y <= 1; y++)
    {
      for (int z = 0; z <= 1; z++)
      {
        // Get hash entry and raw ID for coord
        int3 coords = intBaseCoords + int3(x, y, z);
        
        HashData hd = getHash(intBaseCoords + int3(x, y, z), ipi.gridDim, ihf.hashToBucketSize, 0);
        
        uint hashBase = hd.hashBase * FIXED_HASH_BUCKET_SIZE;
                
        uint numPoints = 0;
        int pointCellIndex = getPointCellIndexUpdate(hashBase, hd.hashBase, hd.rawCellId, ihf.hashToBucketOffset, numPoints);  
        

        if (pointCellIndex < 0)
          continue;
        
        // Iterate over point cell and get all contributions
        uint pointOffset = 0;
        for (uint32_t pointOffset = 0; pointOffset < FIXED_POINTS_PER_CELL; pointOffset++)
        {
          int offsetIndex = pointCellIndex + ipi.pointCellOffset + pointOffset;
          
          if(!isPointValid(compressedClientAOPoints[offsetIndex].posNormVal))
            continue;
          
          ClientPointData cpd = decompressClientData(compressedClientAOPoints[offsetIndex], coords, sp.diskRadius, ipi.aabbMin);
          
          applySamplingWeight(samplingWeight, sp, cpd, cosDeltaThreshold, cosNormalThreshold);
        }
      }
    }
  }
  
  if(samplingWeight.weightSum == 0.0f)
    return clamp(samplingWeight.fallbackValueSum / samplingWeight.fallbackWeightSum, 0.0f, 1.0f);
  
  return clamp(samplingWeight.valueSum / samplingWeight.weightSum, 0.0f, 1.0f);
}


float samplePointAOKDTree(VSOut vsOut, uint triangleIndex, ShadingData sd, float3 barycentrics)
{
  const uint shadingInstanceId = vsOut.instanceID.index;
  
  ResultSet resultSet;
  
  AOShadingPoint sp = getInstanceShadingPointPreviousFrame(sd, vsOut, interpolationRadiusFactor, barycentrics, triangleIndex);
  
  findNeighborsCustomLinearTree(sp, shadingInstanceId, resultSet, 99999999.9f);
  
  AOSamplingWeight samplingWeight = initSamplingWeight();
  
  for(int i = 0; i < resultSet.count; i++)
  {
    uint offsetIndex = resultSet.indices[i];
    
    ClientPointData cpd;
    cpd.position = serverAOPoints[offsetIndex].position;
    cpd.value = serverAOPoints[offsetIndex].value;
    cpd.normal = serverAOPoints[offsetIndex].normal;
    
    applySamplingWeight(samplingWeight, sp, cpd, cosDeltaThreshold, cosNormalThreshold);
  }
  
  if(samplingWeight.weightSum == 0.0f)
    return clamp(samplingWeight.fallbackValueSum / samplingWeight.fallbackWeightSum, 0.0f, 1.0f);
  
  return clamp(samplingWeight.valueSum / samplingWeight.weightSum, 0.0f, 1.0f);
}

float sampleTexAO(VSOut vsOut)
{
  uint2 texSize;
	
  aoTex.GetDimensions(texSize.x, texSize.y);
	
  float2 normalizedCoords = vsOut.posH.xy;
  normalizedCoords /= texSize;
	
  if(aoType == AO_TYPE_PER_PIXEL_RTAO)
    return aoTex.SampleLevel(mainSampler, normalizedCoords, 0).r;
  else
    return ssaoTex.SampleLevel(mainSampler, normalizedCoords, 0).r;
}

float4 main(VSOut vsOut, uint triangleIndex : SV_PrimitiveID, float3 barycentrics : SV_Barycentrics) : SV_TARGET
{
  let lod = ImplicitLodTextureSampler();
  if (alphaTest(vsOut, triangleIndex, lod))
    discard;

  float3 viewDir = normalize(gScene.camera.getPosition() - vsOut.posW);
  
  ShadingData sd = prepareShadingData(vsOut, triangleIndex, viewDir, lod);

  // Create BSDF instance.
  let bsdf = gScene.materials.getBSDF(sd, lod);

  float3 color = bsdf.getProperties(sd).emission;

  // Direct lighting from analytic light sources
  for (int i = 0; i < gScene.getLightCount(); i++)
  {
    AnalyticLightSample ls;
    evalLightApproximate(sd.posW, gScene.getLight(i), ls);
    color += bsdf.eval(sd, ls.dir) * ls.Li;
  }
  
#ifdef _USE_ENV_MAP
    // Add lighting from environment map
    color.rgb += envMapLighting.evalDiffuse(sd, bsdf.getProperties(sd));
    color.rgb += envMapLighting.evalSpecular(sd, bsdf.getProperties(sd), reflect(-sd.V, sd.N));
#endif
  
    
  float aoFactor = 0.0f;
  if (aoType == AO_TYPE_PER_PIXEL_RTAO)
  {
    aoFactor = sampleTexAO(vsOut);
  }
  else if(aoType == AO_TYPE_POINT_AO_KD_TREE)
  {
    aoFactor = samplePointAOKDTree(vsOut, triangleIndex, sd, barycentrics); 
  }
  else if (aoType == AO_TYPE_POINT_AO_HASH)
  {
    aoFactor = samplePointAOHash(vsOut, triangleIndex, sd, barycentrics); 
  }
  else if (aoType == AO_TYPE_POINT_AO_HASH_KNN)
  {
    aoFactor = samplePointAOHashNN(vsOut, triangleIndex, sd, barycentrics); 
  }
  else if (aoType == AO_TYPE_POINT_AO_HASH_UPDATE)
  {
    aoFactor = samplePointAOHashUpdate(vsOut, triangleIndex, sd, barycentrics); 
  }
  else if (aoType == AO_TYPE_SSAO)
  {
    aoFactor = sampleTexAO(vsOut);
  }
	
  if (aoOnly)
  {
    return float4(aoFactor, aoFactor, aoFactor, 1.0f);
  }
  else if (colorOnly)
  {
    return float4(color, 1.0f);
  }
  else
  {
    return float4(color * aoFactor, 1.0f);
  }
}