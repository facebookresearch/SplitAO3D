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
import PointAOSamplingUtils;

cbuffer perFrameConstantBuffer
{
  bool aoOnly;
  bool colorOnly;
  bool depthPrepass;
  int aoType;
  float aoRadius;
  float interpolationRadiusFactor;
  float cosNormalThreshold;
  float cosDeltaThreshold;
}

EnvMapLighting envMapLighting;

// Takes the base index in the hash table (and the offset for the given instance) and returns the point cell index and number of points
// It looks for any cell that matches the given rawCellId, which is computed from the hash function.
int getPointCellIndexUpdate(uint hashBase, uint hashBucketBase, uint rawCellId, uint hashToBucketOffset, out uint numPoints)
{
  uint baseHashOffset = hashBase + hashToBucketOffset;
  
  for (uint hashBucketOffset = 0; hashBucketOffset < FIXED_HASH_BUCKET_SIZE; hashBucketOffset++) {
      
      CompactHashToCellInfo htci = serverHashToPointCell[baseHashOffset + hashBucketOffset];
    
      if (htci.rawCellId == rawCellId) {
        return htci.encodedIndex;
      }
      else if (htci.rawCellId == INVALID_CELL)
      {
        return -1;
      }
        
    }
  
  return -1;
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

float4 main(VSOut vsOut, uint triangleIndex : SV_PrimitiveID, float3 barycentrics : SV_Barycentrics) : SV_TARGET
{
  let lod = ImplicitLodTextureSampler();
  if (alphaTest(vsOut, triangleIndex, lod))
    discard;
  
  if(depthPrepass)
    return (1.0f, 0.0f, 1.0f, 1.0f);

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
  
  float aoFactor = samplePointAOHashUpdate(vsOut, triangleIndex, sd, barycentrics); 
	
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