/*
 * Copyright (c) Meta Platforms, Inc. and affiliates.
 * All rights reserved.
 *
 * This source code is licensed under the license found in the
 * LICENSE file in the root directory of this source tree.
 */

#include "Utils/Math/MathConstants.slangh"
#include "PointAOConstantsShared.slangh"
import Scene.Shading;
import Scene.Raster;
import PointData;
import Utils.Math.MatrixUtils;

struct AOShadingPoint
{
  float3 posW;
  float3 N;
  float3 vertN;
  float diskRadius;
  float searchRadius;
};

struct AOSamplingWeight
{
  float weightSum;
  float valueSum;
  float fallbackWeightSum;
  float fallbackValueSum;
};

AOShadingPoint getInstanceShadingPoint(ShadingData sd, VSOut vsOut, float interpolationRadiusFactor)
{
  GeometryInstanceID shadingInstanceID = {vsOut.instanceID.index};
  const GeometryInstanceData shadingInstance = gScene.getGeometryInstance(shadingInstanceID);
  float4x4 shadingWorldMat = gScene.loadWorldMatrix(shadingInstance.globalMatrixID);
  float3x3 worldInvTransposeMat = gScene.loadInverseTransposeWorldMatrix(shadingInstance.globalMatrixID);

  // Convert ray origin and direction to local space.
  AOShadingPoint sp;
  
  sp.posW = mul(sd.posW - extractTranslate(shadingWorldMat), worldInvTransposeMat);
  sp.N = mul(sd.N, worldInvTransposeMat);
  
  sp.vertN = mul(worldInvTransposeMat, normalize(vsOut.normalW));
  sp.diskRadius = DISK_RADIUS_FACTOR * instanceToDiskRadius[vsOut.instanceID.index];
  sp.searchRadius = interpolationRadiusFactor * 0.5 * (sp.diskRadius) * (sp.diskRadius);
  
  return sp;
}

AOShadingPoint getInstanceShadingPointPreviousFrame(ShadingData sd, VSOut vsOut, float interpolationRadiusFactor, float3 barycentrics, uint triangleIndex)
{
  GeometryInstanceID shadingInstanceID = {vsOut.instanceID.index};
  const GeometryInstanceData shadingInstance = gScene.getGeometryInstance(shadingInstanceID);
  float4x4 shadingWorldMat = gScene.loadWorldMatrix(shadingInstance.globalMatrixID);
  float3x3 worldInvTransposeMat = gScene.loadInverseTransposeWorldMatrix(shadingInstance.globalMatrixID);
  float4x4 prevWorldMat = gScene.loadPrevWorldMatrix(shadingInstance.globalMatrixID);
  float3x3 prevWorldInvTransposeMat = gScene.loadPrevInverseTransposeWorldMatrix(shadingInstance.globalMatrixID);
  
  float3 prevPos = gScene.getPrevPosW(shadingInstanceID, triangleIndex, barycentrics);
  
  // Transform prevPos and current pos to local space
  prevPos = mul(prevPos - extractTranslate(prevWorldMat), prevWorldInvTransposeMat);

  // Convert ray origin and direction to local space.
  AOShadingPoint sp;
  
  sp.posW = prevPos;
  sp.N = mul(sd.N, worldInvTransposeMat);
  sp.vertN = mul(normalize(vsOut.normalW), worldInvTransposeMat);
  sp.diskRadius = DISK_RADIUS_FACTOR * instanceToDiskRadius[vsOut.instanceID.index];
  sp.searchRadius = interpolationRadiusFactor * 0.5 * (sp.diskRadius) * (sp.diskRadius);
  
  return sp;
}

AOSamplingWeight initSamplingWeight()
{
  AOSamplingWeight weight;
  
  weight.fallbackWeightSum = 0;
  weight.fallbackValueSum = 0;
  weight.weightSum = 0;
  weight.valueSum = 0;
  
  return weight;
}

void applySamplingWeight(inout AOSamplingWeight samplingWeight, AOShadingPoint sp, ClientPointData cpd, float cosDeltaThreshold, float cosNormalThreshold)
{
  // Compute distance in local space
  const float3 distVec = cpd.position - sp.posW;
  const float dist = max(0.000001f, dot(distVec, distVec));
   
  float weight = max(0.0f, (sp.searchRadius - dist) / sp.searchRadius);
    
  const float3 vecToPoint = normalize(distVec);
  const float nDotDelta = dot(normalize(sp.vertN), vecToPoint);
  const float cosDist = nDotDelta * nDotDelta;  
  const float cosN = dot(sp.N, cpd.normal);
           
  const float fallbackWeight = max((1.0f - cosDist) + cosN, 0.0001f);
  samplingWeight.fallbackWeightSum += fallbackWeight;
  samplingWeight.fallbackValueSum += cpd.value * fallbackWeight;
          
  // TODO/HACK: this heuristic of weighting the cosine term seems to work well.
  if (cosDist > cosDeltaThreshold || cosN < cosNormalThreshold)
    return;
    
  samplingWeight.weightSum += weight;
  samplingWeight.valueSum += cpd.value * weight;
}


