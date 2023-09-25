/*
 * Copyright (c) Meta Platforms, Inc. and affiliates.
 * All rights reserved.
 *
 * This source code is licensed under the license found in the
 * LICENSE file in the root directory of this source tree.
 */

// Most of this code is a direct port of the nanoflann CPU kNN kd-Tree, and is thus not really optimal for GPU performance
// It was mostly used early on to debug / test the image quality of the hash-based AO methods.
#include "PointAOConstantsShared.slangh"
import PointData;
import Scene.Shading;
import PointAOSamplingUtils;

StructuredBuffer<KDTreeGPUNode> linearTree;
Buffer<uint32_t> kdTreeIndex;
Buffer<uint> instanceToKdTree;
Buffer<uint> instanceToPointOffset;

cbuffer kdTreeConstantBuffer
{
  int numNeighbors;
}

float l2AxisDist(float a, float b)
{
  return (a - b) * (a - b);
}

float l2Dist(float3 a, float3 b)
{
  float3 diff = a - b;
  return dot(diff, diff);
}
 
float cosineHemisphereDist(AOShadingPoint vec, PointData pt, out float l2)
{
  float3 vecToPoint = normalize(pt.position - vec.posW);
  float nDotDelta = dot(normalize(vec.N), normalize(vecToPoint));
  
  l2 = l2Dist(vec.posW, pt.position);
  
  const float cosDist = nDotDelta * nDotDelta;  
  
  const float k1 = 1.0f;
  const float k2 = 0.0f;
  
  
  return k1 * l2 + k2 * cosDist;
}

void addPoint(float dist, uint index, inout ResultSet resultSet)
{
  uint i = 0;
  for (i = resultSet.count; i > 0; --i)
  {
    if (resultSet.dists[i - 1] > dist)
    {
      if (i < numNeighbors)
      {
        resultSet.dists[i] = resultSet.dists[i - 1];
        resultSet.indices[i] = resultSet.indices[i - 1];
      }
    }
    else
      break;
  }
  if (i < numNeighbors)
  {
    resultSet.dists[i] = dist;
    resultSet.indices[i] = index;
  }
  
  if (resultSet.count < numNeighbors)
    resultSet.count++;
}


void addPointCos(float dist, float cosDist, uint index, inout ResultSet resultSet)
{
  uint i = 0;
  for (i = resultSet.count; i > 0; --i)
  {
    if (resultSet.cosDists[i - 1] > cosDist)
    {
      if (i < numNeighbors)
      {
        resultSet.dists[i] = resultSet.dists[i - 1];
        resultSet.cosDists[i] = resultSet.cosDists[i - 1];
        resultSet.indices[i] = resultSet.indices[i - 1];
      }
    }
    else
      break;
  }
  if (i < numNeighbors)
  {
    resultSet.dists[i] = dist;
    resultSet.cosDists[i] = cosDist;
    resultSet.indices[i] = index;
  }
  
  if (resultSet.count < numNeighbors)
    resultSet.count++;
}


void findNeighborsCustomLinearTree(AOShadingPoint vec, uint instanceId, inout ResultSet resultSet, float searchRadius)
{
  float3 dists = float3(0, 0, 0);
  float distSquared = 0.0f;

  bool backtrack = false;
  
  const int kdTreeOffset = instanceToKdTree[instanceId];
  const int kdTreeIndexOffset = instanceToPointOffset[instanceId];

  int lastNode = -1;
  int current = 0;
  
  resultSet.count = 0;
  
  for(int i = 0; i < NUM_KNN_NEIGHBORS; i++)
  {
    resultSet.dists[i] = 9999999.9f;
    resultSet.cosDists[i] = 9999999.9f;
  }
  
  resultSet.maxDist = resultSet.dists[numNeighbors - 1];
  
  while (true)
  {
    if (current == -1)
      break;

    int bestChild = -1;
    int otherChild = -1;
    uint idx;
    float val;
    float diff1;
    float diff2;
    float cutDist;
    
    if ((linearTree[kdTreeOffset + current].child1 != -1) || (linearTree[kdTreeOffset + current].child2 != -1))
    {
      // Check for best/other child if we're not at a leaf node
      idx = linearTree[kdTreeOffset + current].divfeat;
      val = vec.posW[idx];
      diff1 = val - linearTree[kdTreeOffset + current].leftOrDivLow;
      diff2 = val - linearTree[kdTreeOffset + current].rightOrDivHigh;

      if ((diff1 + diff2) < 0)
      {
        bestChild = linearTree[kdTreeOffset + current].child1;
        otherChild = linearTree[kdTreeOffset + current].child2;
        cutDist = l2AxisDist(val, linearTree[kdTreeOffset + current].rightOrDivHigh);
      }
      else
      {
        bestChild = linearTree[kdTreeOffset + current].child2;
        otherChild = linearTree[kdTreeOffset + current].child1;
        cutDist = l2AxisDist(val, linearTree[kdTreeOffset + current].leftOrDivLow);
      }
    }

    if (!backtrack)
    {
      if ((linearTree[kdTreeOffset + current].child1 == -1) && (linearTree[kdTreeOffset + current].child2 == -1))
      {
        // this is equivalent to result.insert()
        uint32_t left = asuint(linearTree[kdTreeOffset + current].leftOrDivLow);
        uint32_t right = asuint(linearTree[kdTreeOffset + current].rightOrDivHigh);

        for (uint32_t i = left; i < right; ++i)
        {
          float l2 = 0.0f;
          float cosDist = cosineHemisphereDist(vec, serverAOPoints[kdTreeIndex[kdTreeIndexOffset + i]], l2);
          
          if(cosDist < resultSet.cosDists[numNeighbors - 1] && l2 < searchRadius)
          {
            addPointCos(l2, cosDist, kdTreeIndex[kdTreeIndexOffset + i], resultSet);
            resultSet.maxDist = resultSet.dists[numNeighbors - 1];
          }
          
        }
        
        backtrack = true;
        lastNode = current;
        current = linearTree[kdTreeOffset + current].parentId;
      }
      else
      {
        lastNode = current;
        current = bestChild;
      }
    }
    else
    {
      // continue moving back up the tree or visit far node?
      // minimum possible distance between query point and a point
      // inside the AABB

      if ((diff1 + diff2) < 0)
      {
        bestChild = linearTree[kdTreeOffset + current].child1;
        otherChild = linearTree[kdTreeOffset + current].child2;
        cutDist = l2AxisDist(val, linearTree[kdTreeOffset + current].rightOrDivHigh);
      }
      else
      {
        bestChild = linearTree[kdTreeOffset + current].child2;
        otherChild = linearTree[kdTreeOffset + current].child1;
        cutDist = l2AxisDist(val, linearTree[kdTreeOffset + current].leftOrDivLow);
      }

      float mindistSquared = 0;
      float dst = dists[idx];
      mindistSquared = mindistSquared + cutDist - dst;
      dists[idx] = cutDist;

      //  the far node was NOT the last node (== not visited yet) AND
      //  there could be a closer point in it
      if ((lastNode == bestChild) && (mindistSquared <= resultSet.maxDist) && (mindistSquared <= searchRadius))
      {
        lastNode = current;
        current = otherChild;
        backtrack = false;
      }
      else
      {
        lastNode = current;
        current = linearTree[kdTreeOffset + current].parentId;
      }
    }
  }
  
}
