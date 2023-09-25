/*
 * Copyright (c) Meta Platforms, Inc. and affiliates.
 * All rights reserved.
 *
 * This source code is licensed under the license found in the
 * LICENSE file in the root directory of this source tree.
 */

#include "PointAOConstantsShared.slangh"
import PointData;
import HashFunctionShared;

cbuffer perFrameConstantBuffer
{
  uint numUpdates;
}

[numthreads(32, 1, 1)]
void main(
    uint3 groupId : SV_GroupID,
    uint3 groupThreadId : SV_GroupThreadID,
    uint3 dispatchThreadId : SV_DispatchThreadID,
    uint groupIndex : SV_GroupIndex)
{
  if(dispatchThreadId.x >= numUpdates)
    return;
  
  HashUpdateInfo hui = hashUpdateInfos[dispatchThreadId.x];
   
  // For each hash bucket, we copy the data to the hash structure
  for(uint hashOffset = 0; hashOffset < FIXED_HASH_BUCKET_SIZE; hashOffset++)
  {
    serverHashToPointCell[hashOffset + hui.globalHashOffset] = hui.hashData[hashOffset]; 
  }
  
}
