/*
 * Copyright (c) Meta Platforms, Inc. and affiliates.
 * All rights reserved.
 *
 * This source code is licensed under the license found in the
 * LICENSE file in the root directory of this source tree.
 */

import PointData;
import HashFunctionShared;
#include "PointAOConstantsShared.slangh"

[numthreads(1, 1, 1)]
void main(
    uint3 groupId : SV_GroupID,
    uint3 groupThreadId : SV_GroupThreadID,
    uint3 dispatchThreadId : SV_DispatchThreadID,
    uint groupIndex : SV_GroupIndex)
{
  uint globalHashOffset = hashDirtyInfos[dispatchThreadId.x];
  
  hashUpdateInfos[dispatchThreadId.x].globalHashOffset = globalHashOffset;
  
  uint validHashCount = 0;
  
  uint validHashIds[FIXED_HASH_BUCKET_SIZE];
  
  for (uint hashOffset = 0; hashOffset < FIXED_HASH_BUCKET_SIZE; hashOffset++)
  {
    if(serverHashToPointCell[globalHashOffset + hashOffset].rawCellId != INVALID_CELL)
    {
      validHashIds[validHashCount] = globalHashOffset + hashOffset;
      validHashCount++;
    }
  }
  
  // Reorder hash buckets so that the valid points are in front
  // Currently, this should do nothing as we don't delete hash buckets
  // once they have been allocated. 
  // Could be useful in the future though.
  for(uint hashOffset = 0; hashOffset < validHashCount; hashOffset++)
  {
    hashUpdateInfos[dispatchThreadId.x].hashData[hashOffset] = serverHashToPointCell[validHashIds[hashOffset]];
  }
  
  // Invalidate all the other slots
  for(uint hashOffset = validHashCount; hashOffset < FIXED_HASH_BUCKET_SIZE; hashOffset++)
  {
    hashUpdateInfos[dispatchThreadId.x].hashData[hashOffset].rawCellId = INVALID_CELL;
    hashUpdateInfos[dispatchThreadId.x].hashData[hashOffset].encodedIndex = INVALID_CELL;
  }
}
