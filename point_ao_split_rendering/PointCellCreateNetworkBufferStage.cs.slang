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
  uint globalCellOffset = cellDirtyInfos[dispatchThreadId.x];
  
  cellUpdateInfos[dispatchThreadId.x].globalCellOffset = globalCellOffset;
  cellUpdateDeltaInfos[dispatchThreadId.x].globalCellOffset = globalCellOffset;
  
  uint validPointCount = 0;
  
  uint validCellIds[FIXED_POINTS_PER_CELL];
  
  for (uint pointOffset = 0; pointOffset < FIXED_POINTS_PER_CELL; pointOffset++)
  {
    if(serverAOValues[globalCellOffset + pointOffset] >= 0)
    {
      validCellIds[validPointCount] = globalCellOffset + pointOffset;
      validPointCount++;
    }
  }
  
  // Reorder points so that the valid points are in front

  for(uint pointOffset = 0; pointOffset < validPointCount; pointOffset++)
  {
    cellUpdateInfos[dispatchThreadId.x].cellData[pointOffset] = compressedClientAOPoints[validCellIds[pointOffset]];
    compressedClientAOPoints[pointOffset + globalCellOffset] = compressedClientAOPoints[validCellIds[pointOffset]];
    //serverAOPoints[pointOffset + globalCellOffset] = serverAOPoints[validCellIds[pointOffset]];
    uint idx = pointOffset + globalCellOffset;
    uint idx2 = validCellIds[pointOffset];
    serverAOPositions[idx] = serverAOPositions[idx2];
    serverAONormals[idx] = serverAONormals[idx2];
    serverAOTangents[idx] = serverAOTangents[idx2];
    serverAOBarycentrics[idx] = serverAOBarycentrics[idx2];
    serverAOInstanceTriangleIDs[idx] = serverAOInstanceTriangleIDs[idx2];
    serverAOInstanceIDs[idx] = serverAOInstanceIDs[idx2];
    serverAOValues[idx] = serverAOValues[idx2];
  }
  
  // Invalidate all the other slots
  for(uint pointOffset = validPointCount; pointOffset < FIXED_POINTS_PER_CELL; pointOffset++)
  {
    cellUpdateInfos[dispatchThreadId.x].cellData[pointOffset].posNormVal = INVALID_CELL;
    compressedClientAOPoints[globalCellOffset + pointOffset].posNormVal = INVALID_CELL;
    //serverAOPoints[globalCellOffset + pointOffset].value = -1.0f;
    serverAOValues[globalCellOffset + pointOffset] = -1.0f;
  }
  
  // Compute delta infos and store previous' values
  for(uint pointOffset = 0; pointOffset < FIXED_POINTS_PER_CELL; pointOffset++)
  {
    cellUpdateDeltaInfos[dispatchThreadId.x].cellData[pointOffset].posNormVal = compressedClientAOPoints[pointOffset + globalCellOffset].posNormVal ^ previousCompressedClientAOPoints[pointOffset + globalCellOffset].posNormVal;
    
    previousCompressedClientAOPoints[pointOffset + globalCellOffset].posNormVal = compressedClientAOPoints[pointOffset + globalCellOffset].posNormVal;
  }
  
  
  // Make sure to reset the cell dirty flag
  cellDirtyFlags[globalCellOffset / FIXED_POINTS_PER_CELL] = CELL_NOT_DIRTY;
}
