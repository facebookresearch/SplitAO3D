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
    if(serverAOPoints[globalCellOffset + pointOffset].value >= 0)
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
    serverAOPoints[pointOffset + globalCellOffset] = serverAOPoints[validCellIds[pointOffset]];
  }
  
  // Invalidate all the other slots
  for(uint pointOffset = validPointCount; pointOffset < FIXED_POINTS_PER_CELL; pointOffset++)
  {
    cellUpdateInfos[dispatchThreadId.x].cellData[pointOffset].posNormVal = INVALID_CELL;
    compressedClientAOPoints[globalCellOffset + pointOffset].posNormVal = INVALID_CELL;
    serverAOPoints[globalCellOffset + pointOffset].value = -1.0f;
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
