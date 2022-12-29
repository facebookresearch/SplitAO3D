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
  
  CellUpdateInfo cui = cellUpdateInfos[dispatchThreadId.x];
   
  // For each cell, we copy the data to the compressed point cell structure
  for(uint pointOffset = 0; pointOffset < FIXED_POINTS_PER_CELL; pointOffset++)
  {
    compressedClientAOPoints[pointOffset + cui.globalCellOffset].posNormVal = compressedClientAOPoints[pointOffset + cui.globalCellOffset].posNormVal ^ cui.cellData[pointOffset].posNormVal; 
  }
  
}
