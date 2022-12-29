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
  PointUpdateData pud = pointUpdateData[dispatchThreadId.x];
  PointData newPointData = pud.newServerPointData;
  HashData newHashData = pud.newHashData;
  InstanceHashInfo ihi = serverInstanceHashInfo[newPointData.instanceId]; 
  InstancePointInfo ipi = instancePointInfo[newPointData.instanceId];
  float diskRadius = DISK_RADIUS_FACTOR * instanceToDiskRadius[newPointData.instanceId];
  
  uint baseHashOffset = newHashData.hashBase + ihi.hashToBucketOffset;
  
  int hashIndex = -1;
  for (uint hashBucketOffset = 0; hashBucketOffset < FIXED_HASH_BUCKET_SIZE; hashBucketOffset++) {
          
    CompactHashToCellInfo htci = serverHashToPointCell[baseHashOffset + hashBucketOffset];

    if (htci.rawCellId == newHashData.rawCellId) {
      hashIndex = hashBucketOffset;
    }
  }
  
  if (hashIndex == -1)
  {
    //TODO/HACK: No space left in hash table. Ignore point.
    return;
  }
  
  CompactHashToCellInfo htci = serverHashToPointCell[baseHashOffset + hashIndex];
  
  // Find the first point (using atomics on the valid/invalid flags) to insert our new point in
  uint pci = decompressPointCellIndex(htci.encodedIndex);
        
  uint pointCellOffset = 0;
        
  for(pointCellOffset = 0; pointCellOffset < FIXED_POINTS_PER_CELL; pointCellOffset++)
  {
    // Do an AtomicAnd to clear the point data position validity flag
    // if the previous value was invalid, this allows us to atomically query the first free slot
    // This is essentially the same thing we do for the cell allocation in PointRTAO.rt.slang
    uint oldPosition = 0;
    InterlockedAnd(compressedClientAOPoints[pci + pointCellOffset + ipi.pointCellOffset].posNormVal, ~INVALID_CELL, oldPosition);
          
    if (!isPointValid(oldPosition))
    {
            
      // If the old position was invalid, we can use this point as the new location
      // This does not need atomics - we already guaranteed that this slot is unique for this thread.
      compressedClientAOPoints[pci + pointCellOffset + ipi.pointCellOffset] = compressClientData(newPointData, diskRadius, ipi.aabbMin);
      serverAOPoints[pci + pointCellOffset + ipi.pointCellOffset] = newPointData;
      
      break;
    }
          
    // If the old point was already valid, we need to skip to the next slot. We ignore cases where we don't find space in the cell.
  }
  
}
