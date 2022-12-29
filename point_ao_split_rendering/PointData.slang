#pragma once
#include "Utils/HostDeviceShared.slangh"
#include "PointAOConstantsShared.slangh"
#include "OctEncoding.slang"

#ifndef HOST_CODE
//import OctEncoding;
import HashFunctionShared;
#else
#include "HashFunctionShared.slang"
#endif


BEGIN_NAMESPACE_FALCOR

/** This is a host/device structure that describes a point for point-based visualization and storage.
*/

struct PointData
{
  float3 position;
  float3 normal;
  float3 tangent;
  float2 barycentrics;
  uint instanceTriangleId;
  uint instanceId;
  float value;
  
#ifdef HOST_CODE
  PointData() : value(-1.0f) {};
#endif
};

struct CompressedClientPointData
{
  // encoded point data - constants are defined in PointAOConstantsShared.slangh
  // 1 bit valid / invalid
  // 15 bits position (5 bits x/y/z, quantized to grid cells)
  // 8 bits normal (oct encoded)
  // 8 bits value (quantized)
  uint posNormVal;
#ifdef HOST_CODE
  CompressedClientPointData() : posNormVal(INVALID_CELL) {};
#endif
};

struct PointUpdateData
{
  PointData newServerPointData;
  HashData newHashData;
};

#ifndef HOST_CODE

struct ClientPointData
{
  float3 position;
  float3 normal;
  float value;
  
#ifdef HOST_CODE
  ClientPointData() : position({999999, 999999, 999999}), normal({0, 1, 0}), value(1.0) {};
#endif
};

bool isPointValid(uint compressedPosition)
{
  return (compressedPosition & INVALID_CELL) == 0;
}

ClientPointData decompressClientData(CompressedClientPointData ccpd, uint3 gridIndex, float gridCellSize, float3 aabbMin)
{
  ClientPointData cpd;
  
  float3 offset = aabbMin + gridCellSize * gridIndex;
  
  cpd.position = offset + float3(uint3(ccpd.posNormVal, ccpd.posNormVal >> POINT_POS_BITS, ccpd.posNormVal >> (2 * POINT_POS_BITS)) & POINT_POS_MAX) * ONE_OVER_POINT_POS_MAX * gridCellSize;
  cpd.normal = octahedral8Inverse((ccpd.posNormVal >> (3 * POINT_POS_BITS)) & NORMAL_MAX);
  cpd.value = ((ccpd.posNormVal >> (3 * POINT_POS_BITS + NORMAL_BITS)) & POINT_VAL_MAX) * ONE_OVER_POINT_VAL_MAX;
  
  return cpd;
}

#endif

inline CompressedClientPointData compressClientData(PointData cpd, float gridCellSize, float3 aabbMin)
{
  CompressedClientPointData ccpd;
  
  float3 localCellPosition = (((cpd.position - (aabbMin))) / gridCellSize);
  
#ifdef HOST_CODE
  float3 remainderVec = localCellPosition - float3((int)localCellPosition.x, (int)localCellPosition.y, (int)localCellPosition.z);
  uint3 localCellUint = uint3(remainderVec.x * POINT_POS_MAX, remainderVec.y * POINT_POS_MAX, remainderVec.z * POINT_POS_MAX);
#else
  uint3 localCellUint = uint3(round((localCellPosition - int3(localCellPosition)) * POINT_POS_MAX));
#endif
  ccpd.posNormVal = localCellUint.x | (localCellUint.y << (POINT_POS_BITS)) | (localCellUint.z << (2 * POINT_POS_BITS));
  ccpd.posNormVal |= (octahedral8(cpd.normal) & NORMAL_MAX) << (POINT_POS_BITS * 3);
  ccpd.posNormVal |= (uint(cpd.value * POINT_VAL_MAX) & POINT_VAL_MAX) << (NORMAL_BITS + POINT_POS_BITS * 3);
 
  return ccpd;
}


struct HashBucketInfo
{
  int pointCellIndex;
  uint rawCellId;
};

struct HashToCellInfo
{
  uint rawCellId;
  int pointCellIndex; // uses -1 to signal used or unused
  uint numPoints;
  uint numCells;
  
#ifdef HOST_CODE
  HashToCellInfo() : rawCellId(0), pointCellIndex(-1), numPoints(0), numCells(0)  {};
#endif
};


struct CompactHashToCellInfo
{
  uint rawCellId;
  uint encodedIndex; // uses -1 to signal used or unused
  
#ifdef HOST_CODE
  CompactHashToCellInfo() : encodedIndex(INVALID_CELL), rawCellId(INVALID_CELL) {};
#endif
};

struct HashNumBuckets
{
  // This is a full int because we need to subtract with atomics -> TODO if we find a better / more compact way.
  int numBuckets; 
};

#ifdef HOST_CODE

#else
uint decompressNumCells(uint encodedIndex)
{
  return (encodedIndex >> 27) & 0x0F;
}

uint decompressNumPoints(uint encodedIndex)
{
  return (encodedIndex >> 22) & 0x1F;
}

uint decompressPointCellIndex(uint encodedIndex)
{
  return (encodedIndex & COMPACT_HASH_INDEX_MASK);
}

uint compressHashToCellInfoIndex(uint pointCellIndex, uint numPoints, uint numCells) {
  return (pointCellIndex & 0x003FFFFF) | ((numPoints & 0x1F) << 22) | ((numCells & 0x0F) << 27);
}
#endif
struct FixedHashBucketInfo
{
  HashToCellInfo hashInfos[FIXED_HASH_BUCKET_SIZE];
  uint numCells;
};

struct KDTreeGPUNode
{
  // Use asuint() to get the correct data for leaf nodes
  float leftOrDivLow;
  float rightOrDivHigh;
  uint divfeat;
  int parentId;
  int child1;
  int child2;
};

struct PointsAABB
{
  float3 max;
  float3 min;
};


struct InstanceHashInfo {
  uint hashToBucketOffset;
  uint hashToBucketSize; // Required for modulo
  uint hashBucketToPointCellOffset;
  uint pointCellOffset;
  float3 aabbMin;
  float3 aabbMax;
  uint3 gridDim;
};

struct InstancePointInfo
{
  uint pointCellOffset;
  uint numAllocatedCells;
  uint maxNumPoints;
  float3 aabbMin;
  float3 aabbMax;
  uint3 gridDim;
};

struct PerFrameUpdateInfo
{
  uint numChangedPoints;
  uint numAllocatedCells;
};

struct IndirectDispatchArgs
{
  uint x;
  uint y;
  uint z;
};

struct CellUpdateInfo
{
  uint globalCellOffset;
  CompressedClientPointData cellData[FIXED_POINTS_PER_CELL];
};

struct HashUpdateInfo
{
  uint globalHashOffset;
  CompactHashToCellInfo hashData[FIXED_HASH_BUCKET_SIZE];
};

#ifndef HOST_CODE
RWStructuredBuffer<PointData> serverAOPoints;
RWStructuredBuffer<CompressedClientPointData> compressedClientAOPoints;
RWStructuredBuffer<CompressedClientPointData> previousCompressedClientAOPoints;
RWStructuredBuffer<PerFrameUpdateInfo> frameUpdateInfo;
RWStructuredBuffer<int> hashToBucket;
RWStructuredBuffer<HashBucketInfo> hashBucketToPointCell;
RWStructuredBuffer<HashNumBuckets> hashNumBuckets;
RWStructuredBuffer<uint> pointCells;
RWStructuredBuffer<uint> cellDirtyFlags;
RWStructuredBuffer<uint> cellDirtyInfos;
RWStructuredBuffer<uint> hashDirtyInfos;
RWStructuredBuffer<CellUpdateInfo> cellUpdateInfos;
RWStructuredBuffer<CellUpdateInfo> cellUpdateDeltaInfos;
RWStructuredBuffer<HashUpdateInfo> hashUpdateInfos;
RWStructuredBuffer<InstanceHashInfo> instanceHashInfo;
RWStructuredBuffer<InstancePointInfo> instancePointInfo;
RWStructuredBuffer<InstanceHashInfo> serverInstanceHashInfo;
RWStructuredBuffer<CompactHashToCellInfo> serverHashToPointCell;
RWStructuredBuffer<float> instanceToDiskRadius;
RWStructuredBuffer<PointUpdateData> pointUpdateData;
RWStructuredBuffer<IndirectDispatchArgs> cellAllocIndirectDispatchArgs;
RWStructuredBuffer<IndirectDispatchArgs> cellNetworkBufferIndirectDispatchArgs;
RWStructuredBuffer<IndirectDispatchArgs> hashNetworkBufferIndirectDispatchArgs;
#endif

END_NAMESPACE_FALCOR
