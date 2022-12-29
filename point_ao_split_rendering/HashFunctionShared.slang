#pragma once
#include "Utils/HostDeviceShared.slangh"
// This file is designed to be included on the CPU/HOST side and imported on the GPU/DEVICE side
BEGIN_NAMESPACE_FALCOR

struct HashData
{
  uint rawCellId;
  uint hashBase;
};

inline HashData getHash(int3 coords, uint3 gridDim, uint hashTableSize, int hash_type)
{
  HashData hd;
  hd.rawCellId = ((coords.x + coords.y * gridDim.x * 17 + coords.z * gridDim.x * gridDim.y * 31));
  
  // This method of doing modulo only works for hash tables with power of two sizes
  hd.hashBase = (hd.rawCellId & (hashTableSize - 1));
  
  return hd;
}

END_NAMESPACE_FALCOR

