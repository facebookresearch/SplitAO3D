/*
 * Copyright (c) Meta Platforms, Inc. and affiliates.
 * All rights reserved.
 *
 * This source code is licensed under the license found in the
 * LICENSE file in the root directory of this source tree.
 */

 
import Scene.Shading;
import Scene.Raster;

cbuffer perFrameConstantBuffer
{
  bool isSecondPass;
}

RWStructuredBuffer<uint> triangleVisibilityData;
RWStructuredBuffer<uint> triangleVisibilityDataPerFrame;
RWStructuredBuffer<uint> triangleVisibilityDataTest;
RWStructuredBuffer<uint> triangleVisibilityOffsetData;


VSOut vsMain(VSIn vIn)
{
  return defaultVS(vIn);
}

float4 main(VSOut vOut, uint triangleIndex : SV_PrimitiveID) : SV_TARGET
{
  let lod = ImplicitLodTextureSampler();
  if (alphaTest(vOut, triangleIndex, lod))
    discard;

  float3 viewDir = normalize(gScene.camera.getPosition() - vOut.posW);
  ShadingData sd = prepareShadingData(vOut, triangleIndex, viewDir, lod);

  //return float4(1.0f, 0.0f, 0.0f, 1.0f);
  if(!isSecondPass)
    return float4(sd.N * 0.5f + 0.5f, 1.0f);


  // TODO: write primitive ID into the corresponding instance ID buffer thing
  // First: support triangles without any indices I guess, otherwise we run into issues

  uint instance_id = vOut.instanceID.index;
  uint triangle_offset = triangleVisibilityOffsetData[instance_id];

  triangleVisibilityData[triangle_offset + triangleIndex] = 1;
  triangleVisibilityDataPerFrame[triangle_offset + triangleIndex] = 1;


  uint test_data = triangleVisibilityDataTest[triangle_offset + triangleIndex];

  return float4(test_data, 1.0f, 0.0f, 1.0f);
}
