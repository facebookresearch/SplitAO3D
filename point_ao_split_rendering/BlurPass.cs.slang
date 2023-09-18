/*
 * Copyright (c) Meta Platforms, Inc. and affiliates.
 * All rights reserved.
 *
 * This source code is licensed under the license found in the
 * LICENSE file in the root directory of this source tree.
 */

RWTexture2D<float> inTex;
RWTexture2D<float> outTex;

cbuffer perFrameConstantBuffer
{
	int kernelSize;
}

// This pass is used as a simple post-process blur for the per-pixel RTAO pass (RTAO.rt.slang)
// It is also used as the blur pass for the SSAO implementation.
[numthreads(16, 16, 1)]
void main(
    uint3 groupId : SV_GroupID,
    uint3 groupThreadId : SV_GroupThreadID,
    uint3 dispatchThreadId : SV_DispatchThreadID,
    uint groupIndex : SV_GroupIndex)
{
	
	float sum = 0.0f;
  uint2 texSize;
	
  inTex.GetDimensions(texSize.x, texSize.y);
	uint numSamples = 0;
	
	for (int x = -kernelSize / 2; x < (kernelSize + 1) / 2; x++)
	{
		for (int y = -kernelSize / 2; y < (kernelSize + 1) / 2; y++)
		{
			float2 coord = dispatchThreadId.xy + float2(x, y);
			
			if(coord.x < 0 || coord.y < 0 || coord.x >= texSize.x || coord.y >= texSize.y)
				continue;
			
			sum += inTex.Load(dispatchThreadId.xy + float2(x, y));
			numSamples++;
		}
	}
	
	outTex[dispatchThreadId.xy] = sum / (numSamples);
}
