/*
 * Copyright (c) Meta Platforms, Inc. and affiliates.
 * All rights reserved.
 *
 * This source code is licensed under the license found in the
 * LICENSE file in the root directory of this source tree.
 */

import Scene.Shading;
import Scene.Raster;
import Utils.Sampling.TinyUniformSampleGenerator;
import Rendering.Lights.LightHelpers;
import Rendering.Lights.EnvMapLighting;


SamplerState mainSampler;
EnvMapLighting envMapLighting;

Texture2D aoTex;

cbuffer perFrameConstantBuffer
{
  bool aoOnly;
  bool colorOnly;
}

float4 main(VSOut vsOut, uint triangleIndex : SV_PrimitiveID) : SV_TARGET
{
  let lod = ImplicitLodTextureSampler();
  if (alphaTest(vsOut, triangleIndex, lod))
    discard;

  float3 viewDir = normalize(gScene.camera.getPosition() - vsOut.posW);
  ShadingData sd = prepareShadingData(vsOut, triangleIndex, viewDir, lod);

  // Create BSDF instance.
  let bsdf = gScene.materials.getBSDF(sd, lod);

  float3 color = bsdf.getProperties(sd).emission;

  // Direct lighting from analytic light sources
  for (int i = 0; i < gScene.getLightCount(); i++)
  {
    AnalyticLightSample ls;
    evalLightApproximate(sd.posW, gScene.getLight(i), ls);
    color += bsdf.eval(sd, ls.dir) * ls.Li;
  }
  
#ifdef _USE_ENV_MAP
    // Add lighting from environment map
    color.rgb += envMapLighting.evalDiffuse(sd, bsdf.getProperties(sd));
    color.rgb += envMapLighting.evalSpecular(sd, bsdf.getProperties(sd), reflect(-sd.V, sd.N));
#endif

		
  uint2 texSize;
	
  aoTex.GetDimensions(texSize.x, texSize.y);
	
  float2 normalizedCoords = vsOut.posH.xy; // vOut.posH.w;
  normalizedCoords /= texSize;
		
	
  float aoFactor = aoTex.SampleLevel(mainSampler, normalizedCoords, 0).r;
	
  if (aoOnly)
  {
    return float4(aoFactor, aoFactor, aoFactor, 1.0f);
  }
  else if (colorOnly)
  {
    return float4(color, 1.0f);
  }
  else
  {
    return float4(color * aoFactor, 1.0f);
  }
}
