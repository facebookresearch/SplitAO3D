// (c) Meta Platforms, Inc. and its affiliates
import Scene.Shading;
import Scene.Raster;
import Utils.Sampling.TinyUniformSampleGenerator;
import Rendering.Lights.LightHelpers;

SamplerState sampler;

Texture2D aoTex;
Texture2D ssaoTex;
Texture2D normalTex;
Texture2D depthTex;

cbuffer perFrameConstantBuffer
{
  bool aoOnly;
  bool ssaoOnly;
  bool colorOnly;
  bool colorPlusSSAO;
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
		
  uint2 texSize;
	
  normalTex.GetDimensions(texSize.x, texSize.y);
	
  float2 normalizedCoords = vsOut.posH.xy; // vOut.posH.w;
  normalizedCoords /= texSize;
		
	
  float aoFactor = aoTex.SampleLevel(sampler, normalizedCoords, 0).r;
  float ssaoVal = ssaoTex.SampleLevel(sampler, normalizedCoords, 0).r;
  float3 normals = normalTex.SampleLevel(sampler, normalizedCoords, 0).rgb;
	
	
  if (aoOnly)
  {
    return float4(aoFactor, aoFactor, aoFactor, 1.0f);
  }
  else if (ssaoOnly)
  {
    return float4(ssaoVal, ssaoVal, ssaoVal, 1.0f);
  }
  else if (colorOnly)
  {
    return float4(color.rgb, 1.0f);
  }
  else if (colorPlusSSAO)
  {
    return float4(color.rgb * ssaoVal, 1.0f);
  }
  else
  {
    return float4(color.rgb * aoFactor, 1.0f);
  }
}
