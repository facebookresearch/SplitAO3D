import Scene.Raytracing;
import Utils.Sampling.TinyUniformSampleGenerator;
import Rendering.Lights.LightHelpers;
import Utils.Math.MathHelpers;

cbuffer perFrameConstantBuffer
{
  float4x4 invView;
  float2 viewportDims;
  float tanHalfFovY;
  uint sampleIndex;
  float aoRadius;
  int aoSamples;
};

struct PrimaryRayData
{
  float color;
};

struct AORayData
{
  float hitT;
};

float3 randomSampleCosineHemisphere(ShadingData sd, float2 rng)
{
  float pdf;
  float3 relativeVec = normalize(sample_cosine_hemisphere_concentric(rng, pdf));
	
  return normalize(sd.fromLocal(relativeVec));
}

[shader("miss")]
void aoMiss(inout AORayData hitData)
{
  hitData.hitT = aoRadius;
}

[shader("closesthit")]
void aoClosestHit(inout AORayData hitData, in BuiltInTriangleIntersectionAttributes attribs)
{
  hitData.hitT = RayTCurrent();
}

[shader("anyhit")]
void aoAnyHit(
    inout AORayData rayData,
    BuiltInTriangleIntersectionAttributes attribs)
{
  let lod = ExplicitLodTextureSampler(0.f);
  float3 rayDirW = WorldRayDirection();
  // Alpha test for non-opaque geometry.
  GeometryInstanceID instanceID = getGeometryInstanceID();
  VertexData v = getVertexData(instanceID, PrimitiveIndex(), attribs);
  uint materialID = gScene.getMaterialID(instanceID);
  ShadingData sd = gScene.materials.prepareShadingData(v, materialID, -rayDirW, lod);
  if (gScene.materials.alphaTest(v, materialID, 0.f))
  {
    IgnoreHit();
  }
}


[shader("miss")]
void primaryMiss(inout PrimaryRayData hitData)
{
  hitData.color = 0.0f;
}

// Note: uniforms passed to hit shaders and miss shaders are automatically converted to shader records when compiling to SPIR-V
[shader("closesthit")]
void primaryClosestHit(
    inout PrimaryRayData hitData,
    in BuiltInTriangleIntersectionAttributes attribs)
{
  // Get the hit-point data.
  float3 rayDirW = WorldRayDirection();
  float hitT = RayTCurrent();
  uint triangleIndex = PrimitiveIndex();
  let lod = ExplicitLodTextureSampler(0.f);
  
  // Prepare the shading data.
  const GeometryInstanceID instanceID = getGeometryInstanceID();
  VertexData v = getVertexData(instanceID, triangleIndex, attribs);
  uint materialID = gScene.getMaterialID(instanceID);
  ShadingData sd = gScene.materials.prepareShadingData(v, materialID, -rayDirW, lod);
	
  int numSamples = aoSamples;
  float sampleLength = aoRadius;
    
  uint3 launchIndex = DispatchRaysIndex();
	
  TinyUniformSampleGenerator sg = TinyUniformSampleGenerator(launchIndex.xy, 42);

  float aoTerm = 0.0f;
  uint numHitRays = 0;
  for (int i = 0; i < numSamples; i++)
  {
    float3 direction = randomSampleCosineHemisphere(sd, sampleNext2D(sg));
		
    // Cast a ray towards that
    RayDesc ray;
    ray.Origin = sd.computeNewRayOrigin();
    ray.Direction = normalize(direction);
    ray.TMin = 0.00;
    ray.TMax = sampleLength;

    AORayData rayData;
    rayData.hitT = -1.0f;
    
    TraceRay(gScene.rtAccel, 0x0 /*rayFlags*/, 0xFF, 1 /* ray index*/, rayTypeCount, 1, ray, rayData);
    
    if (rayData.hitT >= 0)
    {
      numHitRays++;
      aoTerm += rayData.hitT / sampleLength;
    }
  }

  aoTerm /= (float) numHitRays;
	
  hitData.color = aoTerm;
}

[shader("anyhit")]
void primaryAnyHit(
    inout PrimaryRayData rayData,
    BuiltInTriangleIntersectionAttributes attribs)
{
  // Alpha test for non-opaque geometry.
  GeometryInstanceID instanceID = getGeometryInstanceID();
  VertexData v = getVertexData(instanceID, PrimitiveIndex(), attribs);
  uint materialID = gScene.getMaterialID(instanceID);
  if (gScene.materials.alphaTest(v, materialID, 0.f))
  {
    IgnoreHit();
  }
}

uniform RWTexture2D<float> gOutput;

[shader("raygeneration")]
void rayGen()
{
  uint3 launchIndex = DispatchRaysIndex();

  RayDesc ray;

  ray = gScene.camera.computeRayPinhole(launchIndex.xy, viewportDims).toRayDesc();

  PrimaryRayData hitData;
  hitData.color = 0.0f;
  TraceRay(gScene.rtAccel, 0x0 /*rayFlags*/, 0xFF, 0 /* ray index*/, rayTypeCount, 0, ray, hitData);
  gOutput[launchIndex.xy] = hitData.color;
}