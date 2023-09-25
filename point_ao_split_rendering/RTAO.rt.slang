/*
 * Copyright (c) Meta Platforms, Inc. and affiliates.
 * All rights reserved.
 *
 * This source code is licensed under the license found in the
 * LICENSE file in the root directory of this source tree.
 */

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

uint rand_xorshift(inout uint rng_state)
{
    // Xorshift algorithm from George Marsaglia's paper
    rng_state ^= (rng_state << 13);
    rng_state ^= (rng_state >> 17);
    rng_state ^= (rng_state << 5);
    return rng_state;
}

float random( uint hash_val ) {
    const uint mantissaMask = 0x007FFFFFu;
    const uint one          = 0x3F800000u;
   
    uint h = hash_val;//hashInt1D( hash_val ) );
    h &= mantissaMask;
    h |= one;
    
    float  r2 = asfloat( h );
    return r2 - 1.0;
}

uint hashInt2D( uint x, uint y )
{
x += x >> 11;
x ^= x << 7;
x += y;
x ^= x << 6;
x += x >> 15;
x ^= x << 5;
x += x >> 12;
x ^= x << 9;
return x;
}

#define M_E                 2.71828182845904523536  // e
#define M_LOG2E             1.44269504088896340736  // log2(e)
#define M_LOG10E            0.434294481903251827651 // log10(e)
#define M_LN2               0.693147180559945309417 // ln(2)
#define M_LN10              2.30258509299404568402  // ln(10)
#define M_PI                3.14159265358979323846  // pi
#define M_PI_2              1.57079632679489661923  // pi/2
#define M_PI_4              0.785398163397448309616 // pi/4
#define M_1_PI              0.318309886183790671538 // 1/pi
#define M_2_PI              0.636619772367581343076 // 2/pi
#define M_2_SQRTPI          1.12837916709551257390  // 2/sqrt(pi)
#define M_SQRT2             1.41421356237309504880  // sqrt(2)
#define M_SQRT1_2           0.707106781186547524401 // 1/sqrt(2)
#define PI 3.14159265359


float3 fromLocal2(float3 v, float3 T, float3 B, float3 N)
{
    return T * v.x + B * v.y + N * v.z;
}

float2 sample_disk_concentric2(float2 u)
{
    u = 2.f * u - 1.f;
    if (u.x == 0.f && u.y == 0.f) return u;
    float phi, r;
    if (abs(u.x) > abs(u.y))
    {
        r = u.x;
        phi = (u.y / u.x) * M_PI_4;
    }
    else
    {
        r = u.y;
        phi = M_PI_2 - (u.x / u.y) * M_PI_4;
    }
    return r * float2(cos(phi), sin(phi));
}

float3 sample_cosine_hemisphere_concentric2(float2 u, out float pdf)
{
    float2 d = sample_disk_concentric2(u);
    float z = sqrt(max(0.f, 1.f - dot(d, d)));
    pdf = z * M_1_PI;
    return float3(d, z);
}

float3 randomSampleCosineHemisphereSD(float3 normal, float3 tangent, float2 rng)
{
	float pdf;
	float3 bitangent = cross(normal, tangent);
  float3 relativeVec = normalize(sample_cosine_hemisphere_concentric2(rng, pdf));
	
  return normalize(fromLocal2(relativeVec, tangent, bitangent, normal));
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
  uint3 launch_size = DispatchRaysDimensions();
	
  //TinyUniformSampleGenerator sg = TinyUniformSampleGenerator(launchIndex.xy, 42);

  uint rng_state = hashInt2D(launchIndex.x, launchIndex.y);

  float aoTerm = 0.0f;
  uint numHitRays = 0;
  for (int i = 0; i < 128; i++)
  {

    
    uint rng_val_1 = rand_xorshift(rng_state);
    uint rng_val_2 = rand_xorshift(rng_state);
    uint rng_val_3 = rand_xorshift(rng_state);

    float3 next_rng = float3(random(rng_val_1), random(rng_val_2), random(rng_val_3));

    //float3 next_rng = float3((i / 16) / 8.0f, (i % 16) / 16.0f, i / 128.0f);

    float3 direction = randomSampleCosineHemisphere(sd, next_rng.xy);
    //float3 direction = randomSampleCosineHemisphereSD(sd.N, sd.T, next_rng.xy);
    //direction = float3(1.0f, 0.0f, 0.0f);
		
    // Cast a ray towards that
    RayDesc ray;
    ray.Origin = sd.posW + 0.001f * sd.N;
    ray.Direction = normalize(direction);
    ray.TMin = 0.00;
    ray.TMax = sampleLength;

    AORayData rayData;
    rayData.hitT = -1.0f;
    
    TraceRay(gScene.rtAccel, 0x0 /*rayFlags*/, 0xFF, 1 /* ray index*/, rayTypeCount, 1, ray, rayData);
    
    //if (rayData.hitT >= 0 && rayData.hitT <= sampleLength)
    //{
      numHitRays++;
      aoTerm += rayData.hitT / sampleLength;
    //}
  }

  aoTerm /= (float) numHitRays;

  //if (aoTerm < 1.0f)
  //   aoTerm = 0.0f;

  //if (aoTerm < 0.7f)
  //    aoTerm = 0;
  //  else
  //    aoTerm = 1;
	
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
  TraceRay(gScene.rtAccel, 0x10 /*rayFlags*/, 0xFF, 0 /* ray index*/, rayTypeCount, 0, ray, hitData);
  gOutput[launchIndex.xy] = hitData.color;
}
