// Copyright (c) Facebook
import PointData;

StructuredBuffer<PointData> gpuPoints;

cbuffer perFrameConstantBuffer : register(b0)
{
  float4x4 matViewProj;
  float4x4 meshDesc;
  float4x4 invTransposemeshDesc;
  uint instanceOffset;
  uint instancePointCount;
  float quadSize;
};

struct PointQuadVertex
{
  float4 posH : SV_Position;
  float value : VertexValue;
  float3 N : Normal;
};

PointQuadVertex vsMain(uint vertexId : SV_VertexID)
{
  PointQuadVertex vOut;

  // Look up position stored in the point data buffer and transform with world matrix
  uint quadId = (vertexId) / 6 + instanceOffset;
  uint quadVertexId = vertexId % 6;
  float4 vertexPos = mul(float4(gpuPoints[quadId].position, 1), meshDesc);
  
  
  // Compute world space offset based on tangent and bitangent
  float2 offset;
  
  if (quadVertexId == 0)
    offset = quadSize * float2(-0.5, 0.5);
  else if (quadVertexId == 1)
    offset = quadSize * float2(-0.5, -0.5);
  else if (quadVertexId == 2)
    offset = quadSize * float2(0.5, 0.5);
  else if (quadVertexId == 3)
    offset = quadSize * float2(0.5, 0.5);
  else if (quadVertexId == 4)
    offset = quadSize * float2(-0.5, -0.5);
  else
    offset = quadSize * float2(0.5, -0.5);
  
  
  // Transform normal, tangent and compute bitangent
  float3 normal = normalize(mul(gpuPoints[quadId].normal, (float3x3) (invTransposemeshDesc)));
  
  float3 tangent = normalize(mul(gpuPoints[quadId].tangent, (float3x3) meshDesc));
  float3 bitangent = normalize(cross(normal, tangent));
  
  // Shift quad vertices in world space according to tangent space 
  vertexPos.xyz += offset.x * tangent + offset.y * bitangent;
  
  vertexPos.xyz += normal * 0.001f;
  
  // View projection
  vOut.posH = mul(vertexPos, matViewProj);
  
  // Show the value stored in the point structure
  vOut.value = gpuPoints[quadId].value;
  
  vOut.N = normal;
  
  // Invalid gpuPoints are set to NaN and then subsequently culled.
  if (gpuPoints[quadId].value < 0)
    vOut.posH = float4(asfloat(0x7fc00000u /* nan */), asfloat(0x7fc00000u /* nan */), asfloat(0x7fc00000u /* nan */), asfloat(0x7fc00000u /* nan */));
  
  return vOut;
}

float4 psMain(PointQuadVertex vOut) : SV_TARGET
{
  return float4(vOut.value, vOut.value, vOut.value, 1.0f);
}