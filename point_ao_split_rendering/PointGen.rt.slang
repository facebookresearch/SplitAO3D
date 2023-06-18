// (c) Meta Platforms, Inc. and its affiliates
import Scene.Raytracing;
import Scene.SceneTypes;
import Utils.Sampling.UniformSampleGenerator;
import Rendering.Lights.LightHelpers;
import Utils.Math.MathHelpers;
import PointData;

cbuffer perFrameConstantBuffer
{
  uint instanceId;
  uint uniformSamplesOffset;
  uint numTriangles;
};

RWStructuredBuffer<uint> triangleSampleCountsPerInstance;
RWStructuredBuffer<uint> triangleSampleOffsetsPerInstance;
RWStructuredBuffer<PointData> uniformPoints;

VertexData getUntransformedVertexData(const GeometryInstanceID instanceID, const uint triangleIndex, const float3 barycentrics)
{
  const uint3 vtxIndices = gScene.getIndices(instanceID, triangleIndex);
  StaticVertexData verts[3] = { gScene.getVertex(vtxIndices[0]), gScene.getVertex(vtxIndices[1]), gScene.getVertex(vtxIndices[2]) };

  VertexData v = { };

  v.posW = verts[0].position * barycentrics[0];
  v.posW += verts[1].position * barycentrics[1];
  v.posW += verts[2].position * barycentrics[2];

  v.normalW = verts[0].normal * barycentrics[0];
  v.normalW += verts[1].normal * barycentrics[1];
  v.normalW += verts[2].normal * barycentrics[2];

  v.tangentW = verts[0].tangent * barycentrics[0];
  v.tangentW += verts[1].tangent * barycentrics[1];
  v.tangentW += verts[2].tangent * barycentrics[2];
  v.tangentW.w = sign(v.tangentW.w); // Preserve zero to indicate invalid tangent.

  v.texC = verts[0].texCrd * barycentrics[0];
  v.texC += verts[1].texCrd * barycentrics[1];
  v.texC += verts[2].texCrd * barycentrics[2];

  return v;
}

[shader("raygeneration")]
void rayGen()
{
  uint3 launchIndex = DispatchRaysIndex();
  uint originalTriangleId = launchIndex.x;
  
  uint numSamples = triangleSampleCountsPerInstance[originalTriangleId];
  
  if (numSamples == 0)
    return;
  
  const GeometryInstanceID instanceID = { instanceId };
  UniformSampleGenerator sg = UniformSampleGenerator(uint2(uniformSamplesOffset, triangleSampleOffsetsPerInstance[originalTriangleId] * numSamples), originalTriangleId);
  
  uint materialID = gScene.getMaterialID(instanceID);
      
  const MaterialHeader mtl = gScene.materials.getMaterialHeader(materialID);

  // We start at the triangle we need, and move towards other triangles if necessary
  uint placedSamples = 0;
  int numTriangleTrials = 5;

  if (mtl.isDoubleSided())
  {
    numTriangleTrials = 50;
  }

  uint triangleId = originalTriangleId;
  
  while(numTriangleTrials--)
  {
    int numTrials = (numSamples - placedSamples) * 5;
    while (numTrials--)
    {
      // https://pharr.org/matt/blog/2019/02/27/triangle-sampling-1
      float su0 = sqrt(sampleNext1D(sg));
      float b0 = 1 - su0;
      float b1 = sampleNext1D(sg) * su0;
      
      const float3 barycentrics = { b0, b1, 1.0 - b0 - b1 };
      
      
      VertexData v = getUntransformedVertexData(instanceID, triangleId, barycentrics);
  
      if (!gScene.materials.alphaTest(v, materialID, 0.f))
      {
        // If we're fully opaque, we can use this sample
        PointData pd;
        pd.position = v.posW;
        pd.normal = v.normalW;
        pd.tangent = v.tangentW.xyz;
        pd.barycentrics = barycentrics.xy;
        pd.instanceTriangleId = triangleId;
        pd.instanceId = instanceId;
        pd.value = 1.0f;
        
        // If we have a doublesided material, randomly flip the normal
        // This is later used on the CPU side as well to run 2 point generation passes, one for each side of the face.
        if(mtl.isDoubleSided() && sampleNext1D(sg) < 0.5)
        {
          pd.normal = -pd.normal;
          pd.value = -1.0f;
        }
        
        uniformPoints[uniformSamplesOffset + triangleSampleOffsetsPerInstance[originalTriangleId] + placedSamples] = pd;
        placedSamples++;
        
        if(placedSamples >= numSamples)
          return;
      }
    }
    
    // If we reach here, we reroll to a different triangle and try that.
    // This can sometimes happen if a triangle is very / mostly transparent.
    triangleId = uint(clamp(sampleNext1D(sg), 0, 1) * numTriangles);
  }
  
  // If we reach here, place the point(s) far away as a hack.
  // Ideally, this shouldn't happen too often as the CPU poisson disk elimination code doesn't handle points that are in exactly the same spot super well.
  while(placedSamples < numSamples)
  {
    uniformPoints[uniformSamplesOffset + triangleSampleOffsetsPerInstance[originalTriangleId] + placedSamples].position = float3(9999999, 9999999, 9999999);
    uniformPoints[uniformSamplesOffset + triangleSampleOffsetsPerInstance[originalTriangleId] + placedSamples].value = -1.0f;
    placedSamples++;
  }
    
  
}
