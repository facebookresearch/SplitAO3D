
import Scene.Shading;
import Scene.Raster;
import Scene.SceneTypes;
import Utils.Sampling.UniformSampleGenerator;
import Rendering.Lights.LightHelpers;
import Utils.Math.MathHelpers;

RWStructuredBuffer<PackedStaticVertexData> skinnedVertices;
RWStructuredBuffer<PackedStaticVertexData> outputVertices;

cbuffer perFrameConstantBuffer
{
	int instID;
	int vertexOffset;
	int vertexCount;
}


[numthreads(1, 1, 1)]
void main(
    uint3 groupId : SV_GroupID,
    uint3 groupThreadId : SV_GroupThreadID,
    uint3 dispatchThreadId : SV_DispatchThreadID,
    uint groupIndex : SV_GroupIndex)
{
	uint vertexID = dispatchThreadId.x + vertexOffset;
	uint outputVertexID = dispatchThreadId.x;
	StaticVertexData inVertex = skinnedVertices[vertexID].unpack();
	StaticVertexData outVertex = inVertex;
	
	const GeometryInstanceID instanceID = { instID };
	
	if (dispatchThreadId.x >= 0)
	{
		
		float4x4 worldMat = gScene.getWorldMatrix(instanceID);
		outVertex.position = mul(worldMat, float4(inVertex.position, 1.f)).xyz;
		outVertex.texCrd = inVertex.texCrd;
		outVertex.normal = mul(gScene.getInverseTransposeWorldMatrix(instanceID), inVertex.normal);
		
		float4 tangent = inVertex.tangent;
		outVertex.tangent = float4(mul((float3x3)gScene.getWorldMatrix(instanceID), tangent.xyz), tangent.w);
		
		outputVertices[outputVertexID].pack(outVertex);
	}

	//skinnedVertices[outputVertexID].position = float3(0, 0, 0);
	return;
}
