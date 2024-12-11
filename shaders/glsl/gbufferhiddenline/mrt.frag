#version 450

layout (binding = 1) uniform sampler2D samplerColor;
layout (binding = 2) uniform sampler2D samplerNormalMap;

layout (location = 0) in vec3 inNormal;
layout (location = 1) in vec3 inWorldPos;
layout (location = 2) in flat int inObjectID;
layout (location = 3) in vec3 inCameraPos;

layout (location = 0) out vec4 outPosition;
layout (location = 1) out vec4 outNormal;
layout (location = 2) out vec4 outAlbedo;

void main() 
{
//	outPosition = vec4(inWorldPos, 1.0);
//
//	// Calculate normal in tangent space
//	vec3 N = normalize(inNormal);
//	vec3 T = normalize(inTangent);
//	vec3 B = cross(N, T);
//	mat3 TBN = mat3(T, B, N);
//	vec3 tnorm = TBN * normalize(texture(samplerNormalMap, inUV).xyz * 2.0 - vec3(1.0));
//	outNormal = vec4(tnorm, 1.0);
//
//	outAlbedo = texture(samplerColor, inUV);
	outPosition = vec4(inCameraPos,1);
	outNormal = vec4(vec3(inWorldPos), 1.0);
	outAlbedo = vec4(vec3(inObjectID + 1), 1.0);
}