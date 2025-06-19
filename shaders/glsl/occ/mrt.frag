#version 450

layout (binding = 1) uniform sampler2D samplerColor;
layout (binding = 2) uniform sampler2D samplerNormalMap;

layout (location = 0) in vec3 inNormal;
layout (location = 1) in vec3 inWorldPos;
layout (location = 2) in flat int inObjectID;
layout (location = 3) in vec3 inCameraPos;
layout (location = 4) in flat int inFaceID;
layout (location = 5) in vec2 inUV;

layout (location = 0) out vec4 outPosition;
layout (location = 1) out vec4 outNormal;
layout (location = 2) out ivec2 outAlbedo;

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
	vec3 color = vec3(1);
	vec3 N = normalize(inNormal);
	vec3 L = normalize(inCameraPos);
	float intensity = dot(N,L);
	if (intensity > 0.98)
		color = color * 1.5;
	else if  (intensity > 0.9)
		color = color * 1.0;
	else if (intensity > 0.5)
		color = color * 0.6;
	else if (intensity > 0.25)
		color = color * 0.4;
	else
		color = color * 0.2;
	color = vec3(mix(color, vec3(dot(vec3(0.2126,0.7152,0.0722), color)), 0.1));	
	//outPosition = vec4(inWorldPos,inUV.x);
	//outPosition = vec4(inCameraPos,inUV.x);
	outPosition = vec4(color,inUV.x);
	outNormal = vec4(inNormal, inUV.y);
	outAlbedo.r = inObjectID;
	outAlbedo.g = inFaceID;
}