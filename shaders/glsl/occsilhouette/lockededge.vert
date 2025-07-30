#version 450

layout (binding = 0) uniform UBO 
{
	mat4 projection;
	mat4 model;
	mat4 view;
} ubo;

layout (location = 0) in vec3 inPos;
layout (location = 1) in vec3 inNormal;
layout (location = 2) in vec3 inFaceNor;
layout (location = 3) in vec3 inSymFaceNor;
layout (location = 4) in vec2 inUV;
layout (location = 5) in int inObjectID;
layout (location = 6) in int inFaceID;
layout (location = 7) in int inBorder;
layout (location = 8) in int inHeID;

layout (location = 0) out vec4 outColor;
layout (location = 1) out flat int outObjectID;
void main() 
{
	gl_Position = ubo.projection * ubo.view * ubo.model * vec4(inPos,1);
	outColor = vec4(inNormal, 1);
	outObjectID = inObjectID;
}