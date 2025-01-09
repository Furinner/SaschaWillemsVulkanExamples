#version 450

layout (location = 0) in vec3 inNormal;
layout (location = 1) in vec3 inWorldPos;
layout (location = 2) in flat int inObjectID;
layout (location = 3) in flat int inFaceID;

layout (location = 0) out vec4 outPosition;
layout (location = 1) out vec4 outNormal;
layout (location = 2) out ivec2 outAlbedo;

void main() 
{
	outPosition = vec4(1,1,0, 1);
}