#version 450

layout (binding = 0) uniform UBO 
{
	mat4 projection;
	mat4 model;
	mat4 view;
} ubo;

layout (location = 0) in vec3 inPos;
layout (location = 1) in vec3 inNormal;
layout (location = 2) in int inObjectID;
layout (location = 3) in int inFaceID;

layout (location = 0) out vec3 outPos;
layout (location = 1) out vec3 outNormal;
layout (location = 2) out flat int outObjectID;
layout (location = 3) out flat int outFaceID;

void main() 
{
	//gl_Position = ubo.projection * ubo.view * ubo.model * vec4(inPos,1);
	float x = -1;
	float y = -1;
	if(gl_VertexIndex == 0){
		y = 3;
	}else if(gl_VertexIndex == 1){

	}else{
		x=3;
	}
	outPos = vec3(ubo.projection * ubo.view * ubo.model * vec4(inPos, 1));
	outNormal = inNormal;
	outObjectID = inObjectID;
	outFaceID = inFaceID;
}
