#version 450

layout (location = 0) in vec3 inPos;
layout (location = 1) in vec3 inNormal;
layout (location = 2) in int objectID;
layout (location = 3) in int faceID;

layout (binding = 0) uniform UBO 
{
	mat4 projection;
	mat4 model;
	mat4 view;
	vec4 instancePos[3];
} ubo;

layout (location = 0) out vec3 outNormal;
layout (location = 1) out vec3 outWorldPos;
layout (location = 2) out flat int outObjectID;
layout (location = 3) out vec3 outCameraPos;
layout (location = 4) out flat int outFaceID;

void main() 
{
	gl_Position = ubo.projection * ubo.view * ubo.model * vec4(inPos,1);

	// Vertex position in world space
	vec3 camPos = vec3(-ubo.view[3]);
	//outWorldPos = normalize(vec3(ubo.view * vec4(camPos, 1)) - vec3(ubo.view * vec4(inPos, 1)));
	outWorldPos = vec3(ubo.projection * ubo.view * vec4(inPos, 1));
	// Normal in world space
	//mat3 mNormal = transpose(inverse(mat3(ubo.model)));
	//outNormal = mNormal * normalize(inNormal);	
	//outNormal = mat3(ubo.view) * normalize(inNormal);
	outNormal = inNormal;
	outCameraPos = vec3(-ubo.model[3]);
	outObjectID = objectID;
	outFaceID = faceID;
}
