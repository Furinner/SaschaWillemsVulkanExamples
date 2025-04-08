#version 450

layout (binding = 0) uniform UBO 
{
	mat4 projection;
	mat4 model;
	mat4 view;
	int orthographic;
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

layout (location = 0) out vec3 outPos;
layout (location = 1) out vec3 outNormal;
layout (location = 2) out vec3 outFaceNor;
layout (location = 3) out vec3 outSymFaceNor;
//flat won't be interpolated, it will choose a provoking vertex for all primitives
//for example, Vertex's id is flat int, ver1.id = 0, ver2.id = 1, ver3.id = 2
//in fragment shader, all fragment of this triangle will have a id of 2, 
//since ver3(the last vertex) is the provoking vertex here.
layout (location = 4) out flat int outObjectID;
layout (location = 5) out flat int outFaceID;
layout (location = 6) out flat int outBorder;
layout (location = 7) out flat int outHeID;

void main() 
{
//	float x = -1;
//	float y = -1;
//	if(gl_VertexIndex == 0){
//
//	}else if(gl_VertexIndex == 1){
//		x = 3;
//		y = 3;
//	}
//	gl_Position = vec4(x, y, 1, 1);
	vec3 viewPos = vec3(ubo.view * ubo.model * vec4(inPos,1));
	
	//move all vertices towards positive z a small amount.
	//vec3 closerViewPos = viewPos + vec3(0, 0, 0.01);
	
	//lerp all vertices with camera. Far vertices will move more since all using one lerp value.
	//vec3 closerViewPos = mix(viewPos, vec3(0), 0.01);
	
	//move all vertices towards camera a small amount.
	//vec3 closerViewPos = viewPos - normalize(viewPos) * 0.01;
	
	vec3 closerViewPos = viewPos;

	gl_Position = ubo.projection * vec4(closerViewPos,1);
	outPos = closerViewPos;
	outNormal = inNormal;
	outFaceNor = mat3(ubo.view) * inFaceNor;
	outSymFaceNor = mat3(ubo.view) * inSymFaceNor;

	vec3 pixelNor;
	if(bool(ubo.orthographic)){
		pixelNor = vec3(0.f, 0.f, -1.f);
		//pixelNor = normalize(closerViewPos);
	}else{
		pixelNor = normalize(closerViewPos);
		//pixelNor = -mat3(ubo.view)[2];
	}

	if(inBorder == 1){
		outBorder = inBorder;
	}else{
		float checkVal1 = dot(mat3(ubo.view) * inFaceNor, pixelNor);
		float checkVal2 = dot(mat3(ubo.view) * inSymFaceNor, pixelNor);
		if((checkVal1 * checkVal2) <= 0.f){
			outBorder = 1;
		}else{
			outBorder = inBorder;
		}
	}

	outObjectID = inObjectID;
	outFaceID = inFaceID;
	outHeID = inHeID;
}