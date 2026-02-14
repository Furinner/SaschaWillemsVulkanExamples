#version 450
#extension GL_EXT_debug_printf : enable

const int MAX_VER_CNT = 200;
const int MAX_SIL_LINK_CNT = 4;

layout (binding = 0) uniform UBO 
{
	mat4 projection;
	mat4 model;
	mat4 view;
	int orthographic;
} ubo;


layout (binding = 10) buffer SBO1
{
	uint edgeCnt[]; //每个object有多少sEdge
} sbo1;

layout (binding = 11) buffer SBO2
{
	uint edgeList[];  //每个object的sEdge的gHeID，每个object留出MAX_VER_CNT * MAX_SIL_LINK_CNT个位置
} sbo2;


layout (location = 0) in vec3 inPos;
layout (location = 1) in vec3 inNormal;
layout (location = 2) in vec3 inFaceNor;
layout (location = 3) in vec3 inSymFaceNor;
layout (location = 4) in vec2 inUV;
layout (location = 5) in int inObjectID;
layout (location = 6) in int inFaceID;
layout (location = 7) in int inBorder;
layout (location = 8) in int inHeID;
layout (location = 9) in int inDebug;
layout (location = 10) in int inGlobalHeID;

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
layout (location = 8) out flat int outDebug;
layout (location = 9) out flat int outGlobalHeID;

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
	//debugPrintfEXT("index=%u", sbo1.edgeCnt);
	vec3 viewPos = vec3(ubo.view * ubo.model * vec4(inPos,1));
	
	//move all vertices towards positive z a small amount.
	vec3 closerViewPos = viewPos + vec3(0, 0, 0.01);
	
	//lerp all vertices with camera. Far vertices will move more since all using one lerp value.
	//vec3 closerViewPos = mix(viewPos, vec3(0), 0.01);
	
	//move all vertices towards camera a small amount.
	//vec3 closerViewPos = viewPos - normalize(viewPos) * 0.01;
	
	//vec3 closerViewPos = viewPos;

	gl_Position = ubo.projection * vec4(closerViewPos,1);
	outPos = closerViewPos;
	outNormal = inNormal;
	outFaceNor = mat3(ubo.view) * inFaceNor;
	outSymFaceNor = mat3(ubo.view) * inSymFaceNor;
	outDebug = inDebug;
	outObjectID = inObjectID;
	outFaceID = inFaceID;
	outHeID = inHeID;
	outGlobalHeID = inGlobalHeID;
	outBorder = inBorder;

	bool startPoint = ((gl_VertexIndex & 1u) == 0u);

	vec3 pixelNor;
	if(bool(ubo.orthographic)){
		pixelNor = vec3(0.f, 0.f, 1.f);
		//pixelNor = normalize(closerViewPos);
	}else{
		pixelNor = normalize(closerViewPos);
		//pixelNor = -mat3(ubo.view)[2];
	}
	
	if(inBorder >= 1){
		outBorder = inBorder;
		if(startPoint){
			//uint idx = atomicAdd(sbo1.edgeCnt, 1);
			//sbo2.edgeList[idx] = inGlobalHeID;
		}
	}else{
		vec3 fn = mat3(ubo.view) * inFaceNor;
		vec3 sfn = mat3(ubo.view) * inSymFaceNor;
		float fnz = dot(fn, pixelNor);
		float sfnz = dot(sfn, pixelNor);
		if(fnz == 0.f && sfnz == 0.f){
			return;
		}
		if((fnz * sfnz) <= 0.f){
			outBorder = 1;
			if(startPoint){
				uint idx = atomicAdd(sbo1.edgeCnt[inObjectID], 1);
				sbo2.edgeList[MAX_VER_CNT * MAX_SIL_LINK_CNT * inObjectID + idx] = inGlobalHeID;
			}
		}else{
			outBorder = inBorder;
		}
	}
}