#version 450

layout (location = 0) in vec3 inPos;
layout (location = 1) in vec3 inNormal;
layout (location = 2) in vec3 inFaceNor;
layout (location = 3) in vec3 inSymFaceNor;
layout (location = 4) in flat int inObjectID;
layout (location = 5) in flat int inFaceID;
layout (location = 6) in flat int inBorder;

layout (location = 0) out vec4 outPosition;

//discard directly throw away current fragment
//this fragment won't affect color, depth and stencil attachment.
void main() 
{
	if(inBorder == 1){
		outPosition = vec4(1,1,1,1);
	}else{
		discard;
	}
	//outPosition = vec4(inFaceNor, 1);
}