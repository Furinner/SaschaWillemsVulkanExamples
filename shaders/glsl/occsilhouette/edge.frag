#version 450

layout (location = 0) in vec3 inPos;
layout (location = 1) in vec3 inNormal;
layout (location = 2) in vec3 inFaceNor;
layout (location = 3) in vec3 inSymFaceNor;
layout (location = 4) in flat int inObjectID;
layout (location = 5) in flat int inFaceID;
layout (location = 6) in flat int inBorder;
layout (location = 7) in flat int inHeID;

layout (location = 0) out ivec4 outPosition;
layout (location = 1) out vec4 outNormal;

layout(push_constant) uniform PushConsts {
	float colorInten;
} pushConsts;

layout (binding = 4) uniform UBO 
{
	float colorInten;
} ubo;

//discard directly throw away current fragment
//this fragment won't affect color, depth and stencil attachment.
void main() 
{
	if(inBorder == 1){
		outPosition = ivec4(inObjectID,inHeID,1,1);
		if(inObjectID == 0){
			outNormal = vec4(inNormal, 1);
		}else{
			if(ubo.colorInten <= 0.25f){
				discard;
			}else{
				outNormal = vec4(inNormal * ubo.colorInten, 1);
			}
		}
	}else{
		discard;
	}
	//outPosition = vec4(inFaceNor, 1);
}