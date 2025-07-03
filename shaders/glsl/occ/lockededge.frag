#version 450
layout (location = 0) in vec4 inColor;
layout (location = 1) in flat int inObjectID;

layout (binding = 4) uniform UBO 
{
	float colorInten;
} ubo;

layout (location = 0) out vec4 outPosition;
layout (location = 1) out vec4 outNormal;

layout(push_constant) uniform PushConsts {
	float colorInten;
} pushConsts;

void main() 
{
	//outPosition = vec4(1,1,1,1);
	//outPosition = vec4(inFaceNor, 1);
	if(inObjectID == 0){
		outPosition = inColor;
	}else if(inObjectID == 3){
		outNormal = inColor;
	}else{
		if(ubo.colorInten <= 0.25f){
			discard;
		}else{
			outPosition = inColor * ubo.colorInten;
		}
	}
}