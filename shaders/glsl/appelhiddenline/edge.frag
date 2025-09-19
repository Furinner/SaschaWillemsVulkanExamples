#version 450

layout (location = 0) in vec3 inPos;
layout (location = 1) in vec3 inNormal;
layout (location = 2) in vec3 inFaceNor;
layout (location = 3) in vec3 inSymFaceNor;
layout (location = 4) in flat int inObjectID;
layout (location = 5) in flat int inFaceID;
layout (location = 6) in flat int inBorder;
layout (location = 7) in flat int inHeID;
layout (location = 8) in flat int inDebug;
layout (location = 9) in flat int inGlobalHeID;

layout (location = 0) out ivec4 outPosition;
//layout(depth_less) out float gl_FragDepth; //声明只会把深度变小，以保留early-z

float nextDown(float x) {
    if (isnan(x) || isinf(x)) return x;
    uint ux = floatBitsToUint(x);//不改变bit，只改变解释方式
    if (ux == 0x00000000u) ux = 0x80000000u; // +0.0 -> -0.0

    if (uintBitsToFloat(ux) > 0.0){
        ux -= 1u;
    }else{
        ux += 1u;
    }
    return uintBitsToFloat(ux);
}

//discard directly throw away current fragment
//this fragment won't affect color, depth and stencil attachment.
void main() 
{
    if(inDebug == 1){
        outPosition = ivec4(-1,-1,1,2);
    }else if(inBorder == 1){
		//outPosition = ivec4(inObjectID,inHeID,1,1);
        outPosition = ivec4(inObjectID,inHeID,1,1);
	}else{
		discard;
	}
    //float z = gl_FragCoord.z;
    //gl_FragDepth = nextDown(z);

	//outPosition = vec4(inFaceNor, 1);
}