#version 450

layout (binding = 1) uniform sampler2D samplerposition;
layout (binding = 2) uniform sampler2D samplerNormal;
layout (binding = 3) uniform isampler2D samplerAlbedo;
layout (binding = 8) uniform isampler2D samplerEdge;
layout (binding = 9) uniform sampler2D samplerLockedEdge;
layout (binding = 12) uniform sampler2D samplerEdge2;

layout (location = 0) in vec2 inUV;

layout (location = 0) out vec4 outFragcolor;

struct Light {
	vec4 position;
	vec3 color;
	float radius;
};

layout (binding = 4) uniform UBO 
{
	Light lights[6];
	vec4 viewPos;
	mat4 camView;
	mat4 camViewTr;
	int displayDebugTarget;
	int singleStride;
	float depthFactor;
	int uFactor;
	int vFactor;
	int orthographic;
} ubo;

layout (binding = 5) readonly buffer FaceInfo
{
    int faceInfos[];
};

layout (binding = 6) readonly buffer FaceData
{
    int faceData[];
};

layout (binding = 7) readonly buffer FaceNor
{
	vec4 faceNor[];
};

layout(push_constant) uniform PushConsts {
	int max_neighbor;
	float screenHalfLengthX;
	float screenHalfLengthY;
} pushConsts;


bool grid_3x3(vec2 center_uv, vec2 tex_offset){
	bool shouldColor = false;
	int faceID = texture(samplerAlbedo, center_uv).g;  
	int objectID = texture(samplerAlbedo, center_uv).r;  
	bool inObj = false;
	int neighborFaceCnt = 0;
	int faceIdxStart = 0;
	if(objectID > -1){
		inObj = true;
	};
	if(inObj){
		faceIdxStart = faceInfos[objectID] * pushConsts.max_neighbor + pushConsts.max_neighbor * faceID;
		for(int i = 0; i < pushConsts.max_neighbor; ++i){
			if(faceData[faceIdxStart + i] > -1){
				++neighborFaceCnt;
			}else{
				break;
			}
		}
	};
	for(int i = -1; i <= 1; ++i){
		for(int j = -1; j <= 1; ++j){
			if((i == 0) && (j == 0)){
				continue;
			}
			int currFaceID = texture(samplerAlbedo, center_uv + vec2(tex_offset.x * i, tex_offset.y * j)).g;
			if(currFaceID != faceID){
				if(neighborFaceCnt > 0){
					bool haveThisNeighbor = false;
					for(int k = faceIdxStart; k < faceIdxStart + neighborFaceCnt; ++k){
						if(faceData[k] == currFaceID){
							haveThisNeighbor = true;
							break;
						}
					}
					if(haveThisNeighbor){
						continue;
					}else{
						shouldColor = true;
						return shouldColor;
					}
				}else{
					shouldColor = true;
					return shouldColor;
				}
			}
		}
	}
	return shouldColor;
}

bool grid_3x3_2(vec2 center_uv, vec2 tex_offset){
	bool shouldColor = false;
	vec2 screen_uv = center_uv * 2 -1;
	screen_uv.y = - screen_uv.y;
	vec2 screen_length = vec2(pushConsts.screenHalfLengthX, pushConsts.screenHalfLengthY);
	vec3 negViewDir = mat3(ubo.camViewTr) * -normalize(vec3(screen_length * screen_uv, -1));
	int faceID = texture(samplerAlbedo, center_uv).g;  
	int objectID = texture(samplerAlbedo, center_uv).r;
	int objFaceInfo = faceInfos[objectID];
	int faceIdx = objFaceInfo + faceID;
	bool inObj = false;
	int neighborFaceCnt = 0;
	int faceIdxStart = 0;
	if(objectID > -1){
		inObj = true;
	};
	if(inObj){
		faceIdxStart = faceIdx * pushConsts.max_neighbor;
		for(int i = 0; i < pushConsts.max_neighbor; ++i){
			if(faceData[faceIdxStart + i] > -1){
				++neighborFaceCnt;
			}else{
				break;
			}
		}
	};
	vec3 nor = vec3(faceNor[faceIdx]);
	float checkVal1 = dot(nor, negViewDir);
	float checkVal2 = 1.f;
//	if(checkVal1 <= 0.f){
//		return true;
//	}
	for(int i = -1; i <= 1; ++i){
		for(int j = -1; j <= 1; ++j){
			if((i == 0) && (j == 0)){
				continue;
			}
			vec2 currUV = center_uv + vec2(tex_offset.x * i, tex_offset.y * j);
			vec2 curr_screen_uv = currUV * 2 -1;
			curr_screen_uv.y = - screen_uv.y;
			int currFaceID = texture(samplerAlbedo, currUV).g;
			if(currFaceID != faceID){
				if(neighborFaceCnt > 0){
					bool haveThisNeighbor = false;
					for(int k = faceIdxStart; k < faceIdxStart + neighborFaceCnt; ++k){
						if(faceData[k] == currFaceID){
							haveThisNeighbor = true;
							break;
						}
					}
					if(haveThisNeighbor){
						continue;
					}else{
						//check neighbor for this pixel  (center_uv)
						for(int k = faceIdxStart; k < faceIdxStart + neighborFaceCnt; ++k){
							int neighborFaceIdx = objFaceInfo + faceData[k];
							vec3 neighborNor = vec3(faceNor[neighborFaceIdx]);
							checkVal2 = dot(neighborNor, negViewDir);
							//if it has one neighbor face's normal dot with camera direction is negative
							if(checkVal2 <= 0.f){
								shouldColor = true;
								return shouldColor;
							}else{
								continue;
							}
						}
						//check neighbor for the other pixel
						int faceIdx2 = objFaceInfo + currFaceID;
						int faceIdxStart2 = faceIdx2 * pushConsts.max_neighbor;
						int neighborFaceCnt2 = 0;
						for(int k = 0; k < pushConsts.max_neighbor; ++k){
							if(faceData[faceIdxStart2 + k] > -1){
								++neighborFaceCnt2;
							}else{
								break;
							}
						}
						vec3 nor2 = vec3(faceNor[faceIdx2]);
						vec3 negViewDir2 = mat3(ubo.camViewTr) * -normalize(vec3(screen_length * curr_screen_uv, -1));
						checkVal2 = dot(nor2, negViewDir2);
//						if(checkVal2 <= 0.f){
//							shouldColor = true;
//							return shouldColor;
//						}
						for(int k = faceIdxStart2; k < faceIdxStart2 + neighborFaceCnt2; ++k){
							int neighborFaceIdx = objFaceInfo + faceData[k];
							vec3 neighborNor = vec3(faceNor[neighborFaceIdx]);
							checkVal2 = dot(neighborNor, negViewDir2);
							if(checkVal2 <= 0.f){
								shouldColor = true;
								return shouldColor;
							}else{
								continue;
							}
						}
					}
				}else{
					shouldColor = true;
					return shouldColor;
				}
			}
		}
	}
	return shouldColor;
}


bool sampleOn3x3Grid(vec2 low_left_uv, vec2 tex_offset, int stride){
	int move = (stride - 1) * 2;
	for(int i = 0; i <= move; ++i){
		for(int j = 0; j <= move; ++j){
			vec2 currUV = low_left_uv + vec2(tex_offset.x * i, tex_offset.y * j);
			if(grid_3x3(currUV, tex_offset)){
				return true;
			}
		}
	}
	return false;
}

bool sampleOn3x3Grid2(vec2 low_left_uv, vec2 tex_offset, int stride){
	int move = (stride - 1) * 2;
	for(int i = 0; i <= move; ++i){
		for(int j = 0; j <= move; ++j){
			vec2 currUV = low_left_uv + vec2(tex_offset.x * i, tex_offset.y * j);
			if(grid_3x3_2(currUV, tex_offset)){
				return true;
			}
		}
	}
	return false;
}


vec3 case3(int size, vec2 inUV, vec2 tex_offset){
	int objectID = texture(samplerAlbedo, inUV).r;  
	int faceID = texture(samplerAlbedo, inUV).g;  
	for(int i = -size; i <= size; ++i){
		for(int j = -size; j <= size; ++j){
			if((i == 0) && (j == 0)){
				continue;
			}
			int currID = texture(samplerAlbedo, inUV + vec2(tex_offset.x * i, tex_offset.y * j)).r;
			if(abs(currID - objectID) > 0){
				return vec3(1);
			}else{
				int currFaceID = texture(samplerAlbedo, inUV + vec2(tex_offset.x * i, tex_offset.y * j)).g;
				if(abs(currFaceID - faceID) > 0){
					return vec3(1);
				}
			}
		}
	}
	return vec3(0);
}

vec3 case4(int size, vec2 inUV, vec2 tex_offset){
	int objectID = texture(samplerAlbedo, inUV).r;  
	for(int i = -size; i <= size; ++i){
		for(int j = -size; j <= size; ++j){
			if((i == 0) && (j == 0)){
				continue;
			}
			int currID = texture(samplerAlbedo, inUV + vec2(tex_offset.x * i, tex_offset.y * j)).r;
			if(abs(currID - objectID) > 0){
				return vec3(1);
			}
		}
	}
	return vec3(0);
}

vec3 case5(int size, vec2 center_uv, vec2 tex_offset){
	int objectID = texture(samplerAlbedo, center_uv).r;  
	bool inObj = false;
	if(objectID > -1){
		inObj = true;
	};
	for(int i = -size; i <= size; ++i){
		for(int j = -size; j <= size; ++j){
			if((i == 0) && (j == 0)){
				continue;
			}
			int currID = texture(samplerAlbedo, center_uv + vec2(tex_offset.x * i, tex_offset.y * j)).r;
			if(abs(currID - objectID) > 0){
				return vec3(1);
			}
		}
	}
	if(inObj){
		vec2 low_left_uv = center_uv + vec2(-tex_offset.x * (size-1), -tex_offset.y * (size-1));
		bool shouldColor = sampleOn3x3Grid(low_left_uv, tex_offset, size);
		if(shouldColor){
			return vec3(1);
		}
	}
	return vec3(0);
}

//LineFaceNor
vec3 case6(int size, vec2 center_uv, vec2 tex_offset){
	int objectID = texture(samplerAlbedo, center_uv).r;  
	vec3 normalTex = texture(samplerNormal, center_uv).rgb;
	bool inObj = false;
	if(objectID > -1){
		inObj = true;
	};
	vec2 screen_length = vec2(pushConsts.screenHalfLengthX, pushConsts.screenHalfLengthY);
	if(inObj){
		vec2 screen_uv = center_uv * 2 -1;
		screen_uv.y = - screen_uv.y;
		vec3 screen_normal = mat3(ubo.camViewTr) * -normalize(vec3(screen_length * screen_uv, -1));
//		if(dot(screen_normal, normalTex) <= 0.f){
//			return vec3(1);
//		}
	}
	for(int i = -size; i <= size; ++i){
		for(int j = -size; j <= size; ++j){
			if((i == 0) && (j == 0)){
				continue;
			}
			vec2 currUV = center_uv + vec2(tex_offset.x * i, tex_offset.y * j);
			//check objectID
			int currObjID = texture(samplerAlbedo, currUV).r;
			if(abs(currObjID - objectID) > 0){
				return vec3(1);
			}
			//check normal
			if(currObjID > -1){
				vec3 curr_normal_tex = texture(samplerNormal, currUV).rgb;
				vec2 curr_screen_uv = currUV * 2 -1;
				curr_screen_uv.y = -curr_screen_uv.y;
				vec3 curr_screen_normal = mat3(ubo.camViewTr) * -normalize(vec3(screen_length * curr_screen_uv, -1));
//				if(dot(curr_normal_tex, curr_screen_normal) <= 0.f){
//					return vec3(1);
//				}
			}
		}
	}
	if(inObj){
		//if size == 1, then low_left_uv is just center_uv
		vec2 low_left_uv = center_uv + vec2(-tex_offset.x * (size-1), -tex_offset.y * (size-1));
		bool shouldColor = sampleOn3x3Grid2(low_left_uv, tex_offset, size);
		if(shouldColor){
			return vec3(1);
		}
	}
	return vec3(0);
}


vec3 case7(int size, vec2 center_uv, vec2 tex_offset){
	//pure normal
	int objectID = texture(samplerAlbedo, center_uv).r;  
	vec3 normalTex = texture(samplerNormal, center_uv).rgb;
	vec2 screen_length = vec2(pushConsts.screenHalfLengthX, pushConsts.screenHalfLengthY);
	bool noObj = true;
	for(int i = -size; i <= size; ++i){
		for(int j = -size; j <= size; ++j){
			int currObjID = texture(samplerAlbedo, center_uv).r; 
			if(currObjID != -1){
				noObj = false;
				break;
			}
		}
		if(!noObj){
			break;
		}
	}
	if(noObj){
		return vec3(0);
	}
	for(int i = -size; i <= size; ++i){
		for(int j = -size; j <= size; ++j){
			if((i == 0) && (j == 0)){
				vec2 screen_uv = center_uv * 2 -1;
				screen_uv.y = -screen_uv.y;
				vec3 screen_normal = mat3(ubo.camViewTr) * -normalize(vec3(screen_length * screen_uv, -1));
				if(dot(normalTex, screen_normal) <= 0.f){
					return vec3(1);
				}
				continue;
			}
			vec2 currUV = center_uv + vec2(tex_offset.x * i, tex_offset.y * j);
			//check objectID
			int currObjID = texture(samplerAlbedo, currUV).r;
			if(abs(currObjID - objectID) > 0){
				return vec3(1);
			}
			//check normal
			if(currObjID > -1){
				vec3 curr_normal_tex = texture(samplerNormal, currUV).rgb;
				vec2 curr_screen_uv = currUV * 2 -1;
				curr_screen_uv.y = -curr_screen_uv.y;
				vec3 curr_screen_normal = mat3(ubo.camViewTr) * -normalize(vec3(screen_length * curr_screen_uv, -1));
				if(dot(curr_normal_tex, curr_screen_normal) <= 0.f){
					return vec3(1);
				}
			}
		}
	}
	return vec3(0);
}

vec3 case8(int size, vec2 center_uv, vec2 tex_offset){
	//depth nor
	int objectID = texture(samplerAlbedo, center_uv).r;  
	vec3 normalTex = texture(samplerNormal, center_uv).rgb;
	vec2 screen_length = vec2(pushConsts.screenHalfLengthX, pushConsts.screenHalfLengthY);
	bool noObj = true;
	for(int i = -size; i <= size; ++i){
		for(int j = -size; j <= size; ++j){
			int currObjID = texture(samplerAlbedo, center_uv).r; 
			if(currObjID != -1){
				noObj = false;
				break;
			}
		}
		if(!noObj){
			break;
		}
	}
	if(noObj){
		return vec3(0);
	}
	for(int i = -size; i <= size; ++i){
		for(int j = -size; j <= size; ++j){
			if((i == 0) && (j == 0)){
				vec2 screen_uv = center_uv * 2 -1;
				screen_uv.y = -screen_uv.y;
				vec3 screen_normal = mat3(ubo.camViewTr) * -normalize(vec3(screen_length * screen_uv, -1));
				if(dot(normalTex, screen_normal) <= 0.f){
					return vec3(1);
				}
				continue;
			}
			vec2 currUV = center_uv + vec2(tex_offset.x * i, tex_offset.y * j);
			//check objectID
			int currObjID = texture(samplerAlbedo, currUV).r;
			if(abs(currObjID - objectID)*100.f > 0){
				return vec3(1);
			}
			//check normal
//			if(currObjID > -1){
//				vec3 curr_normal_tex = texture(samplerNormal, currUV).rgb;
//				vec2 curr_screen_uv = currUV * 2 -1;
//				curr_screen_uv.y = -curr_screen_uv.y;
//				vec3 curr_screen_normal = mat3(ubo.camViewTr) * -normalize(vec3(screen_length * curr_screen_uv, -1));
//				if(dot(curr_normal_tex, curr_screen_normal) <= 0.f){
//					return vec3(1);
//				}
//			}
		}
	}
	int sampleNum = 2 * size * 2;
	float depthVal = 0.f;
	float center_depth = -texture(samplerposition, center_uv).b;
	for(int i = -size; i <= size; ++i){
		if(i == 0){
			continue;
		}
		vec2 currUV = center_uv + vec2(tex_offset.x * i, 0);
		depthVal += -texture(samplerposition, currUV).z;
	}
	for(int i = -size; i <= size; ++i){
		if(i == 0){
			continue;
		}
		vec2 currUV = center_uv + vec2(0, tex_offset.y * i);
		depthVal += -texture(samplerposition, currUV).z;
	}
	depthVal /= sampleNum;
	return vec3(clamp((center_depth - depthVal) * ubo.depthFactor, 0.f, 1.f));
	return vec3(0);
}

vec3 case9(int size, vec2 inUV, vec2 tex_offset, int uFactor, int vFactor){
	//isoparametric line
	int objectID = texture(samplerAlbedo, inUV).r;  
	int faceID = texture(samplerAlbedo, inUV).g;  
	vec2 uvVal = vec2(texture(samplerposition, inUV).a, texture(samplerNormal, inUV).a);
	float uDiv = 1.f / uFactor;
	float vDiv = 1.f / vFactor;
	int uSlot = int(uvVal.x / uDiv);
	int vSlot = int(uvVal.y / vDiv);
	bool noObj = true;
	for(int i = -size; i <= size; ++i){
		for(int j = -size; j <= size; ++j){
			if((i == 0) && (j == 0)){
				continue;
			}
			int currID = texture(samplerAlbedo, inUV + vec2(tex_offset.x * i, tex_offset.y * j)).r;
			if(abs(currID - objectID) > 0){
				return vec3(1);
			}else{
				if((currID == -1) || (objectID) == -1){
					continue;
				}
				vec2 currUVVal = vec2(texture(samplerposition, inUV + vec2(tex_offset.x * i, tex_offset.y * j)).a, texture(samplerNormal, inUV + vec2(tex_offset.x * i, tex_offset.y * j)).a);
				int currUSlot = int(currUVVal.x / uDiv);
				int currVSlot = int(currUVVal.y / vDiv);
				if(((currUSlot - uSlot) > 0) || ((currVSlot - vSlot) > 0)){
					return vec3(1);
				}
			}
		}
	}
	return vec3(0);
}

vec3 case10(int size, vec2 inUV, vec2 tex_offset){
	//edge
	ivec3 center_edge =  texture(samplerEdge, inUV).rgb;
	if(center_edge.b == 1){
		return vec3(1);
	}
	int objectID = texture(samplerAlbedo, inUV).r;  
	for(int i = -size; i <= size; ++i){
		for(int j = -size; j <= size; ++j){
			if((i == 0) && (j == 0)){
				continue;
			}
			int currID = texture(samplerAlbedo, inUV + vec2(tex_offset.x * i, tex_offset.y * j)).r;
			if(abs(currID - objectID) > 0){
				return vec3(1);
			}
		}
	}
	return vec3(0);
}

vec3 case11(int size, vec2 inUV, vec2 tex_offset, int uFactor, int vFactor){
	//edge UV
	ivec3 center_edge =  texture(samplerEdge, inUV).rgb;
	if(center_edge.b == 1){
		return vec3(1);
	}
	int objectID = texture(samplerAlbedo, inUV).r; 
	vec2 uvVal = vec2(texture(samplerposition, inUV).a, texture(samplerNormal, inUV).a);
	float uDiv = 1.f / uFactor;
	float vDiv = 1.f / vFactor;
	int uSlot = int(uvVal.x / uDiv);
	int vSlot = int(uvVal.y / vDiv);
	bool noObj = true;
	for(int i = -size; i <= size; ++i){
		for(int j = -size; j <= size; ++j){
			if((i == 0) && (j == 0)){
				continue;
			}
			int currID = texture(samplerAlbedo, inUV + vec2(tex_offset.x * i, tex_offset.y * j)).r;
			if(abs(currID - objectID) > 0){
				return vec3(1);
			}else{
				if((currID == -1) || (objectID) == -1){
					continue;
				}
				vec2 currUVVal = vec2(texture(samplerposition, inUV + vec2(tex_offset.x * i, tex_offset.y * j)).a, texture(samplerNormal, inUV + vec2(tex_offset.x * i, tex_offset.y * j)).a);
				int currUSlot = int(currUVVal.x / uDiv);
				int currVSlot = int(currUVVal.y / vDiv);
				if((abs(currUSlot - uSlot) > 0) || (abs(currVSlot - vSlot) > 0)){
					return vec3(1);
				}
			}
		}
	}
	return vec3(0);
}

vec3 case12(int size, vec2 inUV, vec2 tex_offset){
	//edgePure
	ivec4 center_edge =  texture(samplerEdge, inUV).rgba;
	vec4 center_edge2 =  texture(samplerEdge2, inUV).rgba;

	if(center_edge.a == 2){
		//return vec3(1, 0, 0);
		return vec3(center_edge2.rgb);
	}else if(center_edge.a > 2){
		vec3 result = vec3((center_edge.a - 3) * 540.f / 10000.f);
		return clamp(result, vec3(0), vec3(1));
	}
	if(center_edge.b == 1){
		return vec3(1);
	}

	return vec3(0);

}

vec3 case13(int size, vec2 inUV, vec2 tex_offset){
	//LockedEdge
	vec3 center_edge =  texture(samplerLockedEdge, inUV).rgb;
	return center_edge;

}

vec3 case14(int size, vec2 center_uv, vec2 tex_offset){
	vec2 screen_uv = center_uv * 2 -1;
	screen_uv.y = - screen_uv.y;
	vec2 screen_length = vec2(pushConsts.screenHalfLengthX, pushConsts.screenHalfLengthY);
	vec3 screen_normal = mat3(ubo.camViewTr) * -normalize(vec3(screen_length * screen_uv, -1));
	int faceID = texture(samplerAlbedo, center_uv).g;  
	int objectID = texture(samplerAlbedo, center_uv).r;
	vec3 normalTex = texture(samplerNormal, center_uv).rgb;
	return normalTex;
	if(objectID > -1){
		int faceIdx = faceInfos[objectID] + faceID;
		//vec3 viewNor = mat3(ubo.camView) * vec3(faceNor[faceIdx]);
		//vec3 viewDirection = vec3(ubo.camView[0][2], ubo.camView[1][2], ubo.camView[2][2]);
		return vec3(dot(screen_normal, normalTex));
	}
	return vec3(0);
}

vec3 case15(int size, vec2 center_uv, vec2 tex_offset){
	vec2 screen_uv = center_uv * 2 -1;
	screen_uv.y = - screen_uv.y;
	vec2 screen_length = vec2(pushConsts.screenHalfLengthX, pushConsts.screenHalfLengthY);
	vec3 screen_normal = mat3(ubo.camViewTr) * -normalize(vec3(screen_length * screen_uv, -1));
	int faceID = texture(samplerAlbedo, center_uv).g;  
	int objectID = texture(samplerAlbedo, center_uv).r;
	if(objectID > -1){
		int faceIdx = faceInfos[objectID] + faceID;
		//vec3 viewNor = mat3(ubo.camView) * vec3(faceNor[faceIdx]);
		//vec3 viewDirection = vec3(ubo.camView[0][2], ubo.camView[1][2], ubo.camView[2][2]);
		return vec3(dot(screen_normal, vec3(faceNor[faceIdx])));
	}
	return vec3(0);
}


void main() 
{
	// Get G-Buffer values
	vec3 pos = texture(samplerposition, inUV).rgb;
	vec3 normal = texture(samplerNormal, inUV).rgb;
	vec2 uv = vec2(texture(samplerposition, inUV).a, texture(samplerNormal, inUV).a);
	vec4 edge = texture(samplerEdge, inUV).rgba;
	//vec4 albedo = texture(samplerAlbedo, inUV);
	//float objectID = albedo.r;
	int objectID = texture(samplerAlbedo, inUV).r;  
	int faceID = texture(samplerAlbedo, inUV).g; 
	vec2 tex_offset = 1.f / textureSize(samplerAlbedo, 0); // gets size of single texel
	int size = ubo.singleStride;

	vec3 finalCol = vec3(0);
	vec2 uv1 = inUV - (tex_offset / 2.f);
	vec2 uv2 = uv1 + vec2(tex_offset.x, 0);
	vec2 uv3 = uv1 + vec2(0, tex_offset.y);
	vec2 uv4 = uv1 + tex_offset;

	// Debug display
	if (ubo.displayDebugTarget > 0) {
		switch (ubo.displayDebugTarget) {
			case 1: 
				//outFragcolor.rgb = fragPos;
//				vec3 color = vec3(1,0,0);
//				float intensity = dot(normal,fragPos);
//				if (intensity > 0.98)
//					color *= 1.5;
//				else if  (intensity > 0.9)
//					color *= 1.0;
//				else if (intensity > 0.5)
//					color *= 0.6;
//				else if (intensity > 0.25)
//					color *= 0.4;
//				else
//					color *= 0.2;
//				// Desaturate a bit
//				outFragcolor.rgb = vec3(mix(color, vec3(dot(vec3(0.2126,0.7152,0.0722), color)), 0.1));	
//				//outFragcolor.rgb = fragPos;	
//				outFragcolor.rgb = vec3(fragPos.z);
				outFragcolor.rgb = pos;
				break;
			case 2: 
				outFragcolor.rgb = normal;
				//outFragcolor.rgb = vec3(pushConsts.max_neighbor/100.f);
				//outFragcolor.rgb = vec3(inUV.x, inUV.y, 0);
				break;
			case 3: 
				//LineWire
//				for(int i = -size; i <= size; ++i){
//					for(int j = -size; j <= size; ++j){
//						if((i == 0) && (j == 0)){
//							continue;
//						}
//						int currID = texture(samplerAlbedo, inUV + vec2(tex_offset.x * i, tex_offset.y * j)).r;
//						if(abs(currID - objectID) > 0){
//							outFragcolor = vec4(1);
//							return;
//						}else{
//							int currFaceID = texture(samplerAlbedo, inUV + vec2(tex_offset.x * i, tex_offset.y * j)).g;
//							if(abs(currFaceID - faceID) > 0){
//								outFragcolor = vec4(1);
//								return;
//							}
//						}
//					}
//				}
//				outFragcolor.rgb = vec3(0);
				finalCol += case3(size, uv1, tex_offset);
				finalCol += case3(size, uv2, tex_offset);
				finalCol += case3(size, uv3, tex_offset);
				finalCol += case3(size, uv4, tex_offset);
				outFragcolor.rgb = finalCol / 4.f;
				break;
			case 4: 
				//LineObj
//				for(int i = -size; i <= size; ++i){
//					for(int j = -size; j <= size; ++j){
//						if((i == 0) && (j == 0)){
//							continue;
//						}
//						int currID = texture(samplerAlbedo, inUV + vec2(tex_offset.x * i, tex_offset.y * j)).r;
//						if(abs(currID - objectID) > 0){
//							outFragcolor = vec4(1);
//							return;
//						}
//					}
//				}
//				outFragcolor.rgb = vec3(0);
				finalCol += case4(size, uv1, tex_offset);
				finalCol += case4(size, uv2, tex_offset);
				finalCol += case4(size, uv3, tex_offset);
				finalCol += case4(size, uv4, tex_offset);
				outFragcolor.rgb = finalCol / 4.f;
				break;
			case 5:
				//LineFace
//			    bool inObj = false;
//				int neighborFaceCnt = 0;
//				int faceIdxStart = 0;
//				if(objectID > -1){
//					inObj = true;
//				};
//				if(inObj){
//					int objIdx = faceInfos[objectID];
//					faceIdxStart = objIdx + pushConsts.max_neighbor * faceID;
//					for(int i = 0; i < pushConsts.max_neighbor; ++i){
//						if(faceData[faceIdxStart + i] > -1){
//							++neighborFaceCnt;
//						}else{
//							break;
//						}
//					}
//				};
//				for(int i = -size; i <= size; ++i){
//					for(int j = -size; j <= size; ++j){
//						if((i == 0) && (j == 0)){
//							continue;
//						}
//						int currID = texture(samplerAlbedo, inUV + vec2(tex_offset.x * i, tex_offset.y * j)).r;
//						if(abs(currID - objectID) > 0){
//							outFragcolor = vec4(1);
//							return;
//						}
//					}
//				}
//				if(inObj){
//					vec2 low_left_uv = inUV + vec2(-tex_offset.x * size, -tex_offset.y * size);
//					bool shouldColor = sampleOn3x3Grid(low_left_uv, tex_offset, size, neighborFaceCnt, faceIdxStart, faceID);
//					if(shouldColor){
//						outFragcolor = vec4(1);
//						return;
//					}
//				}
//				outFragcolor.rgb = vec3(0);
				finalCol += case5(size, uv1, tex_offset);
				finalCol += case5(size, uv2, tex_offset);
				finalCol += case5(size, uv3, tex_offset);
				finalCol += case5(size, uv4, tex_offset);
				outFragcolor.rgb = finalCol / 4.f;
				break;
			case 6:
				//LineFaceNor
				finalCol += case6(size, uv1, tex_offset);
				finalCol += case6(size, uv2, tex_offset);
				finalCol += case6(size, uv3, tex_offset);
				finalCol += case6(size, uv4, tex_offset);
				outFragcolor.rgb = finalCol / 4.f;
				break;
			case 7:
				finalCol += case7(size, uv1, tex_offset);
				finalCol += case7(size, uv2, tex_offset);
				finalCol += case7(size, uv3, tex_offset);
				finalCol += case7(size, uv4, tex_offset);
				outFragcolor.rgb = finalCol / 4.f;
				break;
			case 8:
				finalCol += case8(size, uv1, tex_offset);
				finalCol += case8(size, uv2, tex_offset);
				finalCol += case8(size, uv3, tex_offset);
				finalCol += case8(size, uv4, tex_offset);
				outFragcolor.rgb = finalCol / 4.f;
				break;
			case 9:
				//isoparametric Line
				finalCol += case9(size, uv1, tex_offset, ubo.uFactor, ubo.vFactor);
				finalCol += case9(size, uv2, tex_offset, ubo.uFactor, ubo.vFactor);
				finalCol += case9(size, uv3, tex_offset, ubo.uFactor, ubo.vFactor);
				finalCol += case9(size, uv4, tex_offset, ubo.uFactor, ubo.vFactor);
				outFragcolor.rgb = finalCol / 4.f;
				break;
			case 10:
				//Edge
				finalCol += case10(size, uv1, tex_offset);
				finalCol += case10(size, uv2, tex_offset);
				finalCol += case10(size, uv3, tex_offset);
				finalCol += case10(size, uv4, tex_offset);
				outFragcolor.rgb = finalCol / 4.f;
				break;
			case 11:
				//Edge UV
				finalCol += case11(size, uv1, tex_offset, ubo.uFactor, ubo.vFactor);
				finalCol += case11(size, uv2, tex_offset, ubo.uFactor, ubo.vFactor);
				finalCol += case11(size, uv3, tex_offset, ubo.uFactor, ubo.vFactor);
				finalCol += case11(size, uv4, tex_offset, ubo.uFactor, ubo.vFactor);
				outFragcolor.rgb = finalCol / 4.f;
				break;
			case 12:
				//Edge Pure
				finalCol += case12(size, uv1, tex_offset);
				finalCol += case12(size, uv2, tex_offset);
				finalCol += case12(size, uv3, tex_offset);
				finalCol += case12(size, uv4, tex_offset);
				outFragcolor.rgb = finalCol / 4.f;
				ivec3 center_edge =  texture(samplerEdge, inUV/2.f).rgb;
//				if(center_edge.b == 1){
//					outFragcolor.rgb = vec3(1);
//				}else{
//					outFragcolor.rgb = vec3(0);
//				}
				break;
			case 13:
				finalCol += case13(size, uv1, tex_offset);
				finalCol += case13(size, uv2, tex_offset);
				finalCol += case13(size, uv3, tex_offset);
				finalCol += case13(size, uv4, tex_offset);
				outFragcolor.rgb = finalCol / 4.f;
				break;
			case 14:
				finalCol += case14(size, uv1, tex_offset);
				finalCol += case14(size, uv2, tex_offset);
				finalCol += case14(size, uv3, tex_offset);
				finalCol += case14(size, uv4, tex_offset);
				outFragcolor.rgb = finalCol / 4.f;
				break;
			case 15:
				finalCol += case15(size, uv1, tex_offset);
				finalCol += case15(size, uv2, tex_offset);
				finalCol += case15(size, uv3, tex_offset);
				finalCol += case15(size, uv4, tex_offset);
				outFragcolor.rgb = finalCol / 4.f;
				break;
		}		
		outFragcolor.a = 1.0;
		return;
	}

	// Render-target composition

	#define lightCount 6
	#define ambient 0.0
	
	// Ambient part
//	vec3 fragcolor  = albedo.rgb * ambient;
//	
//	for(int i = 0; i < lightCount; ++i)
//	{
//		// Vector to light
//		vec3 L = ubo.lights[i].position.xyz - fragPos;
//		// Distance from light to fragment position
//		float dist = length(L);
//
//		// Viewer to fragment
//		vec3 V = ubo.viewPos.xyz - fragPos;
//		V = normalize(V);
//		
//		//if(dist < ubo.lights[i].radius)
//		{
//			// Light to fragment
//			L = normalize(L);
//
//			// Attenuation
//			float atten = ubo.lights[i].radius / (pow(dist, 2.0) + 1.0);
//
//			// Diffuse part
//			vec3 N = normalize(normal);
//			float NdotL = max(0.0, dot(N, L));
//			vec3 diff = ubo.lights[i].color * albedo.rgb * NdotL * atten;
//
//			// Specular part
//			// Specular map values are stored in alpha of albedo mrt
//			vec3 R = reflect(-L, N);
//			float NdotR = max(0.0, dot(R, V));
//			vec3 spec = ubo.lights[i].color * albedo.a * pow(NdotR, 16.0) * atten;
//
//			fragcolor += diff + spec;	
//		}	
//	}    	
   
  outFragcolor = vec4(vec3(0), 1.0);	
}