#version 450

layout (binding = 1) uniform sampler2D samplerposition;
layout (binding = 2) uniform sampler2D samplerNormal;
layout (binding = 3) uniform isampler2D samplerAlbedo;

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
	int displayDebugTarget;
	int singleStride;
} ubo;

layout (binding = 5) buffer FaceInfo
{
    int faceInfos[];
};

layout (binding = 6) buffer FaceData
{
    int faceData[];
};

layout(push_constant) uniform PushConsts {
	int max_neighbor;
} pushConsts;


bool grid_3x3(vec2 low_left_uv, vec2 tex_offset, int neighborFaceCnt, int faceIdxStart, int faceID){
	bool shouldColor = false;
	for(int i = 0; i < 3; ++i){
		for(int j = 0; j < 3; ++j){
			if((i == 1) && (j == 1)){
				continue;
			}
			int currFaceID = texture(samplerAlbedo, low_left_uv + vec2(tex_offset.x * i, tex_offset.y * j)).g;
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

bool sampleOn3x3Grid(vec2 low_left_uv, vec2 tex_offset, int stride, int neighborFaceCnt, int faceIdxStart, int faceID){
	int move = 2 * stride - 1;
	for(int i = 0; i < move; ++i){
		for(int j = 0; j < move; ++j){
			vec2 currUV = low_left_uv + vec2(tex_offset.x * i, tex_offset.y * j);
			if(grid_3x3(currUV, tex_offset, neighborFaceCnt, faceIdxStart, faceID)){
				return true;
			}
		}
	}
	return false;
}

vec3 case3(int objectID, int size, int faceID, vec2 inUV, vec2 tex_offset){
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

vec3 case4(int objectID, int size, vec2 inUV, vec2 tex_offset){
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

vec3 case5(int objectID, int size, int faceID, vec2 inUV, vec2 tex_offset){
	bool inObj = false;
	int neighborFaceCnt = 0;
	int faceIdxStart = 0;
	if(objectID > -1){
		inObj = true;
	};
	if(inObj){
		int objIdx = faceInfos[objectID];
		faceIdxStart = objIdx + pushConsts.max_neighbor * faceID;
		for(int i = 0; i < pushConsts.max_neighbor; ++i){
			if(faceData[faceIdxStart + i] > -1){
				++neighborFaceCnt;
			}else{
				break;
			}
		}
	};
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
	if(inObj){
		vec2 low_left_uv = inUV + vec2(-tex_offset.x * size, -tex_offset.y * size);
		bool shouldColor = sampleOn3x3Grid(low_left_uv, tex_offset, size, neighborFaceCnt, faceIdxStart, faceID);
		if(shouldColor){
			return vec3(1);
		}
	}
	return vec3(0);
}

void main() 
{
	// Get G-Buffer values
	vec3 fragPos = texture(samplerposition, inUV).rgb;
	vec3 normal = texture(samplerNormal, inUV).rgb;
	//vec4 albedo = texture(samplerAlbedo, inUV);
	//float objectID = albedo.r;
	int objectID = texture(samplerAlbedo, inUV).r;  
	int faceID = texture(samplerAlbedo, inUV).g;  
	vec2 tex_offset = 1.f / textureSize(samplerAlbedo, 0); // gets size of single texel
	int size = ubo.singleStride;
	// Debug display
	if (ubo.displayDebugTarget > 0) {
		switch (ubo.displayDebugTarget) {
			case 1: 
				//outFragcolor.rgb = fragPos;
				vec3 color = vec3(1,0,0);
				float intensity = dot(normal,fragPos);
				if (intensity > 0.98)
					color *= 1.5;
				else if  (intensity > 0.9)
					color *= 1.0;
				else if (intensity > 0.5)
					color *= 0.6;
				else if (intensity > 0.25)
					color *= 0.4;
				else
					color *= 0.2;
				// Desaturate a bit
				outFragcolor.rgb = vec3(mix(color, vec3(dot(vec3(0.2126,0.7152,0.0722), color)), 0.1));	
				//outFragcolor.rgb = fragPos;	
				outFragcolor.rgb = vec3(fragPos.z);
				break;
			case 2: 
				//outFragcolor.rgb = normal;
				//outFragcolor.rgb = vec3(pushConsts.max_neighbor/100.f);
				outFragcolor.rgb = vec3(inUV.x, inUV.y, 0);
				break;
			case 3: 
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
				vec3 finalCol = vec3(0);
				vec2 uv1 = inUV - vec2(tex_offset / 2.f);
				vec2 uv2 = uv1 + vec2(tex_offset.x, 0);
				vec2 uv3 = uv1 + vec2(0, tex_offset.y);
				vec2 uv4 = uv1 + vec2(tex_offset.x, tex_offset.y);
				finalCol += case3(objectID, size, faceID, uv1, tex_offset);
				finalCol += case3(objectID, size, faceID, uv2, tex_offset);
				finalCol += case3(objectID, size, faceID, uv3, tex_offset);
				finalCol += case3(objectID, size, faceID, uv4, tex_offset);
				outFragcolor.rgb = finalCol / 4.f;
				break;
			case 4: 
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
				finalCol = vec3(0);
				uv1 = inUV - vec2(tex_offset / 2.f);
				uv2 = uv1 + vec2(tex_offset.x, 0);
				uv3 = uv1 + vec2(0, tex_offset.y);
				uv4 = uv1 + vec2(tex_offset.x, tex_offset.y);
				finalCol += case4(objectID, size, uv1, tex_offset);
				finalCol += case4(objectID, size, uv2, tex_offset);
				finalCol += case4(objectID, size, uv3, tex_offset);
				finalCol += case4(objectID, size, uv4, tex_offset);
				outFragcolor.rgb = finalCol / 4.f;
				break;
			case 5:
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
				finalCol = vec3(0);
				uv1 = inUV - vec2(tex_offset / 2.f);
				uv2 = uv1 + vec2(tex_offset.x, 0);
				uv3 = uv1 + vec2(0, tex_offset.y);
				uv4 = uv1 + vec2(tex_offset.x, tex_offset.y);
				finalCol += case5(objectID, size, faceID, uv1, tex_offset);
				finalCol += case5(objectID, size, faceID, uv2, tex_offset);
				finalCol += case5(objectID, size, faceID, uv3, tex_offset);
				finalCol += case5(objectID, size, faceID, uv4, tex_offset);
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

	vec3 color = vec3(1,0,0);
	float factor = abs(dot(vec3(ubo.viewPos) - fragPos, normal));
   
  outFragcolor = vec4(vec3(0), 1.0);	
}