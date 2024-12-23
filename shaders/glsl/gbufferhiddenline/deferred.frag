#version 450

layout (binding = 1) uniform sampler2D samplerposition;
layout (binding = 2) uniform sampler2D samplerNormal;
layout (binding = 3) uniform sampler2D samplerAlbedo;

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

void main() 
{
	// Get G-Buffer values
	vec3 fragPos = texture(samplerposition, inUV).rgb;
	vec3 normal = texture(samplerNormal, inUV).rgb;
	vec4 albedo = texture(samplerAlbedo, inUV);
	float objectID = albedo.r;
	vec2 tex_offset = 1.f / textureSize(samplerAlbedo, 0); // gets size of single texel
	// Debug display
	if (ubo.displayDebugTarget > 0) {
		switch (ubo.displayDebugTarget) {
			case 1: 
				outFragcolor.rgb = fragPos;
				break;
			case 2: 
				outFragcolor.rgb = normal;
				break;
			case 3: 
				//top left
				if(abs(texture(samplerAlbedo, inUV + vec2(-tex_offset.x, tex_offset.y)).r - objectID) > 0.1f){
					outFragcolor = vec4(1);
					return;
				}
				//top
				if(abs(texture(samplerAlbedo, inUV + vec2(tex_offset.x, tex_offset.y)).r - objectID) > 0.1f){
					outFragcolor = vec4(1);
					return;
				}
				//top right
				if(abs(texture(samplerAlbedo, inUV + vec2(tex_offset.x, tex_offset.y)).r - objectID) > 0.1f){
					outFragcolor = vec4(1);
					return;
				}
				//left
				if(abs(texture(samplerAlbedo, inUV + vec2(-tex_offset.x, 0)).r - objectID) > 0.1f){
					outFragcolor = vec4(1);
					return;
				}
				//right
				if(abs(texture(samplerAlbedo, inUV + vec2(tex_offset.x, 0)).r - objectID) > 0.1f){
					outFragcolor = vec4(1);
					return;
				}
				//down left
				if(abs(texture(samplerAlbedo, inUV + vec2(-tex_offset.x, -tex_offset.y)).r - objectID) > 0.1f){
					outFragcolor = vec4(1);
					return;
				}
				//down
				if(abs(texture(samplerAlbedo, inUV + vec2(0, -tex_offset.y)).r - objectID) > 0.1f){
					outFragcolor = vec4(1);
					return;
				}
				//down right
				if(abs(texture(samplerAlbedo, inUV + vec2(tex_offset.x, -tex_offset.y)).r - objectID) > 0.1f){
					outFragcolor = vec4(1);
					return;
				}
				outFragcolor.rgb = vec3(0);
				break;
			case 4: 
				int size = ubo.singleStride;
				for(int i = -size; i <= size; ++i){
					for(int j = -size; j <= size; ++j){
						if((i == 0) && (j == 0)){
							continue;
						}
						float currID = texture(samplerAlbedo, inUV + vec2(tex_offset.x * i, tex_offset.y * j)).r;
						if(abs(currID - objectID) > 0.1f){
							outFragcolor = vec4(1);
							return;
						}
					}
				}
				outFragcolor.rgb = vec3(0);
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