#version 450

layout (binding = 1) uniform sampler2D samplerColor;

layout (binding = 0) uniform UBO 
{
	float blurScale;
	float blurStrength;
} ubo;

layout (constant_id = 0) const int blurdirection = 0;

layout (location = 0) in vec2 inUV;

layout (location = 0) out vec4 outFragColor;

void main() 
{
	float weight[5];
	weight[0] = 0.227027;
	weight[1] = 0.1945946;
	weight[2] = 0.1216216;
	weight[3] = 0.054054;
	weight[4] = 0.016216;

	vec2 tex_offset = 0.25f / textureSize(samplerColor, 0); // gets size of single texel
	vec3 currCol = texture(samplerColor, inUV).rgb; // current fragment's contribution
	vec3 result = vec3(0.f);
	for(int i = 1; i < 5; ++i)
	{
		if (blurdirection == 1)
		{
			// H
//			result += texture(samplerColor, inUV + vec2(tex_offset.x * i, 0.0)).rgb * weight[i] * ubo.blurStrength;
//			result += texture(samplerColor, inUV - vec2(tex_offset.x * i, 0.0)).rgb * weight[i] * ubo.blurStrength;
			//左右
			if (length(texture(samplerColor, inUV + vec2(tex_offset.x * i, 0.0)).rgb - currCol) > 0.5f){
				result = vec3(1.f);
				break;
			}
			if (length(texture(samplerColor, inUV - vec2(tex_offset.x * i, 0.0)).rgb - currCol) > 0.5f){
				result = vec3(1.f);
				break;
			}
			//左上
			if (length(texture(samplerColor, inUV + vec2(-tex_offset.x * i, tex_offset.y * i)).rgb - currCol) > 0.5f){
				result = vec3(1.f);
				break;
			}
			//右下
			if (length(texture(samplerColor, inUV + vec2(tex_offset.x * i, -tex_offset.y * i)).rgb - currCol) > 0.5f){
				result = vec3(1.f);
				break;
			}
		}
		else
		{
			// V
//			result += texture(samplerColor, inUV + vec2(0.0, tex_offset.y * i)).rgb * weight[i] * ubo.blurStrength;
//			result += texture(samplerColor, inUV - vec2(0.0, tex_offset.y * i)).rgb * weight[i] * ubo.blurStrength;
			//上下
			if (length(texture(samplerColor, inUV + vec2(0.0, tex_offset.y * i)).rgb - currCol) > 0.5f){
				result = vec3(1.f);
				break;
			}
			if (length(texture(samplerColor, inUV - vec2(0.0, tex_offset.y * i)).rgb - currCol) > 0.5f){
				result = vec3(1.f);
				break;
			}
			//右上
			if (length(texture(samplerColor, inUV + vec2(tex_offset.x * i, tex_offset.y * i)).rgb - currCol) > 0.5f){
				result = vec3(1.f);
				break;
			}
			//左下
			if (length(texture(samplerColor, inUV - vec2(tex_offset.x * i, tex_offset.y * i)).rgb - currCol) > 0.5f){
				result = vec3(1.f);
				break;
			}

		}
	}


	if(ubo.blurScale == 1.0){
		outFragColor = vec4(currCol, 1.0);
	}else if(ubo.blurScale < 1.0){
		outFragColor = vec4(vec3(0.f), 1.0);
	}else{
		outFragColor = vec4(result, 1.0);
	};


	//outFragColor = vec4(result, 1.0);
}