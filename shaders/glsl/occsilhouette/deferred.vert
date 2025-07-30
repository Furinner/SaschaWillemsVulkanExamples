#version 450

layout (location = 0) out vec2 outUV;

void main() 
{
//	outUV = vec2((gl_VertexIndex << 1) & 2, gl_VertexIndex & 2);
//	gl_Position = vec4(outUV * 2.0f - 1.0f, 0.0f, 1.0f);
	outUV = vec2(0, -1);
	float x = -1;
	float y = -1;
	if(gl_VertexIndex == 0){
		y = 3;
	}else if(gl_VertexIndex == 1){
		outUV.y = 1;
	}else{
		x=3;
		outUV = vec2(2,1);
	}
	gl_Position = vec4(x, y, 0.0f, 1.0f);
}
