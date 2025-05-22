#version 450
layout (location = 0) out vec4 outPosition;

void main() 
{
	outPosition = vec4(1,1,1,1);
	//outPosition = vec4(inFaceNor, 1);
}