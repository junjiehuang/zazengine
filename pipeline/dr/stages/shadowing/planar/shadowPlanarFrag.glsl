#version 400 core

layout( location = 0 ) out float fragDepth;

void main()
{
	fragDepth = gl_FragCoord.z;
}