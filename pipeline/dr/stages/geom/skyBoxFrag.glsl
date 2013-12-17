#version 400 core

in vec3 ex_texCoord;

uniform samplerCube SkyBoxCubeMap;

layout( location = 0 ) out vec4 out_diffuse;

void main()
{
	// sky-box has diffuse-color only
	out_diffuse.rgb = texture( SkyBoxCubeMap, ex_texCoord ).rgb;
	out_diffuse.a = 1.0;
}