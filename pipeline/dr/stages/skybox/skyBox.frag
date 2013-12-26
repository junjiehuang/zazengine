#version 430 core

in vec3 ex_texCoord;

layout( location = 0 ) out vec4 out_diffuse;

layout( location = 20 ) uniform samplerCube SkyBoxCubeMap;

void main()
{
	// sky-box has diffuse-color only
	out_diffuse.rgb = texture( SkyBoxCubeMap, ex_texCoord ).rgb;
	out_diffuse.a = 1.0;
}