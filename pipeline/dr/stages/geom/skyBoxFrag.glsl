#version 400 core

in vec3 ex_texCoord;

uniform samplerCube SkyBoxCubeMap;

// OPTIMIZE: remove bindfraglocation in init
layout( location = 0 ) out vec4 out_diffuse;

void main()
{
	// sky-box does diffuse-color only
	out_diffuse.rgb = texture( SkyBoxCubeMap, ex_texCoord ).rgb;
	// store 0 as material-type for sky-box
	out_diffuse.a = 0.0;
}