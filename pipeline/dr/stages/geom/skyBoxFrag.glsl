#version 330 core

in vec4 ex_position;
in vec4 ex_normal;
in vec2 ex_texCoord;

uniform sampler2D DiffuseTexture;

// OPTIMIZE: remove bindfraglocation in init
layout( location = 0 ) out vec4 out_diffuse;

void main()
{
	out_diffuse.rgb = texture( DiffuseTexture, ex_texCoord ).rgb;
	// store materialtype in diffuse-component alpha-channel
	out_diffuse.a = 255;
}