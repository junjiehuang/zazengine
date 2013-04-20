#version 330 core

in vec4 ex_position;
in vec4 ex_normal;
in vec2 ex_texCoord;

uniform sampler2D DiffuseTexture;

// OPTIMIZE: remove bindfraglocation in init
layout( location = 0 ) out vec4 out_diffuse;
layout( location = 1 ) out vec4 out_normal;
layout( location = 2 ) out vec4 out_position;

layout( shared ) uniform MaterialUniforms
{
	vec4 config; 			// x=materialtype, y=apply texture
	vec4 color;     		// base-color of material
} Material;

void main()
{
	// store base-color of material
	out_diffuse.rgb = Material.color.rgb;
	// store materialtype in diffuse-component alpha-channel
	out_diffuse.a = Material.config.x / 255;

	// use diffuse-texture for color
	if ( 1.0 == Material.config.y )
	{
		out_diffuse.rgb += texture( DiffuseTexture, ex_texCoord ).rgb;
	}

    // set alpha component of normal to 0
	out_normal.xyz = ex_normal.xyz;
	out_normal.a = 0.0;

    out_position = ex_position;
}