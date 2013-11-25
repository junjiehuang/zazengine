#version 400 core

in vec4 ex_position;
in vec4 ex_normal;
in vec2 ex_texCoord;
in vec4 ex_tangent;
in vec4 ex_biTangent;

uniform sampler2D DiffuseTexture;
uniform sampler2D SpecularTexture;
uniform sampler2D NormalMap;

// OPTIMIZE: remove bindfraglocation in init
layout( location = 0 ) out vec4 out_diffuse;
layout( location = 1 ) out vec4 out_normal;
layout( location = 2 ) out vec4 out_position;
layout( location = 3 ) out vec4 out_tangent;
layout( location = 4 ) out vec4 out_biTangent;

layout( shared ) uniform MaterialUniforms
{
	vec4 config; 			// x=materialtype, y=apply texture
	vec4 color;     		// base-color of material
} Material;

void main()
{
	out_position = ex_position;
	out_tangent = ex_tangent;
	out_biTangent = ex_biTangent;

	// store materialtype in diffuse-component alpha-channel
	out_diffuse.a = Material.config.x;

	// LAMBERT & PHONG Material-Types
	if ( 3 > Material.config.x )
	{
		// store base-color of material
		out_diffuse.rgb = Material.color.rgb;

		// apply texture
		if ( 1.0 == Material.config.y )
		{
			out_diffuse.rgb += texture( DiffuseTexture, ex_texCoord ).rgb;
		}

		out_normal.xyz = ex_normal.xyz;

		// set alpha component to 0 => RFU
		out_normal.a = 0.0;
		out_tangent.a = 0.0;
	}
	// DOOM3 Material-Type
	else if ( 3 == Material.config.x )
	{
		// store base-color of material
		out_diffuse.rgb = texture( DiffuseTexture, ex_texCoord ).rgb;
		// store normals of normal-map
		out_normal.rgb = texture( NormalMap, ex_texCoord ).rgb;

		vec3 specular = texture( SpecularTexture, ex_texCoord ).rgb;

		// store specular material in the 3 unused alpha-channels
		out_normal.a = specular.r;
		out_tangent.a = specular.g;
		out_biTangent.a = specular.b;
	}
}