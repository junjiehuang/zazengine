#version 400 core

in vec4 ex_position;
in vec4 ex_normal;
in vec2 ex_texCoord;
in vec4 ex_tangent;
in vec4 ex_biTangent;

uniform sampler2D DiffuseTexture;
uniform sampler2D SpecularTexture;
uniform sampler2D NormalMap;

layout( location = 0 ) out vec4 out_diffuse;
layout( location = 1 ) out vec4 out_normal;
layout( location = 2 ) out vec4 out_position;
layout( location = 3 ) out vec4 out_tangent;
layout( location = 4 ) out vec4 out_biTangent;

layout( shared ) uniform MaterialUniforms
{
	vec2 config;	// x holds material-type (1.0 lambert, 2.0 phong, 3.0 doom3)
	vec4 color;		// base-color of material
} Material;

subroutine void storeMaterialProperties();

// LAMBERT & PHONG Material-Types
subroutine ( storeMaterialProperties ) void classicMaterial()
{
	// store materialtype in diffuse-component alpha-channel
	out_diffuse.a = Material.config.x;

	// store base-color of material
	out_diffuse.rgb = Material.color.rgb;

	out_normal.xyz = ex_normal.xyz;

	// set alpha component to 0 => RFU
	out_normal.a = 0.0;
	out_tangent.a = 0.0;
}

// LAMBERT & PHONG Material-Types
subroutine ( storeMaterialProperties ) void classicMaterialTextured()
{
	// set classic material first
	classicMaterial();
	// then just add diffuse-texture color
	out_diffuse.rgb += texture( DiffuseTexture, ex_texCoord ).rgb;
}

subroutine ( storeMaterialProperties ) void doom3Material()
{
	// hard-code doom3 material to 3.0
	out_diffuse.a = 3.0;

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

subroutine uniform storeMaterialProperties storeMaterialPropertiesSelection;

void main()
{
	// just write through without any change depending on the material
	out_position = ex_position;
	out_tangent = ex_tangent;
	out_biTangent = ex_biTangent;

	// subroutine-call based on the selection from application
	storeMaterialPropertiesSelection();
}
