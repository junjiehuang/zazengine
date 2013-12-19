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
layout( location = 2 ) out vec4 out_tangent;
layout( location = 3 ) out vec4 out_biTangent;

layout( shared ) uniform MaterialUniforms
{
	vec2 config;	// x holds material-type (1.0 lambert, 2.0 phong, 3.0 doom3)
	vec4 color;		// base-color of material
} Material;

// encodes a vec3 direction-vector into its vec2 representation to save one channel in the render-target 
// will be reconstructed in lighting-stage
vec2 encodeDirection( vec3 n )
{
	// all encodings need normalized normals
	n = normalize( n );

	/* UNIT-LENGTH ENCODING
	return n.xy;
	*/

	/* SPHEREMAP-TRANSFORM ENCODING USED IN CRYENGINE 3.0
    vec2 enc = normalize( n.xy ) * ( sqrt( -n.z * 0.5 + 0.5 ) );
    enc = enc * 0.5 + 0.5;
    return enc;
	*/
	
	// SPHEREMAP-TRANSFORM ENCODING USING LAMBERT AZIMUTHAL EQUAL-AREA PROJECTION
	float f = sqrt( 8 * n.z + 8 );
    return n.xy / f + 0.5;
}

subroutine void storeMaterialProperties();

// LAMBERT & PHONG Material-Types
subroutine ( storeMaterialProperties ) void classicMaterial()
{
	// store materialtype in diffuse-component alpha-channel
	out_diffuse.a = Material.config.x;

	// store base-color of material
	out_diffuse.rgb = Material.color.rgb;

	out_normal.rg = encodeDirection( ex_normal.xyz );

	// RFU
	out_normal.b = 0.0;
	out_normal.a = 0.0;
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
	// store normal of normal-map - scaling is done in lighting-shader
	out_normal.rg = encodeDirection( texture( NormalMap, ex_texCoord).rgb );
	// fetch specular and store it in alpha-channels of directions
	vec3 specular = texture( SpecularTexture, ex_texCoord ).rgb;
	// store specular material in the 3 unused alpha-channels
	out_normal.a = specular.r;
	out_tangent.a = specular.g;
	out_biTangent.a = specular.b;

	// RFU
	out_normal.b = 0.0;
}

subroutine uniform storeMaterialProperties storeMaterialPropertiesSelection;

void main()
{
	// encode both tangent and bi-tanget to save one channel each
	// works the same way as normals because tangent&bitangent are directions
	out_tangent.xy = encodeDirection( ex_tangent.xyz );
	out_biTangent.xy = encodeDirection( ex_biTangent.xyz );

	// RFU
	out_tangent.z = 0.0;
	out_biTangent.z = 0.0;

	// subroutine-call based on the selection from application
	storeMaterialPropertiesSelection();
}
