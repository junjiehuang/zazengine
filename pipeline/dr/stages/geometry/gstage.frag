#version 430 core

// defines the input-interface block from the vertex-shader
in IN_OUT_BLOCK
{
	vec4 normal;
	vec2 texCoord;
	vec4 tangent;
	vec4 biTangent;
} IN_OUT;

// write to 4 render-targetsdepth and stencil will be written implicitly
layout( location = 0 ) out vec4 out_diffuse;
layout( location = 1 ) out vec4 out_normal;
layout( location = 2 ) out vec4 out_tangent;
layout( location = 3 ) out vec4 out_biTangent;

// THE MATERIAL-CONFIGURATION OF THE MATERIAL OF THE CURRENT MESH
layout( shared ) uniform GStageMaterialUniforms
{
	// x holds material-type (1.0 lambert, 2.0 phong, 3.0 doom3)
	vec2 config;
	// base-color of material
	vec4 color;		
} GStageMaterial;

// the textures necessary to realize all materials
layout( binding = 0 ) uniform sampler2D DiffuseTexture;
layout( binding = 1 ) uniform sampler2D SpecularTexture;
layout( binding = 2 ) uniform sampler2D NormalMap;

// prototype of material-subroutine - application will select according to material-type
subroutine void storeMaterialProperties();
// subroutine selection-uniform
subroutine uniform storeMaterialProperties storeMaterialPropertiesSelection;

// encodes a vec3 direction-vector into its vec2 representation to save one channel in the render-target 
// will be reconstructed in lighting-stage
vec2 encodeDirection( vec3 dir )
{
	// encoding need normalized direction
	dir = normalize( dir );
	
	/* SPHEREMAP-TRANSFORM ENCODING USING LAMBERT AZIMUTHAL EQUAL-AREA PROJECTION */
	float f = sqrt( 8 * dir.z + 8 );
    return dir.xy / f + 0.5;
}

// LAMBERT & PHONG Material-Types
subroutine ( storeMaterialProperties ) void classicMaterial()
{
	// store materialtype in diffuse-component alpha-channel
	out_diffuse.a = GStageMaterial.config.x;

	// store base-color of material
	out_diffuse.rgb = GStageMaterial.color.rgb;
	// encode normal
	out_normal.rg = encodeDirection( IN_OUT.normal.xyz );

	// RFU - no need to set it to 0.0 but here for explicit clarity
	out_normal.b = 0.0;
	out_normal.a = 0.0;
	out_tangent.b = 0.0;
	out_tangent.a = 0.0;
	out_biTangent.b = 0.0;
	out_biTangent.a = 0.0;
}

// LAMBERT & PHONG Material-Types
subroutine ( storeMaterialProperties ) void classicMaterialTextured()
{
	// set classic material first
	classicMaterial();
	// then just add diffuse-texture color
	out_diffuse.rgb += texture( DiffuseTexture, IN_OUT.texCoord ).rgb;
}

// DOOM3 Material-Type
subroutine ( storeMaterialProperties ) void doom3Material()
{
	// hard-code doom3 material to 3.0
	out_diffuse.a = 3.0;

	// store base-color of material
	out_diffuse.rgb = texture( DiffuseTexture, IN_OUT.texCoord ).rgb;

	// doom3 pulls its normals from normal-maps
	vec3 normal = texture( NormalMap, IN_OUT.texCoord).rgb;
	// need to scale light into range [-1, +1] because was stored in normal-map in range [0, 1] 
	normal = 2.0 * normal.xyz - 1.0;
	// encode normal
	out_normal.rg = encodeDirection( normal );

	// fetch specular and store it in alpha-channels of directions
	vec3 specular = texture( SpecularTexture, IN_OUT.texCoord ).rgb;
	// store specular material in the 3 unused b-channels
	out_normal.b = specular.r;
	out_tangent.b = specular.g;
	out_biTangent.b = specular.b;

	// RFU - no need to set it to 0.0 but here for explicit clarity
	out_normal.a = 0.0;
	out_tangent.a = 0.0;
	out_biTangent.a = 0.0;
}

void main()
{
	// encode both tangent and bi-tanget to save one channel each
	// works the same way as normals because tangent&bitangent are directions
	out_tangent.rg = encodeDirection( IN_OUT.tangent.xyz );
	out_biTangent.rg = encodeDirection( IN_OUT.biTangent.xyz );

	// subroutine-call based on the selection from application
	storeMaterialPropertiesSelection();
}
