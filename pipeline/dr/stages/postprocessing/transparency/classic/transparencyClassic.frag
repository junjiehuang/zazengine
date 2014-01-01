#version 430 core

// defines the input-interface block from the vertex-shader
in IN_OUT_BLOCK
{
	vec2 texCoord;
} IN;

layout( location = 0 ) out vec4 out_color;

// diffuse color of the transparent object
layout( binding = 0 ) uniform sampler2D DiffuseTexture;

// THE CONFIGURATION OF THE CURRENTLY TRANSPARENTY CLASSIC MATERIAL
layout( shared ) uniform TransparentClassicMaterialUniforms
{
	// the configuration of the material. x = opacity
	vec2 params;
} TransparentClassicMaterial;

void main()
{
	out_color.rgb = texture( DiffuseTexture, IN.texCoord ).rgb;
    out_color.a = TransparentClassicMaterial.params.x;
}