#version 330 core

#define MAX_DIFFUSE_TEXT 4

in vec2 ex_textureCoord;
in vec4 ex_normal;

out vec4 out_diffuse;
out vec4 out_normal;
out vec4 out_generic1;
out vec4 out_generic2;

layout(shared) uniform material
{
	vec4 materialConfig; 			// x=materialtype, y=diffuseTextureCounter, z=normal map 0/1, w=specular map0/1
	vec4 genericMaterialAttrib1;    // written to ColorAttachment 2
	vec4 genericMaterialAttrib2;    // written to ColorAttachment 3

	vec4 materialColor;     		// base-color of material
};

uniform sampler2D diffuseTexture;
uniform sampler2D normalMap;

void main()
{
	// store materialtype in diffuse-component alpha-channel
	out_diffuse.a = materialConfig.x;

	// store base-color of material
	out_diffuse.rgb = materialColor.rgb;

	if ( 1.0 == materialConfig.y )
	{
		out_diffuse.rgb += texture( diffuseTexture, ex_textureCoord ).rgb;
	}

    // normal-mapping enabled – fetch from texture
	if ( 1.0 == materialConfig.z )
	{
		out_normal.xyz = texture( normalMap, ex_textureCoord ).xyz;
	}
	else
	{
		out_normal.xyz = ex_normal.xyz;
	}

    // set alpha component of normal to 0
	out_normal.a = 0.0;

    out_generic1 = genericMaterialAttrib1;
    out_generic2 = genericMaterialAttrib2;
}