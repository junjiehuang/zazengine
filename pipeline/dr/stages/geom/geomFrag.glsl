#version 330 core

in vec4 ex_normal;

// provided by OpenGL
in vec4 gl_FragCoord;
out float gl_FragDepth;
///////////////////////

layout( location = 0 ) out vec4 out_diffuse;
layout( location = 1 ) out vec4 out_normal;
layout( location = 2 ) out vec4 out_generic1;
layout( location = 3 ) out vec4 out_generic2;

layout( shared ) uniform material
{
	vec4 materialConfig; 			// x=materialtype, y=diffuseTextureCounter, z=normal map 0/1, w=specular map0/1
	vec4 genericMaterialAttrib1;    // written to ColorAttachment 2
	vec4 genericMaterialAttrib2;    // written to ColorAttachment 3

	vec4 materialColor;     		// base-color of material
};

//uniform sampler2D diffuseTexture;
//uniform sampler2D normalMap;

void main()
{
	// store base-color of material
	out_diffuse.rgb = materialColor.rgb;
	// store materialtype in diffuse-component alpha-channel
	out_diffuse.a = materialConfig.x;

	/*
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
	*/

    // set alpha component of normal to 0
	out_normal.xyz = ex_normal.xyz;
	out_normal.a = 0.0;

    out_generic1 = genericMaterialAttrib1;
    out_generic2 = genericMaterialAttrib2;

	gl_FragDepth = gl_FragCoord.z;
	//gl_FragDepth = gl_FragCoord.x / gl_FragCoord.y;
}