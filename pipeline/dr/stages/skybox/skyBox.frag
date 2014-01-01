#version 430 core

// defines the input-interface block from the vertex-shader
in IN_OUT_BLOCK
{
	vec3 texCoord;
} IN;

// writing to bound color-attachment at index 4 - note that to index 4 COLOR_ATTACHMENT4 will be bound
layout( location = 4 ) out vec4 out_diffuse;

layout( binding = 20 ) uniform samplerCube SkyBoxCubeMap;

void main()
{
	// sky-box has diffuse-color only
	out_diffuse.rgb = texture( SkyBoxCubeMap, IN.texCoord ).rgb;
	out_diffuse.a = 1.0;
}