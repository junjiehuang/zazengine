#version 430 core

// defines the input-interface block from the vertex-shader
in IN_OUT_BLOCK
{
	vec3 texCoord;
} IN_OUT;

// writing to bound color-attachment at index 0 - note that to index 0 COLOR_ATTACHMENT4 will be bound
layout( location = 0 ) out vec4 out_diffuse;

layout( binding = 20 ) uniform samplerCube SkyBoxCubeMap;

void main()
{
	// sky-box has diffuse-color only
	out_diffuse.rgb = texture( SkyBoxCubeMap, IN_OUT.texCoord ).rgb;
	out_diffuse.a = 1.0;
}