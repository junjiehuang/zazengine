#version 330 core

in vec3 in_vertPos;
in vec2 in_texCoord;

out vec2 ex_texCoord;

layout( shared ) uniform TransformUniforms
{
	mat4 modelViewMatrix;
	mat4 projectionMatrix;

	mat4 normalsModelViewMatrix;
} Transforms;

void main()
{
	// no transform for texture-coords, just interpolated
	ex_texCoord = in_texCoord;

	// OPTIMIZE: premultiply projection & modelView on CPU 
	gl_Position = Transforms.projectionMatrix * vec4( in_vertPos, 1.0 );
}
