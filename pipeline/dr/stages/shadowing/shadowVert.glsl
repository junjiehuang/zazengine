#version 330 core

in vec3 in_vertPos;

layout( shared ) uniform TransformUniforms
{
	mat4 modelViewMatrix;
	mat4 projectionMatrix;

	mat4 normalsModelViewMatrix;
} Transforms;

void main()
{
	// OPTIMIZE: premultiply projection & modelView on CPU 
	gl_Position = Transforms.projectionMatrix * Transforms.modelViewMatrix * vec4( in_vertPos, 1.0 );
}
