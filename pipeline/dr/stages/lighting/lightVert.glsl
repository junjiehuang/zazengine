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
	// HINT: only apply projection-matrix because we do orthogonal quad rendering and apply no modeling
	gl_Position = Transforms.projectionMatrix * vec4( in_vertPos, 1.0 );
}
