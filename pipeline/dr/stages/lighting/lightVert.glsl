#version 330 core

in vec3 in_vertPos;

layout( shared ) uniform CameraUniforms
{
	vec4 rectangle;

	mat4 modelMatrix;
	mat4 viewMatrix;

	mat4 projectionMatrix;
} Camera;

layout( shared ) uniform TransformUniforms
{
	mat4 modelMatrix;
	mat4 modelViewMatrix;

	mat4 normalsModelViewMatrix;
} Transforms;

void main()
{
	// HINT: only apply projection-matrix because we do orthogonal quad rendering and apply no modeling
	gl_Position = Camera.projectionMatrix * vec4( in_vertPos, 1.0 );
}
