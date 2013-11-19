#version 330 core

in vec3 in_vertPos;

out vec3 ex_texCoord;

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
	gl_Position = Camera.projectionMatrix * Transforms.modelViewMatrix * vec4( in_vertPos, 1.0 );

	ex_texCoord = in_vertPos;
}