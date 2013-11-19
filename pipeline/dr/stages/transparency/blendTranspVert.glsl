#version 330 core

in vec3 in_vertPos;
in vec2 in_texCoord;

out vec2 ex_texCoord;

layout( shared ) uniform CameraUniforms
{
	vec4 rectangle;

	mat4 modelMatrix;
	mat4 viewMatrix;

	mat4 projectionMatrix;
} Camera;

void main()
{
	// no transform for texture-coords, just interpolated
	ex_texCoord = in_texCoord;

	// OPTIMIZE: premultiply projection & modelView on CPU 
	gl_Position = Camera.projectionMatrix * vec4( in_vertPos, 1.0 );
}
