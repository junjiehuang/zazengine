#version 330 core

in vec3 in_vertPos;
in vec3 in_vertNorm;
in vec2 in_texCoord;

out vec4 ex_pos;
out vec4 ex_normal;
out vec2 ex_texCoord;

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
	ex_normal = Transforms.normalsModelViewMatrix * vec4( in_vertNorm, 0.0 );
	// no transform for texture-coords, just interpolated
	ex_texCoord = in_texCoord;

	// OPTIMIZE: premultiply projection & modelView on CPU 
	gl_Position = Camera.projectionMatrix * Transforms.modelViewMatrix * vec4( in_vertPos, 1.0 );
	
	ex_pos = gl_Position;
}
