#version 400 core

layout( location = 0 ) in vec3 in_vertPos;
layout( location = 1 ) in vec2 in_texCoord;

out vec2 ex_texCoord;

uniform mat4 projectionMatrix;

void main()
{
	// no transform for texture-coords, just interpolated
	ex_texCoord = in_texCoord;

	// OPTIMIZE: premultiply projection & modelView on CPU 
	gl_Position = projectionMatrix * vec4( in_vertPos, 1.0 );
}
