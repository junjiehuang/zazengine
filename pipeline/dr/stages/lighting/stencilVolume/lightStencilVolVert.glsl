#version 400 core

// light-boundary has only vertex-positions
layout( location = 0 ) in vec3 in_vertPos;

// the mvp-matrix of the current light-bounding mesh
// in the case of a directional light which has a FSQ this is the plain identity matrix
// in the other case of point/spot it is a normal mvp-matrix and projects the bounding mesh into the world
uniform mat4 LightBoundingMeshMVP;

void main()
{
	gl_Position = LightBoundingMeshMVP * vec4( in_vertPos, 1.0 );
}
