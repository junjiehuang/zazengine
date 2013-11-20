#version 400 core

// light-boundary has only vertex-positions
in vec3 in_vertPos;

// TRANSFORMATIONS OF THE BOUNDARY OF THE CURRENT LIGHT
layout( shared ) uniform ScreenRenderingBoundaryUniforms
{
	// the projection-matrix of the current light-boundary
	mat4 projectionMatrix;
} ScreenRenderingBoundary;

void main()
{
	// HINT: only apply projection-matrix because we do orthogonal quad rendering and apply no modeling
	gl_Position = ScreenRenderingBoundary.projectionMatrix * vec4( in_vertPos, 1.0 );
}
