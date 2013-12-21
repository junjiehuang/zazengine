#version 400 core

// light-boundary has only vertex-positions
layout( location = 0 ) in vec3 in_vertPos;

// the screen-coordinate interpolated for each fragment - is used to do the texture-fetches of the MRTs
out vec2 ex_screen_texture_coord;
// the normalized-device-coordinates
out vec3 ex_ndc;

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

	// perform perspective divison by w => clip-space
	ex_ndc = gl_Position.xyz / gl_Position.w;

	// need to transform from clip-space which is in range [-1, 1] to texture-space [0, 1]
	ex_screen_texture_coord = ( ex_ndc.xy + 1.0 ) * 0.5;
}
