#version 400 core

// light-boundary has only vertex-positions
layout( location = 0 ) in vec3 in_vertPos;

// defines the output-interface block to the fragment-shader
out VS_TO_FS_OUT
{
	// the screen-coordinate interpolated for each fragment - is used to do the texture-fetches of the MRTs
	vec2 screenTexCoord;
	// the normalized-device-coordinates
	vec3 ndc;
} VS_TO_FS;

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
	VS_TO_FS.ndc = gl_Position.xyz / gl_Position.w;
	// need to transform from clip-space which is in range [-1, 1] to texture-space [0, 1]
	VS_TO_FS.screenTexCoord = ( VS_TO_FS.ndc.xy + 1.0 ) * 0.5;
}
