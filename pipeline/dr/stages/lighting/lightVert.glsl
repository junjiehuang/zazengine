#version 400 core

// light-boundary has only vertex-positions
layout( location = 0 ) in vec3 in_vertPos;

// the eye-coordinate in clip-space from the origin
out vec3 ex_eye_dir_clip;
// the screen-coordinate interpolated for each fragment
// is used to do the texture-fetches of the MRTs
out vec2 ex_screen_coord;

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

	// perform perspective divison by w => clips-space
	vec2 clipPos = gl_Position.xy / gl_Position.w;

	// need to transform from clip-space which is in range [-1.0 1.0] to texture-space [0.0 to 1.0]
	ex_screen_coord = ( clipPos.xy + 1.0 ) * 0.5;
	// 
	ex_eye_dir_clip = vec3( clipPos.xy, -1 );
}
