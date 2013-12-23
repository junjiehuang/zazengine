#version 400 core

// final light-pass mesh has only vertex-positions which are already in NDC
layout( location = 0 ) in vec3 in_vertPos;

// defines the output-interface block to the fragment-shader
out VS_TO_FS_OUT
{
	// the screen-coordinate interpolated for each fragment - is used to do the texture-fetches of the MRTs
	vec2 screenTexCoord;
	// the normalized-device-coordinates
	vec3 ndc;
} VS_TO_FS;

void main()
{
	// the final pass for each light is always done using a full-screen-quad in NDC
	gl_Position = vec4( in_vertPos, 1.0 );

	// just write the values through as the vertex positions are already in NDC
	VS_TO_FS.ndc = gl_Position.xyz;
	// need to transform from NDC which is in range [-1, 1] to texture-space [0, 1]
	VS_TO_FS.screenTexCoord = ( VS_TO_FS.ndc.xy + 1.0 ) * 0.5;
}
