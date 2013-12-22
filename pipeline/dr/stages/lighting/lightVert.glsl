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

// THE CONFIGURATION FOR THE CURRENT LIGHT BOUNDING-MESH
layout( shared ) uniform LightBoundingMeshUniforms
{
	// the mvp-matrix of the current light-bounding mesh
	// in the case of a directional light which has a FSQ this is the plain identity matrix
	// in the other case of point/spot it is a normal mvp-matrix and projects the bounding mesh into the world
	mat4 mvp;
} LightBoundingMesh;

void main()
{
	gl_Position = LightBoundingMesh.mvp * vec4( in_vertPos, 1.0 );

	// works only for orthographic projection: perspective projection seems not to interpolate linearly!! => need to find a way to linearize
	// perform perspective divison by w => clip-space
	VS_TO_FS.ndc = gl_Position.xyz / gl_Position.w;
	// need to transform from clip-space which is in range [-1, 1] to texture-space [0, 1]
	VS_TO_FS.screenTexCoord = ( VS_TO_FS.ndc.xy + 1.0 ) * 0.5;
}
