#version 430 core

layout( location = 0 ) in vec3 in_vertPos;
layout( location = 1 ) in vec3 in_vertNorm;
layout( location = 2 ) in vec2 in_texCoord;

// the screen-coordinate interpolated for each fragment - is used to do the texture-fetches of the MRTs
out vec2 ex_screen_texture_coord;
out vec2 ex_texCoord;

// TRANSFORMATIONS FOR THE CURRENT MESH/VERTEX-STREAM
layout( shared ) uniform TransformUniforms
{
	// the model-matrix of the current rendered mesh - transform to world-space
	mat4 modelMatrix;
	// the model-view-matrix of the current rendered mesh - the view-matrix is the one of the Camera
	mat4 modelViewMatrix;
	// the model-view-projection matrix of the current rendered mesh - the view-projection-matrix is the one of the Camera
	mat4 modelViewProjMatrix;
} Transforms;

void main()
{
	// just write-through texture-coordinates
	ex_texCoord = in_texCoord;

	gl_Position = Transforms.modelViewProjMatrix * vec4( in_vertPos, 1.0 );
}
