#version 430 core

layout( location = 0 ) in vec3 in_vertPos;

// defines the output-interface block to the fragment-shader
out IN_OUT_BLOCK
{
	vec3 texCoord;
} OUT;

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
	// just write through tex-coords from vertex-position because vertex position is specified as unit-cube in ranges [-1.0, +1.0]
	OUT.texCoord = in_vertPos;

	gl_Position = Transforms.modelViewProjMatrix * vec4( in_vertPos, 1.0 );
}