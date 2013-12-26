#version 430 core

layout( location = 0 ) in vec3 in_vertPos;
layout( location = 1 ) in vec3 in_vertNorm;

// defines the output-interface block to the fragment-shader
out IN_OUT_BLOCK
{
	vec3 normal;
} IN_OUT;

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
	IN_OUT.normal = vec4( Transforms.modelMatrix * vec4( in_vertNorm, 0.0 ) ).xyz;

	gl_Position = Transforms.modelViewProjMatrix * vec4( in_vertPos, 1.0 );
}
