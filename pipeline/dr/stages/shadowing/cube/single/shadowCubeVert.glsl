#version 400 core

#define MAX_BONES_PER_MESH 128

in vec3 in_vertPos;
in uint in_bone_count;
in uvec4 in_bone_indices;
in vec4 in_bone_weights;

// TRANSFORMATIONS FOR THE CURRENT MESH/VERTEX-STREAM
layout( shared ) uniform TransformUniforms
{
	// the model-matrix of the current rendered mesh - transform to world-space
	mat4 modelMatrix;
	// the model-view-matrix of the current rendered mesh - the view-matrix is the one of the Camera - transforms from model-space to view/eye/camera-space
	mat4 modelViewMatrix;
} Transforms;

uniform mat4 u_bones[ MAX_BONES_PER_MESH ];

subroutine vec4 processInputs();

subroutine ( processInputs ) vec4 processInputsAnimated()
{
	vec4 skinnedPosition = vec4( 0.0 );

	for ( uint i = 0u; i < in_bone_count; i++ )
	{
		uint boneIndex = in_bone_indices[ i ];
		mat4 bone = u_bones[ boneIndex ];
		float boneWeight = in_bone_weights[ i ];

		skinnedPosition += boneWeight * ( bone * vec4( in_vertPos, 1.0 ) );
	}

	return skinnedPosition;
}

subroutine ( processInputs ) vec4 processInputsStatic()
{
	return vec4( in_vertPos, 1.0 );
}

subroutine uniform processInputs processInputsSelection;

void main()
{
	// transform vertex into world-space
	gl_Position = Transforms.modelMatrix * processInputsSelection();
}
