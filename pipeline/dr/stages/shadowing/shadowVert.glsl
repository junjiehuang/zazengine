#version 400 core

#define MAX_BONES_PER_MESH 128

in vec3 in_vertPos;
in uint in_bone_count;
in uvec4 in_bone_indices;
in vec4 in_bone_weights;

layout( shared ) uniform TransformUniforms
{
	mat4 modelViewMatrix;
	mat4 projectionMatrix;

	mat4 normalsModelViewMatrix;
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
	vec4 processedInputPosition = processInputsSelection();
	// OPTIMIZE: premultiply projection & modelView on CPU 
	gl_Position = Transforms.projectionMatrix * Transforms.modelViewMatrix * processedInputPosition;
}
