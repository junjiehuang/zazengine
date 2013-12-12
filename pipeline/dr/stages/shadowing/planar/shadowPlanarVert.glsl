#version 400 core

#define MAX_BONES_PER_MESH 128

// continue with index 4 after in_vertPos because VAO is arranged this way, in between are the normals, tex-coords and tangents
// those are not needed for shadowing
layout( location = 0 ) in vec3 in_vertPos;
layout( location = 4 ) in uint in_bone_count;
layout( location = 5 ) in uvec4 in_bone_indices;
layout( location = 6 ) in vec4 in_bone_weights;

// THE CURRENT CAMERA
// THIS CORRESPONDS TO THE CAMERA USED FOR RENDERING THE SHADOW-MAP IN THE CASE OF SHADOW-RENDERING IT IS THE LIGHT ITSELF
layout( shared ) uniform CameraUniforms
{
	// the width (x) and height (y) of the camera-window in pixels ( the resolution )
	vec2 window;	
	// the near- (x) and far-plane distances (y)
	vec2 nearFar;

	// the model-matrix of the camera (orienation within world-space)
	mat4 modelMatrix;
	// the view-matrix of the camera to apply to the objects to transform to view/eye/camera-space (is its inverse model-matrix)
	mat4 viewMatrix;
	// the projection-matrix of the camera
	mat4 projectionMatrix;
} Camera;

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
	gl_Position = Camera.projectionMatrix * Transforms.modelViewMatrix * processInputsSelection();
}
