#version 430 core

#define MAX_BONES_PER_MESH 128

// continue with index 4 after in_vertPos because VAO is arranged this way, in between are the normals, tex-coords and tangents
// those are not needed for shadowing
layout( location = 0 ) in vec3 in_vertPos;
layout( location = 4 ) in uint in_bone_count;
layout( location = 5 ) in uvec4 in_bone_indices;
layout( location = 6 ) in vec4 in_bone_weights;

// defines the output-interface block to the fragment-shader
out IN_OUT_BLOCK
{
	vec3 lightDirWorld;
} IN_OUT;

// THE CAMERA CONFIGURATION FOR THE CURRENT VIEW
// THIS CORRESPONDS TO THE CAMERA USED FOR RENDERING THE SHADOW-MAP IN THE CASE OF SHADOW-RENDERING IT IS THE LIGHT ITSELF
layout( shared ) uniform CameraUniforms
{
	// the resolution of the viewport, z&w are the reciprocal values
	vec4 viewport;		
	// the near- (x) and far-plane distances (y)
	vec2 nearFar;
	// the symetric frustum: right (left=-right) and top (bottom=-top)
	vec2 frustum;

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
	// the model-view-matrix of the current rendered mesh - the view-matrix is the one of the Camera
	mat4 modelViewMatrix;
	// the model-view-projection matrix of the current rendered mesh - the view-projection-matrix is the one of the Camera
	mat4 modelViewProjMatrix;
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
	// calculate world-position of vertex by applying model-matrix of current mesh
	vec4 vertexPositionWorld = Transforms.modelMatrix * processInputsSelection();
	// the Camera IS the Light, so take the translation-vector of the modelmatrix to obtain world-space position of light
	vec3 lightPosWorld = Camera.modelMatrix[ 3 ].xyz;
	// calculate light-direction in vertex-shader - will be interpolated across vertices, so no need to calculate it in fragment-shader
	IN_OUT.lightDirWorld = vertexPositionWorld.xyz - lightPosWorld;

	// TODO: upload MV-matrix
	gl_Position = Camera.projectionMatrix * Camera.viewMatrix * vertexPositionWorld;
}
