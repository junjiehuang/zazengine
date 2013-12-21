#version 400 core

// restrict to max of 128 bones per mesh
#define MAX_BONES_PER_MESH 128

// inputs from the vertex-stream
layout( location = 0 ) in vec3 in_vertPos;
layout( location = 1 ) in vec3 in_vertNorm;
layout( location = 2 ) in vec2 in_texCoord;
layout( location = 3 ) in vec3 in_tangent;
layout( location = 4 ) in uint in_bone_count;
layout( location = 5 ) in uvec4 in_bone_indices;
layout( location = 6 ) in vec4 in_bone_weights;

// defines the output-interface block to the fragment-shader
out VS_TO_FS_OUT
{
	vec4 normal;
	vec2 texCoord;
	vec4 tangent;
	vec4 biTangent;
} VS_TO_FS;

// THE TRANSFORMATIONS FOR THE CURRENT MESH/VERTEX-STREAM
layout( shared ) uniform TransformUniforms
{
	// the model-matrix of the current rendered mesh 
	mat4 modelMatrix;
	// the model-view-matrix of the current rendered mesh - the view-matrix is the one of the Camera
	mat4 modelViewMatrix;
	// the model-view-projection matrix of the current rendered mesh - the view-projection-matrix is the one of the Camera
	mat4 modelViewProjMatrix;
} Transforms;

// the bones for skinning
uniform mat4 u_bones[ MAX_BONES_PER_MESH ];

// the prototype of the input-processing subroutines - all implementing subroutines copy the corresponding data into the VS_TO_FS block and returns the position
// application will select according whether a mesh is animated or not
// NOTE: directions have to be filled up with 0.0 as w because have no length
// NOTE: positions have to be filled up with 1.0 because have a length 
subroutine vec4 processInputs();
// subroutine selection-uniform
subroutine uniform processInputs processInputsSelection;

// skinning is performed
subroutine ( processInputs ) vec4 processInputsAnimated()
{
	vec4 position = vec4( 0.0 );

	for ( uint i = 0u; i < in_bone_count; i++ )
	{
		uint boneIndex = in_bone_indices[ i ];
		mat4 bone = u_bones[ boneIndex ];
		float boneWeight = in_bone_weights[ i ];

		position += boneWeight * ( bone * vec4( in_vertPos, 1.0 ) );
		VS_TO_FS.normal += boneWeight * ( bone * vec4( in_vertNorm, 0.0 ) );
		VS_TO_FS.tangent += boneWeight * ( bone * vec4( in_tangent, 0.0 ) );
	}

	return position;
}

// no animation performed, just copy
subroutine ( processInputs ) vec4 processInputsStatic()
{
	VS_TO_FS.normal = vec4( in_vertNorm, 0.0 );
	VS_TO_FS.tangent = vec4( in_tangent, 0.0 );
	return vec4( in_vertPos, 1.0 );
}

void main()
{
	// returns position transformed or static depending if mesh is animated or not
	vec4 position = processInputsSelection();

	// no transform for texture-coords, just interpolated accross vertices
	VS_TO_FS.texCoord = in_texCoord;

	// non-uniform scaling is forbidden in this engine therefore we can use the normal modelViewMatrix
	VS_TO_FS.normal = Transforms.modelViewMatrix * VS_TO_FS.normal; 
	// non-uniform scaling is forbidden in this engine therefore we can use the normal modelViewMatrix
	VS_TO_FS.tangent = Transforms.modelViewMatrix * VS_TO_FS.tangent;
	// construct bi-tangent from normal and tangent using the cross-product
	VS_TO_FS.biTangent = vec4( cross( VS_TO_FS.normal.xyz, VS_TO_FS.tangent.xyz ), 0.0 ); 
	 
	gl_Position = Transforms.modelViewProjMatrix * position;
}
