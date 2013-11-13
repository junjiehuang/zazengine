#version 400 core

#define MAX_BONES_PER_MESH 128

in vec3 in_vertPos;
in vec3 in_vertNorm;
in vec2 in_texCoord;
in vec3 in_tangent;
in uint in_bone_count;
in uvec4 in_bone_indices;
in vec4 in_bone_weights;

// do we really need vec4 or does vec3 suffice?
// IMPORTANT: out-variables are interepolated but no perspective division is applied
out vec4 ex_position;
out vec4 ex_normal;
out vec2 ex_texCoord;
out vec4 ex_tangent;
out vec4 ex_biTangent;

layout( shared ) uniform TransformUniforms
{
	mat4 modelViewMatrix;
	mat4 projectionMatrix;

	mat4 normalsModelViewMatrix;
} Transforms;

uniform mat4 u_bones[ MAX_BONES_PER_MESH ];

subroutine void processInputs();

subroutine ( processInputs ) void processInputsAnimated()
{
	for ( uint i = 0u; i < in_bone_count; i++ )
	{
		uint boneIndex = in_bone_indices[ i ];
		mat4 bone = u_bones[ boneIndex ];
		float boneWeight = in_bone_weights[ i ];

		ex_position += boneWeight * ( bone * vec4( in_vertPos, 1.0 ) ); // fill up with 1.0 because its a position and thus has a length 
		ex_normal += boneWeight * ( bone * vec4( in_vertNorm, 0.0 ) ); // fill up with 0.0 because its a direction and has no length as opposed to position
		ex_tangent += boneWeight * ( bone * vec4( in_tangent, 0.0 ) ); // fill up with 0.0 because its a direction and has no length as opposed to position
	}

	// NOTE: seems not be necessary
	//ex_normal = normalize( ex_normal );
	//ex_tangent = normalize( ex_tangent );
}

subroutine ( processInputs ) void processInputsStatic()
{
	ex_position = vec4( in_vertPos, 1.0 ); // fill up with 1.0 because its a position and thus has a length 
	ex_normal = vec4( in_vertNorm, 0.0 ); // fill up with 0.0 because its a direction and has no length as opposed to position
	ex_tangent = vec4( in_tangent, 0.0 ); // fill up with 0.0 because its a direction and has no length as opposed to position
}

subroutine uniform processInputs processInputsSelection;

void main()
{
	processInputsSelection();

	// store position in view-space (EyeCoordinates) 
	ex_position = Transforms.modelViewMatrix * ex_position;
	// store normals in view-space too (EC)
	ex_normal = Transforms.normalsModelViewMatrix * ex_normal; 
	// no transform for texture-coords, just interpolated
	ex_texCoord = in_texCoord;

	ex_tangent = Transforms.normalsModelViewMatrix * ex_tangent;
	ex_biTangent = vec4( cross( ex_normal.xyz, ex_tangent.xyz ), 0.0 ); // fill up with 0.0 because its a direction and has no length as opposed to position
	 
	// OPTIMIZE: premultiply projection & modelView on CPU 
	// calculate position of vertex using MVP-matrix. 
	// will then be in clip-space after this transformation is applied
	// clipping will be applied
	// then opengl will apply perspective division 
	// after this the coordinates will be between -1 to 1 which is NDC
	// then view-port transform will happen
	// then fragment-shader takes over
	gl_Position = Transforms.projectionMatrix * ex_position;
}
