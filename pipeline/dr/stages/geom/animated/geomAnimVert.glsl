#version 330 core

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

void main()
{
	mat4 boneTransform;

	for ( uint i = 0u; i < in_bone_count; i++ )
	{
		uint boneIndex = in_bone_indices[ i ];
		mat4 bone = u_bones[ boneIndex ];
		float boneWeight = in_bone_weights[ i ];

		boneTransform += ( bone * boneWeight );
	}

	ex_position = boneTransform * vec4( in_vertPos, 1.0 );

	// store position in view-space (EyeCoordinates) 
	ex_position = Transforms.modelViewMatrix * ex_position;
	// store normals in view-space too (EC)
	ex_normal = Transforms.normalsModelViewMatrix * vec4( in_vertNorm, 0.0 ); // fill up with 0.0 because its a direction and has no length as opposed to position
	// no transform for texture-coords, just interpolated
	ex_texCoord = in_texCoord;

	ex_tangent = Transforms.normalsModelViewMatrix * vec4( in_tangent, 0.0 ); // fill up with 0.0 because its a direction and has no length as opposed to position
	ex_biTangent = vec4( cross( ex_normal.xyz, ex_tangent.xyz ), 0.0 ); // fill up with 0.0 because its a direction and has no length as opposed to position
	 
	// OPTIMIZE: premultiply projection & modelView on CPU 
	// calculate position of vertex using MVP-matrix. 
	// will then be in clip-space after this transformation is applied
	// clipping will be applied
	// then opengl will apply perspective division 
	// after this the coordinates will be between -1 to 1 which is NDC
	// then view-port transform will happen
	// then fragment-shader takes over
	gl_Position = Transforms.projectionMatrix * Transforms.modelViewMatrix * vec4( in_vertPos, 1.0 );
}
