#version 400 core

#define MAX_BONES_PER_MESH 128

layout( location = 0 ) in vec3 in_vertPos;
layout( location = 1 ) in vec3 in_vertNorm;
layout( location = 2 ) in vec2 in_texCoord;
layout( location = 3 ) in vec3 in_tangent;
layout( location = 4 ) in uint in_bone_count;
layout( location = 5 ) in uvec4 in_bone_indices;
layout( location = 6 ) in vec4 in_bone_weights;

// do we really need vec4 or does vec3 suffice?
// IMPORTANT: out-variables are interpolated but no perspective division is applied
out vec4 ex_position;
out vec4 ex_normal;
out vec2 ex_texCoord;
out vec4 ex_tangent;
out vec4 ex_biTangent;

// THE CAMERA CONFIGURATION FOR THE CURRENT VIEW
layout( shared ) uniform CameraUniforms
{
	// the width (x) and height (y) of the camera-window in pixels ( the resolution )
	vec2 window;	
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

// THE TRANSFORMATIONS FOR THE CURRENT MESH/VERTEX-STREAM
layout( shared ) uniform TransformUniforms
{
	// the model-matrix of the current rendered mesh - transform to world-space
	mat4 modelMatrix;
	// the model-view-matrix of the current rendered mesh - the view-matrix is the one of the Camera - transforms from model-space to view/eye/camera-space
	mat4 modelViewMatrix;
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
	// store normals in view-space too (EC) - non-uniform scaling is forbidden in this engine therefore we can use the normal modelViewMatrix
	ex_normal = Transforms.modelViewMatrix * ex_normal; 
	// no transform for texture-coords, just interpolated
	ex_texCoord = in_texCoord;

	// non-uniform scaling is forbidden in this engine therefore we can use the normal modelViewMatrix
	ex_tangent = Transforms.modelViewMatrix * ex_tangent;
	ex_biTangent = vec4( cross( ex_normal.xyz, ex_tangent.xyz ), 0.0 ); // fill up with 0.0 because its a direction and has no length as opposed to position
	 
	// OPTIMIZE: premultiply projection & modelView on CPU 
	// calculate position of vertex using MVP-matrix. 
	// will then be in clip-space after this transformation is applied
	// clipping will be applied
	// then opengl will apply perspective division 
	// after this the coordinates will be between -1 to 1 which is NDC
	// then view-port transform will happen
	// then fragment-shader takes over
	gl_Position = Camera.projectionMatrix * ex_position;
}
