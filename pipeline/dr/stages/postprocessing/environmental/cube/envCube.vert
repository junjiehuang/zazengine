#version 430 core

layout( location = 0 ) in vec3 in_vertPos;
layout( location = 1 ) in vec3 in_vertNorm;

// defines the output-interface block to the fragment-shader
out VS_TO_FS_OUT
{
	vec3 normal;
	vec3 incident;
} VS_TO_FS;

// THE CAMERA CONFIGURATION FOR THE CURRENT VIEW
// NOTE: THIS HAS TO BE THE CAMERA THE GEOMETRY-STAGE WAS RENDERED WITH
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

void main()
{
	vec3 vertPosWS = vec4( Transforms.modelMatrix * vec4( in_vertPos, 1.0 ) ).xyz;

	VS_TO_FS.incident = normalize( vertPosWS - Camera.modelMatrix[ 3 ].xyz );
	VS_TO_FS.normal = vec4( Transforms.modelMatrix * vec4( in_vertNorm, 0.0 ) ).xyz;

	gl_Position = Transforms.modelViewProjMatrix * vec4( in_vertPos, 1.0 );
}
