#version 400 core

layout( location = 0 ) in vec3 in_vertPos;

out vec3 ex_texCoord;

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

// TRANSFORMATIONS FOR THE CURRENT MESH/VERTEX-STREAM
layout( shared ) uniform TransformUniforms
{
	// the model-matrix of the current rendered mesh - transform to world-space
	mat4 modelMatrix;
	// the model-view-matrix of the current rendered mesh - the view-matrix is the one of the Camera - transforms from model-space to view/eye/camera-space
	mat4 modelViewMatrix;
} Transforms;

void main()
{
	gl_Position = Camera.projectionMatrix * Transforms.modelViewMatrix * vec4( in_vertPos, 1.0 );

	ex_texCoord = in_vertPos;
}