#version 400 core

// THE CAMERA CONFIGURATION FOR THE CURRENT VIEW
// THIS CORRESPONDS TO THE CAMERA USED FOR RENDERING THE SHADOW-MAP IN THE CASE OF SHADOW-RENDERING IT IS THE LIGHT ITSELF
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

// passed in interpolated from vertex-shader
in vec3 out_lightDir_world;

void main()
{
	// calculate distance 
	float ws_dist = length( out_lightDir_world ); 
 
	// map value to [0;1] by dividing by far plane distance 
	float ws_dist_normalized = ws_dist / Camera.nearFar.y; 
	gl_FragDepth = ws_dist_normalized;
}
