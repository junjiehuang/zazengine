#version 430 core

// defines the input-interface block from the geometry-shader
in IN_OUT_BLOCK
{
	vec4 posWorld;
} IN;

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

void main()
{
	// the Camera IS the Light, so take the translation-vector of the modelmatrix to obtain world-space position of light
	vec3 lightPosWorld = Camera.modelMatrix[ 3 ].xyz;
	vec3 lightDirWorld = IN.posWorld.xyz - lightPosWorld;

	// calculate distance 
	float ws_dist = length( lightDirWorld ); 
 
	// map value to [0;1] by dividing by far plane distance 
	float ws_dist_normalized = ws_dist / Camera.nearFar.y; 
	gl_FragDepth = ws_dist_normalized;
}
