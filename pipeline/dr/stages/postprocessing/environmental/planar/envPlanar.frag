#version 430 core

layout( location = 0 ) out vec4 out_color;

layout( binding = 1 ) uniform sampler2D PlanarEnvMap;

// THE CONFIGURATION OF THE CURRENTLY ENVIRONMENTAL CUBE MATERIAL
layout( shared ) uniform EnvironmentPlanarMaterialUniforms
{
	// the configuration of the material. x = blending-factor, y = refraction-factor
	vec2 params;
} EnvironmentPlanarCubeMaterial;

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

void main()
{
	// calculate texture-coordinates in screen-space
	vec2 screenTexCoord = vec2( gl_FragCoord.x * Camera.viewport.z, gl_FragCoord.y * Camera.viewport.w );

	// lookup the background-color
	vec3 bgColorScreen = texture( PlanarEnvMap, screenTexCoord ).rgb;

	out_color.rgb = bgColorScreen;
    out_color.a = 0.0;
}