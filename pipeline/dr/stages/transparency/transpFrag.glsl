#version 400 core

uniform sampler2D DiffuseTexture;
uniform sampler2D NormalTexture;

uniform sampler2D Background;
uniform sampler2DShadow BackgroundDepth;

in vec2 ex_texCoord;

layout( location = 0 ) out vec4 out_color;

layout( shared ) uniform TransparentMaterialUniforms
{
	vec4 config;
} TransparentMaterial;

// THE CAMERA CONFIGURATION FOR THE CURRENT VIEW
layout( shared ) uniform CameraUniforms
{
	// the width (x) and height (y) of the camera-window in pixels ( the resolution )
	vec2 window;	
	// the near- (x) and far-plane distances (y)
	vec2 nearFar;

	// the model-matrix of the camera (orienation within world-space)
	mat4 modelMatrix;
	// the view-matrix of the camera to apply to the objects to transform to view/eye/camera-space (is its inverse model-matrix)
	mat4 viewMatrix;
	// the projection-matrix of the camera
	mat4 projectionMatrix;
} Camera;

const float blendingFactor = 0.5; // TransparentMaterial.config.x
const float refractFactor = 0.05; // TransparentMaterial.config.y

void main()
{
	vec4 refractNormal = 2.0 * texture( NormalTexture, ex_texCoord ) - 1.0;

	vec2 screenTexCoord = vec2( gl_FragCoord.x / Camera.window.x, gl_FragCoord.y / Camera.window.y );
	vec2 refractTexCoord = screenTexCoord + refractNormal.xy * TransparentMaterial.config.y;

	vec4 bgColorRefract = texture( Background, refractTexCoord );
	vec4 bgColorScreen = texture( Background, screenTexCoord );

    vec4 materialColor = texture( DiffuseTexture, ex_texCoord );
	
	vec3 depthTexCoord = vec3( refractTexCoord, gl_FragCoord.z );
	float refractFragDepthComparison = texture( BackgroundDepth, depthTexCoord );

	out_color.rgb = materialColor.rgb * TransparentMaterial.config.x + 
		( bgColorRefract.rgb * ( 1.0 - TransparentMaterial.config.x ) ) * refractFragDepthComparison + 
		( bgColorScreen.rgb * ( 1.0 - TransparentMaterial.config.x ) ) * ( 1.0 - refractFragDepthComparison );

    out_color.a = 0.0;
}