#version 400 core

uniform sampler2D DiffuseTexture;
uniform sampler2D NormalTexture;

uniform sampler2D Background;
uniform sampler2DShadow BackgroundDepth;

in vec2 ex_texCoord;

layout( location = 0 ) out vec4 out_color;

// THE CONFIGURATION OF THE CURRENTLY TRANSPARENTY MATERIAL
layout( shared ) uniform TransparentMaterialUniforms
{
	// TODO: document components
	vec4 config;
} TransparentMaterial;

// THE CAMERA CONFIGURATION FOR THE CURRENT VIEW
layout( shared ) uniform CameraUniforms
{
	// the width (x) and height (y) of the camera-window in pixels ( the resolution ), z&w are the reciprocal values
	vec4 window;	
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

const float blendingFactor = 0.5; // TransparentMaterial.config.x
const float refractFactor = 0.05; // TransparentMaterial.config.y

void main()
{
	// transform normal from texture-space [0.0, 1.0] into tangent-space [-1.0, 1.0]
	vec4 refractNormal = 2.0 * texture( NormalTexture, ex_texCoord ) - 1.0;

	// calculate texture-coordinates in screen-space
	vec2 screenTexCoord = vec2( gl_FragCoord.x * Camera.window.z, gl_FragCoord.y * Camera.window.w );
	// pertube the normal in the given direction and scale by a factor
	vec2 refractTexCoord = screenTexCoord + refractNormal.xy * TransparentMaterial.config.y;

	// lookup the refraction-color
	vec4 bgColorRefract = texture( Background, refractTexCoord );
	// lookup the background-color
	vec4 bgColorScreen = texture( Background, screenTexCoord );
	// lookup the material-color
    vec4 materialColor = texture( DiffuseTexture, ex_texCoord );
	
	vec3 depthTexCoord = vec3( refractTexCoord, gl_FragCoord.z );
	float refractFragDepthComparison = texture( BackgroundDepth, depthTexCoord );

	// material-color percentage + refraction-color percentage + transparent background-color percentage
	out_color.rgb = materialColor.rgb * TransparentMaterial.config.x + 
		// will be canceled out if the refracted pixel is occluded by an object nearer => refractFragDepthComparison is 0.0 => removes "bleeding"
		( bgColorRefract.rgb * ( 1.0 - TransparentMaterial.config.x ) ) * refractFragDepthComparison + 
		// if refraction-color is not canceled out, cancel this one out otherwise we would add too much color => too bright 
		( bgColorScreen.rgb * ( 1.0 - TransparentMaterial.config.x ) ) * ( 1.0 - refractFragDepthComparison );

    out_color.a = 0.0;
}