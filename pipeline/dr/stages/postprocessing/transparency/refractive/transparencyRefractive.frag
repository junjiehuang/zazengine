#version 430 core

// defines the input-interface block from the vertex-shader
in IN_OUT_BLOCK
{
	vec2 texCoord;
} IN_OUT;

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

// diffuse color of the transparent object
layout( binding = 0 ) uniform sampler2D DiffuseTexture;
// normal-map of the transparent object - will be used for refraction
layout( binding = 1 ) uniform sampler2D NormalTexture;

// the background-color behind the object to simulate transparency
layout( binding = 2 ) uniform sampler2D Background;
// need background-depth to prevent "bleeding" objects infront into the refractive one
layout( binding = 3 ) uniform sampler2DShadow BackgroundDepth;

void main()
{
	// transform normal from texture-space [0.0, 1.0] into tangent-space [-1.0, 1.0]
	vec4 refractNormal = 2.0 * texture( NormalTexture, IN_OUT.texCoord ) - 1.0;

	// calculate texture-coordinates in screen-space
	vec2 screenTexCoord = vec2( gl_FragCoord.x * Camera.viewport.z, gl_FragCoord.y * Camera.viewport.w );
	// pertube the normal in the given direction and scale by a factor
	// TODO: scale refractNormal.xy by texture-size
	vec2 refractTexCoord = screenTexCoord + refractNormal.xy * TransparentMaterial.config.y;

	// lookup the refraction-color
	vec4 bgColorRefract = texture( Background, refractTexCoord );
	// lookup the background-color
	vec4 bgColorScreen = texture( Background, screenTexCoord );
	// lookup the material-color
    vec4 materialColor = texture( DiffuseTexture, IN_OUT.texCoord );
	
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