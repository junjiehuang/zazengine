#version 400 core

uniform sampler2D DiffuseMap;
uniform sampler2D NormalMap;
uniform sampler2D PositionMap;
uniform sampler2D DepthMap;

out vec4 final_color;

// THE CAMERA CONFIGURATION FOR THE CURRENT VIEW
// NOTE: THIS HAS TO BE THE CAMERA THE GEOMETRY-STAGE WAS RENDERED WITH
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

// THE CONFIGURATION OF THE CURRENT LIGHT
layout( shared ) uniform LightUniforms
{
	vec4 config; 				// x: type, y: falloff, z: shadowCaster 0/1

	mat4 modelMatrix;
	mat4 spaceUniformMatrix;
} Light;

vec4
calculateLambertian( in vec4 diffuse, in vec4 normal, in vec4 position )
{
	// need to transpose light-model matrix to view-space for eye-coordinates 
	// OPTIMIZE: premultiply on CPU
	mat4 lightMV_Matrix = Camera.viewMatrix * Light.modelMatrix;

	// light-position is stored in 4th vector
	vec3 lightPos = vec3( lightMV_Matrix[ 3 ] );
	vec3 lightDir = normalize( lightPos - vec3( position ) );
    
	// calculate attenuation-factor
	float attenuationFactor = max( dot( vec3( normal ), lightDir ), 0.0 );

	return attenuationFactor * diffuse;
}

vec4
calculatePhong( in vec4 diffuse, in vec4 normal, in vec4 position )
{
	// need to transpose light-model matrix to view-space for eye-coordinates 
	// OPTIMIZE: premultiply on CPU
	mat4 lightMV_Matrix = Camera.viewMatrix * Light.modelMatrix;

	vec3 lightPos = vec3( lightMV_Matrix[ 3 ] );
	vec3 lightDir = normalize( lightPos - vec3( position ) );
    
	vec3 eyeDir = normalize( vec3( Camera.modelMatrix[ 3 ] ) - vec3( position ) );
	vec3 vHalfVector = normalize( lightDir + eyeDir );
    
	// calculate attenuation-factor
	float attenuationFactor = max( dot( vec3( normal ), lightDir ), 0.0 );

	float specularFactor =  pow( max( dot( vec3( normal ), vHalfVector ), 0.0 ), 100 ) * 1.5;

	return attenuationFactor * diffuse + specularFactor;    
}

void main()
{
	// fetch the coordinate of this fragment in normalized
	// screen-space ( 0 – 1 ) 
	vec2 screenCoord = vec2( gl_FragCoord.x / Camera.window.x, gl_FragCoord.y / Camera.window.y );

	vec4 diffuse = texture( DiffuseMap, screenCoord );

	// IMPORTANT: need to normalize normals due to possible uniform-scaling applied to models
	// OPTIMIZE: apply SCALE-Matrix only to modelView and not to normalsModelView
	vec4 normal = normalize( texture( NormalMap, screenCoord ) );

	// position of fragment is stored in model-view coordinates = EyeCoordinates (EC)
	// EC is what we need for lighting-calculations
	vec4 ecPosition = texture( PositionMap, screenCoord );

	float matId = diffuse.a * 255;

	if ( 1.0 == matId )
	{
		final_color = calculateLambertian( diffuse, normal, ecPosition );
	}
	else if ( 2.0 == matId )
	{
		final_color = calculatePhong( diffuse, normal, ecPosition );
	}
	else
	{
		final_color = diffuse;
	}
}
