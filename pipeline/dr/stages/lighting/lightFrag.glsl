#version 330 core

uniform sampler2D DiffuseMap;
uniform sampler2D NormalMap;
uniform sampler2D DepthMap;
uniform sampler2D GenericMap1;

uniform sampler2DShadow ShadowMap;

out vec4 final_color;

layout( shared ) uniform camera
{
	mat4 camera_Model_Matrix;			// 0
	mat4 camera_View_Matrix;			// 64
	
	vec4 camera_rec;					// 128
};

layout( shared ) uniform light
{
	vec4 light_Config; 				// x: type, y: falloff, z: shadowCaster 0/1
	vec4 light_Color;

	mat4 light_Model_Matrix;
	mat4 light_SpaceUniform_Matrix;
};

vec4
calculateLambertian( in vec4 diffuse, in vec4 normal, in vec4 position )
{
	// need to transpose light-model matrix to view-space for eye-coordinates 
	mat4 lightMV_Matrix = camera_View_Matrix * light_Model_Matrix;

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
	mat4 lightMV_Matrix = camera_View_Matrix * light_Model_Matrix;

	vec3 lightPos = vec3( lightMV_Matrix[ 3 ] );
	vec3 lightDir = normalize( lightPos - vec3( position ) );
    
	vec3 eyeDir = normalize( vec3( camera_Model_Matrix[ 3 ] ) - vec3( position ) );
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
	vec2 screenCoord = vec2( gl_FragCoord.x / camera_rec.x, gl_FragCoord.y / camera_rec.y );

	vec4 diffuse = texture( DiffuseMap, screenCoord );
	vec4 normal = texture( NormalMap, screenCoord );
	// position of fragment is stored in model-view coordinates = EyeCoordinates (EC)
	// EC is what we need for lighting-calculations
	vec4 ecPosition = texture( GenericMap1, screenCoord );

	// for shadow-mapping we need to transform the position of the fragment to light-space
	// before we can apply the light-space transformation we first need to apply
	// the inverse view-matrix of the camera to transform the position back to world-coordinates (WC)
	// note that world-coordinates is the position after the modeling-matrix was applied to the vertex
	// TODO: something is still wrong, maybe we really need the pure local position directly from the 
	//		 model without ANY matrices applied.
	vec4 wcPosition = inverse( camera_View_Matrix ) * ecPosition;
	vec4 shadowCoord = light_SpaceUniform_Matrix * wcPosition;
	float shadow = 0.0;
	float bias = 0.05;

	// spot-light - do perspective shadow-lookup
	if ( 0.0 == light_Config.x )
	{
		float lightDepth = textureProj( ShadowMap, shadowCoord );

		if ( lightDepth < ( shadowCoord.z - bias ) / shadowCoord.w ) {
			shadow = 0.5;
		}
	}
	else if ( 1.0 == light_Config.x )
	{
		float lightDepth = texture( ShadowMap, shadowCoord );

		if ( lightDepth < shadowCoord.z - bias ) {
			shadow = 0.5;
		}
	}

	// not in shadow
	if ( shadow == 0.0 )
	{
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
	else
	{
		final_color.rgb = diffuse.rgb * 0.5;
		final_color.a = 1.0;
	}
}
