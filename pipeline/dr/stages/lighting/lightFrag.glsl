#version 330 core

uniform int screen_width;
uniform int screen_height;
uniform vec4 camera_pos;

uniform sampler2D DiffuseMap;
uniform sampler2D NormalMap;
uniform sampler2D DepthMap;
uniform sampler2D GenericMap1;

uniform sampler2DShadow ShadowMap;

out vec4 final_color;

layout(shared) uniform transforms
{
	mat4 model_Matrix;					// 0
	mat4 modelView_Matrix;				// 64
	mat4 modelViewProjection_Matrix;	// 128
	
	mat4 normalsModelView_Matrix;		// 192
	mat4 normalsModel_Matrix;			// 256
	
	mat4 projection_Matrix;				// 320
	mat4 viewing_Matrix;				// 384
	
	mat4 projectionInv_Matrix;			// 448
	mat4 viewingInv_Matrix;				// 512
};

layout(shared) uniform light
{
	vec4 lightConfig; 				// x: type, y: falloff, z: shadowCaster 0/1
	vec4 lightColor;

	mat4 lightModel_Matrix;
	mat4 lightSpaceUniform_Matrix;
};

vec4
calculateLambertian( in vec4 diffuse, in vec4 normal, in vec4 position )
{
	// need to transpose light-model matrix to view-space for eye-coordinates 
	mat4 lightMV_Matrix = viewing_Matrix * lightModel_Matrix;

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
	mat4 lightMV_Matrix = viewing_Matrix * lightModel_Matrix;

	vec3 lightPos = vec3( lightMV_Matrix[ 3 ] );
	vec3 lightDir = normalize( lightPos - vec3( position ) );
    
	vec3 eyeDir = normalize( vec3( camera_pos ) - vec3( position ) );
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
	vec2 screenCoord = vec2( gl_FragCoord.x / screen_width, gl_FragCoord.y / screen_height );

	vec4 diffuse = texture( DiffuseMap, screenCoord );
	vec4 normal = texture( NormalMap, screenCoord );
	vec4 position = texture( GenericMap1, screenCoord );

	vec4 shadowCoord = lightSpaceUniform_Matrix * position;
	float shadow = 0.0;
	float bias = 0.1;

	// spot-light - do perspective shadow-lookup
	if ( 0.0 == lightConfig.x )
	{
		float lightDepth = textureProj( ShadowMap, shadowCoord );

		if ( lightDepth < ( shadowCoord.z - bias ) / shadowCoord.w ) {
			shadow = 0.5;
		}
	}
	else if ( 1.0 == lightConfig.x )
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
			final_color = calculateLambertian( diffuse, normal, position );
		}
		else if ( 2.0 == matId )
		{
			final_color = calculatePhong( diffuse, normal, position );
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
