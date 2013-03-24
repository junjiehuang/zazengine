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

vec3
viewSpaceFromDepth( const vec2 screenCoord )
{
	// stored as luminance floating point 32bit
	float depth = texture( DepthMap, screenCoord ).x;
    // Get x/w and y/w (NDC) from the viewport position
    // after perspective division through w we are in the normaliced device coordinages 
    // which are in the range from -1 to +1.
    float ndcX = screenCoord.x * 2 - 1;
    float ndcY = ( 1 - screenCoord.y ) * 2 - 1;
    
    vec4 vProjectedPos = vec4( ndcX, ndcY, depth, 1.0f );
    
    // Transform by the inverse projection-view matrix
    vec4 pos = projectionInv_Matrix * vProjectedPos;
    
    // Divide by w to get the view-space position
    vec3 viewSpace = pos.xyz / pos.w;
    
    return viewSpace; 
}

vec4
renderPhongBRDF( in vec4 diffuse, in vec4 normal, in vec4 position )
{
	/*
	vec3 lightPos = lightPosition.xyz;
	vec3 lightDir = light - position.xyz;
    
	lightDir = normalize( lightDir );
    
	vec3 eyeDir = normalize( camera_pos.xyz - position.xyz );
	vec3 vHalfVector = normalize(lightDir.xyz+eyeDir);
    
	return max( dot( normal, lightDir ), 0 ) * diffuse + 
            pow( max( dot( normal,vHalfVector ), 0.0 ), 100 ) * 1.5;
			*/

	return diffuse;
}

vec4
renderOrenNayarBRDF( in vec4 diffuse, in vec4 normal )
{
	return diffuse;
}

vec4
renderSSSBRDF( in vec4 diffuse, in vec4 normal )
{
	return diffuse;
}

vec4
renderWardsBRDF( in vec4 diffuse, in vec4 normal )
{
	return diffuse;
}

vec4
renderMicrofacetBRDF( in vec4 diffuse, in vec4 normal )
{
	return diffuse;
}

vec4
renderLambertianBRDF( in vec4 diffuse, in vec4 normal )
{
	mat4 lightModelViewMatrix = lightModel_Matrix * viewing_Matrix;
	vec4 lightDir = lightModelViewMatrix[ 2 ];

	float intensity = dot( lightDir, normal );
	return vec4( diffuse.r * intensity, diffuse.g * intensity, diffuse.b * intensity, 1.0 );
}

void main()
{
	// fetch the coordinate of this fragment in normalized
	// screen-space ( 0 – 1 ) 
	vec2 screenCoord = vec2( gl_FragCoord.x / screen_width, gl_FragCoord.y / screen_height );

	vec4 diffuse = texture( DiffuseMap, screenCoord );
	vec4 normal = texture( NormalMap, screenCoord );
	vec4 position = texture( GenericMap1, screenCoord );

	// worldCoord = modelCoord
	vec4 viewSpace = vec4( viewSpaceFromDepth( screenCoord ), 1.0 );
	// transform worldCoord to lightspace & fit from NDC (lightSpace matrix includes viewing-projection) 
	// into the unit-cube 0-1 to be able to access the shadow-map
	vec4 shadowCoord = lightSpaceUniform_Matrix * position;

	float shadow = 0.0f;

	// spot-light is projective – shadowlookup must be projective
	if ( 0 == lightConfig.x )
	{
		shadow = textureProj( ShadowMap, shadowCoord );
	}
	// directional-light is orthographic – no projective shadowlookup
	else
	{
		shadow = texture( ShadowMap, shadowCoord.xyz );
	}

	// this fragment is not in shadow, only then apply lighting
	if ( shadow == 0.0 )
	{
		float matId = diffuse.a * 255;

		if ( 1.0 == matId )
		{
			final_color = renderLambertianBRDF( diffuse, normal );
		}
		else if ( 2.0 == matId )
		{
			//final_color = renderPhongBRDF( diffuse, normal, position );
			final_color = renderLambertianBRDF( diffuse, normal );
		}
		else if ( 3.0 == matId )
		{
			final_color = renderOrenNayarBRDF( diffuse, normal );
		}
		else if ( 4.0 == matId )
		{
			final_color = renderSSSBRDF( diffuse, normal );
		}
		else if ( 5.0 == matId )
		{
			final_color = renderWardsBRDF( diffuse, normal );
		}
		else if ( 6.0 == matId )
		{
			final_color = renderMicrofacetBRDF( diffuse, normal );
		}
		else
		{
			final_color = diffuse;
		}
    }
    else
    {
		// TODO: soft-shadows

		// when in shadow, only the diffuse color is used for
		// the final color output but darkened by its half
		final_color.rgb = diffuse.rgb * 0.5;
		final_color.a = 1.0;
    }
}
