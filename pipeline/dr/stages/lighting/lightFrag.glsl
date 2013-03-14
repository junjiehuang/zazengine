#version 330 core

uniform sampler2D DiffuseMap;
uniform sampler2D NormalMap;
uniform sampler2D DepthMap;

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
	vec4 lightPosition;             // spot & point only
	vec4 lightDirection;    		// spot & directional only
	vec4 lightColor;

	mat4 lightSpaceUniform;
};


vec3
worldPosFromDepth( const vec2 screenCoord )
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
renderLambertianBRDF( in vec4 diffuse, in vec4 normal )
{
	float intensity = dot( lightDirection, normal );
	return vec4( diffuse.r * intensity, diffuse.g * intensity, diffuse.b * intensity, 1.0 );
}

void main()
{
	// fetch the coordinate of this fragment in normalized
	// screen-space ( 0 – 1 ) 
	// TODO problem: what if screen-resolution changes??
	vec2 screenCoord = vec2( gl_FragCoord.x / 1024, gl_FragCoord.y / 768 );
	vec4 diffuse = texture( DiffuseMap, screenCoord );
	vec4 normal = texture( NormalMap, screenCoord );
	
	// worldCoord = modelCoord
	vec4 worldCoord = vec4( worldPosFromDepth( screenCoord ), 1.0 );
	// transform worldCoord to lightspace & fit from NDC (lightSpace matrix includes viewing-projection) 
	// into the unit-cube 0-1 to be able to access the shadow-map
	vec4 shadowCoord = lightSpaceUniform * worldCoord;

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
		final_color = renderLambertianBRDF( diffuse, normal );
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
