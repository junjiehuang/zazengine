#version 330 core

out vec4 out_final;

uniform sampler2D DiffuseMap;
uniform sampler2D DepthMap;
uniform sampler2D NormalMap;

uniform sampler2DShadow ShadowMap;

layout(shared) uniform transforms
{
	mat4 model_Matrix;					// 0
	mat4 modelView_Matrix;				// 64
	mat4 modelViewProjection_Matrix;	// 128
	
	mat4 normalsModelView_Matrix;		// 196
	
	mat4 projection_Matrix;				// 254
	mat4 viewing_Matrix;				// 320
	
	mat4 projectionInv_Matrix;			// 384
	mat4 viewingInv_Matrix;				// 448
};

// contains light-direction in 8,9,10
layout(shared) uniform light
{
	mat4 light_ModelMatrix;
	mat4 light_SpaceMatrix;
	mat4 light_SpaceUnitMatrix;
};

/*
float unpackFloatFromVec4i( const vec4 value )
{
  const vec4 bitSh = vec4( 1.0 / ( 256.0 * 256.0 * 256.0 ), 1.0 / ( 256.0 * 256.0 ), 1.0 / 256.0, 1.0 );
  return ( dot( value, bitSh ) );
}
*/

vec4 positionFromDepth( const vec2 screenCoord, const float depth )
{
    // Get x/w and y/w (NDC) from the viewport position
    // after perspective division through w we are in the normaliced device coordinages 
    // which are in the range from -1 to +1.
    float ndcX = screenCoord.x * 2 - 1;
    float ndcY = ( 1 - screenCoord.y ) * 2 - 1;
    
    vec4 vProjectedPos = vec4( ndcX, ndcY, depth, 1.0f );
    
    // Transform by the inverse projection matrix
    // SOMETHING IS NOT RIGHT HERE MAYBE
    vec4 pos = projectionInv_Matrix * vProjectedPos;
    
    // Divide by w to get the view-space position
    vec4 modelSpace = pos / pos.w;
    
    // transform to model-space because it's the same in lighting-space too
    return viewingInv_Matrix * modelSpace; 
}

float shadowLookup( const vec4 shadowCoord, const float offsetX, const float offsetY )
{
	return textureProj( ShadowMap, shadowCoord + vec4( offsetX, offsetY, 0.0, 0.0 ) );
}

void main()
{
	// 1. get the pixels normalized position on the screen (0-1) 
	vec2 screenCoord = vec2( gl_FragCoord.x / 800, gl_FragCoord.y / 600 );

	// 2. get the diffuse color
	vec4 diffuseComp = texture( DiffuseMap, screenCoord );

	// 3. get depth value
    //float depth = unpackFloatFromVec4i( texture( DepthMap, screenCoord ) );  
	float depth = texture( DepthMap, screenCoord ).r;
	
	// 4. get the position of fragment in world-space
	vec4 fragWorldPos = positionFromDepth( screenCoord, depth );

	// 5. transform the fragment world-pos to its position in light-space unit-cube
	// this means a transformation into the lightspace which is then in -1 to +1 in all directions (NDC)
	// then we map this coordinates to 0-1 to be able to access the shadowmap like a texture
	// the unit-cube transformation is already included in the light_SpaceUnitMatrix
	vec4 fragLightPos = light_SpaceUnitMatrix * fragWorldPos;
	
	// 6. do the shadow lookup
	float shadow = shadowLookup( fragLightPos, 0.0, 0.0 );

	// this fragment is in shadow
	if ( shadow != 1.0 )
	{
		// encode shadow as red
		out_final = vec4( 0.0, 0.0, 1.0, 1.0 );
	}
	else
	{
		// extract the normal from the screen-space normalsmap to apply lighting
		vec3 normal = texture( NormalMap, screenCoord ).xyz;
		
		// we need to transform the lights direction too when it should not stay with the camera
		vec3 lightDir = vec3( 0.0, 1.0, 0.0 );
		
		out_final = diffuseComp * dot( normal.xyz, lightDir );
		out_final.a = 1.0;
	}
}