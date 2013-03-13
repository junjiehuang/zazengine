#version 330 core

uniform sampler diffuseMap;
uniform sampler normalMap;
uniform sampler depthMap;
uniform sampler genericMap1;
uniform sampler genericMap2;

uniform shadowSampler shadowMap;
uniform shadowSamplerCube shadowCubeMap;

out vec4 final_color;

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

uniform block light
{
        vec4 lightConfig; 				// x: type, y: falloff, z: shadowCaster 0/1
        vec4 lightPosition;             // spot & point only
        vec4 lightDirection;    		// spot & directional only
        vec4 color;

        mat4 lightSpace;
        mat4 lightSpaceUniform;
}

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
	// fetch the coordinate of this fragment in normalized
	// screen-space ( 0 – 1 ) 
	vec2 screenCoord = ...;

	vec4 diffuse = texture( diffuseMap, screenCoord );
	vec4 normal = texture( normalMap, screenCoord );
	// stored as luminance floating point 32bit
	vec4 depth = texture( depthMap, screenCoord ).x;
	vec4 generic1 = texture( genericMap1, screenCoord );
	vec4 generic2 = texture( genericMap2, screenCoord );
    
	float shadow = 0.0f;

	// light is not a point light
	if ( 2 != lightConfig.x )
	{
		// get the world-coordinate of this fragment
		vec4 worldCoord = ...; // apply inverse projection-matrix and undo projection
		// transform the worldCoord into model-coordinates
		vec4 modelCoord = ...; // apply inverse view-matrix
		// get the fragment in the light-space
		vec4 lightCoord = ...; // apply the lightSpace-Matrix to the modelCoord
		// transform lightcoord to fit from NDC (lightSpace matrix includes viewing-projection) into the unit-cube 0-1 to be able to access the shadow-map
		vec4 shadowCoord = ...; // apply the unit-cube matrix

		// spot-light is projective – shadowlookup must be projective
		if ( 0 == lightConfig.x )
		{
			shadow = textureProj( shadowMap, shadowcoord );
		}
		// directional-light is orthographic – no projective shadowlookup
		else
		{
			shadow = texture( shadowMap, shadowcoord );
		}
	}
	// point-light shadow is handled with cube map
	else
	{
		// cube-map access is done through the world-space normal
		vec3 worldSpaceNormal = ...;
		shadow = texture( shadowCubeMap, worldSpaceNormal );
    }

	// this fragment is not in shadow, only then apply lighting
	if ( shadow == 0.0 )
	{
		if ( 0 == diffuse.a )
			renderDiffuseBRDF();
		else if ( 1 == diffuse.a )
			renderLambertianBRDF();
		else if ( 2 == diffuse.a )
			renderPhongBRDF();
		else if ( 3 == diffuse.a )
			renderOrenNayarBRDF();
		else if ( 4 == diffuse.a )
			renderSSSBRDF();
		else if ( 5 == diffuse.a )
			renderWardsBRDF();
		else if ( 6 == diffuse.a )
			renderMicrofacetBRDF();
		end if
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

void renderDiffuseBRDF()
{
}

void renderLambertianBRDF()
{
}

void renderPhongBRDF()
{
}

void renderOrenNayarBRDF()
{
}

void renderSSSBRDF()
{
}

void renderWardsBRDF()
{
}

void renderMicrofacetBRDF()
{
}
