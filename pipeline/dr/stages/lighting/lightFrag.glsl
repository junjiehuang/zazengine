#version 330 core

uniform sampler2D DiffuseMap;
uniform sampler2D NormalMap;
uniform sampler2D PositionMap;
uniform sampler2D DepthMap;
uniform sampler2D TangentMap;
uniform sampler2D BiTangentMap;

uniform sampler2DShadow ShadowPlanarMap;
uniform samplerCube ShadowCubeMap;

out vec4 final_color;

const float shadow_bias = 0.005;

layout( shared ) uniform CameraUniforms
{
	vec4 rectangle;

	mat4 modelMatrix;
	mat4 viewMatrix;
} Camera;

layout( shared ) uniform LightUniforms
{
	vec4 config;

	mat4 modelMatrix;
	mat4 spaceUniformMatrix;
} Light;

vec4
calculateLambertian( in vec4 diffuse, in vec4 normal, in vec4 position )
{
	// IMPORTANT: need to normalize normals due to possible uniform-scaling applied to models
	normal = normalize( normal );

	// need to transpose light-model matrix to view-space for eye-coordinates 
	// when rendering the light, the camer IS the light so this works out
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
	// IMPORTANT: need to normalize normals due to possible uniform-scaling applied to models
	normal = normalize( normal );

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

vec4
calculateDoom3Lighting( in vec4 diffuseIn, in vec4 normalIn, in vec4 tangentIn, in vec4 biTangentIn, in vec4 positionIn )
{
	vec3 fragmentPosition = positionIn.xyz;
	vec4 specularMaterial = vec4( normalIn.a, tangentIn.a, biTangentIn.a, 1.0 );

	// need to scale light into range -1.0 to +1.0 because was stored in normal-map and does not come from geometry
	// NOTE: this is the normal in local-space, normal-mapping is done in local-space/tangent-space
	vec3 nLocal = 2.0 * normalIn.xyz - 1.0;
	nLocal = normalize( nLocal );

	// calculate our normal in tangent-space
	vec3 t = normalize( tangentIn.xyz );
	vec3 b = normalize( biTangentIn.xyz );
	vec3 nTangent = cross( t, b );
	nTangent = normalize( nTangent );

	// need to transpose light-model matrix to view-space for eye-coordinates 
	// OPTIMIZE: premultiply on CPU
	mat4 lightMV_Matrix = Camera.viewMatrix * Light.modelMatrix;

	vec3 lightPos = lightMV_Matrix[ 3 ].xyz;
	vec3 lightDir = normalize( lightPos - fragmentPosition );

	vec3 eyeDir = normalize( Camera.modelMatrix[ 3 ].xyz - fragmentPosition );
	vec3 halfVec = normalize( lightDir + eyeDir );

	// transform into tangent-space
	// TODO answer and research questions
	// 1: why do we need to perform lighting in tangent-space? Could we not transform the local normal-vector in the geometry-stage to view-space?
	// 2. what are we actually doing here with those dot-products?
	// 3. what is the math behind normal-mapping?
	vec3 eyeVec;
	eyeVec.x = dot( eyeDir, t );
	eyeVec.y = dot( eyeDir, b );
	eyeVec.z = dot( eyeDir, nTangent );
	eyeVec = normalize( eyeVec );

	vec3 lightVec;
	lightVec.x = dot( lightDir, t );
	lightVec.y = dot( lightDir, b );
	lightVec.z = dot( lightDir, nTangent );
	lightVec = normalize( lightVec );

	vec3 halfVecLocal;
	halfVecLocal.x = dot( halfVec, t );
	halfVecLocal.y = dot( halfVec, b );
	halfVecLocal.z = dot( halfVec, nTangent );
	halfVecLocal = normalize( halfVecLocal );

	vec4 finalColor = vec4( 0.0 );

	// calculate attenuation-factor
	float lambertFactor = max( dot( nLocal, lightVec ), 0.0 );

	if ( lambertFactor > 0.0 )
	{
		float shininess =  pow( max( dot( nLocal, halfVecLocal ), 0.0 ), 90.0 );

		finalColor = vec4( diffuseIn.rgb * lambertFactor, 1.0 );
		finalColor += specularMaterial * shininess;
	}

	return finalColor;
}

float
shadowLookupProj( vec4 shadowCoord, vec2 offset )
{
	// IMPORTANT: we are only in clip-space, need to divide by w to reach projected NDC.
	// normally the forward-rendering shadow-mapping calculates the shadow-coord
	// in the vertex-shader, which is not possible in the deferred renderer without using 
	// an additional render-target. in forward-rendering between the vertex-shader
	// and the fragment-shader where the shadow-map lookup happens
	// interpolation is carried out by the fixed-function but no perspective division
	// the shadowcoord is in our case already multiplied with the projection-transform
	// but we still need to do the projective-division to reach the according space 0-1
	// because this is not done when applying the projection transform
	// ADDITION: either use 	
	//		vec3 shadowCoordPersp = shadowCoord.xyz / shadowCoord.w; with texture( ShadowPlanarMap, shadowCoordPersp ) lookup 
	//		OR use textureProj( ShadowPlanarMap, shadowCoord ); directly
		
	// IMPORTANT: because we installed a compare-function on this shadow-sampler
	// we don't need to compare it anymore to the z-value of the shadow-coord

	// IMPORTANT: 2048 is the resolution of the shadow-map, need to adjust it
	return textureProj( ShadowPlanarMap, shadowCoord + vec4( offset.x * 1.0 / 2048.0 * shadowCoord.w, offset.y * 1.0 / 2048.0 * shadowCoord.w, 0.0, 0.0 ) );
}

float
shadowLookup( vec3 shadowCoord, vec2 offset )
{
	// IMPORTANT: directionaly light uses orthogonal shadow-map so 
	// transformation of position to shadow-coord is orthgonal projection too
	// so no need for perspective lookup (or divide) because orthogonal 
	// projection has 1 at w so there won't be a foreshortening of values
	// IMPORTANT: because we installed a compare-function on this shadow-sampler
	// we don't need to compare it anymore to the z-value of the shadow-coord

	// IMPORTANT: 2048 is the resolution of the shadow-map, need to adjust it
	return texture( ShadowPlanarMap, shadowCoord + vec3( offset.x * 1.0 / 2048.0, offset.y * 1.0 / 2048.0, 0.0 )  );
}

float 
calculateShadow( vec4 ecPosition )
{
	float shadow = 0.0f;

	// for shadow-mapping we need to transform the position of the fragment to light-space
	// before we can apply the light-space transformation we first need to apply
	// the inverse view-matrix of the camera to transform the position back to world-coordinates (WC)
	// note that world-coordinates is the position after the modeling-matrix was applied to the vertex
	// OPTIMIZE: precalculate inverse camera-view matrix on CPU
	vec4 wcPosition = inverse( Camera.viewMatrix ) * ecPosition;
	vec4 shadowCoord = Light.spaceUniformMatrix * wcPosition;

	shadowCoord.z -= shadow_bias;

	// spot-light - do perspective shadow-lookup
	if ( 0.0 == Light.config.x )
	{
		// doing soft-shadows using 'percentage-closer filtering' (see GPU Gems 1)

		float x,y;
		for ( y = -1.5 ; y <= 1.5; y += 1.0 )
			for ( x = -1.5 ; x <= 1.5 ; x += 1.0 )
				shadow += shadowLookupProj( shadowCoord, vec2( x, y ) );
				
		// our filter-kernel has 16 elements => divide with 16
		shadow /= 16.0;
	}
	// directional-light - do normal shadow-lookup 
	else if ( 1.0 == Light.config.x )
	{
		// doing soft-shadows using 'percentage-closer filtering' (see GPU Gems 1)

		float x,y;
		for ( y = -1.5 ; y <= 1.5; y += 1.0 )
			for ( x = -1.5 ; x <= 1.5 ; x += 1.0 )
				shadow += shadowLookup( shadowCoord.xyz, vec2( x, y ) );

		// our filter-kernel has 16 elements => divide with 16
		shadow /= 16.0;
	}
	// point-light - do cube-map shadow-lookup
	else if ( 2.0 == Light.config.x )
	{
		// need to transpose light-model matrix to view-space for eye-coordinates 
		// when rendering the light, the camera IS the light so this works out
		// OPTIMIZE: premultiply on CPU
		mat4 lightMV_Matrix = Camera.viewMatrix * Light.modelMatrix;

		vec3 lightPos = lightMV_Matrix[ 3 ].xyz;
		// no need to normalize, cube-map lookup also works with normalized vectors
		vec3 lightDir = lightPos - ecPosition.xyz;
		
		// the distance from the light to the current fragment in world-coordinates
		float fragDistToLight = length( lightDir );
		// the distance from the light to the first hit in NDC (normalized device coordinates)
		float firstHitDistToLight = texture( ShadowCubeMap, lightDir ).r;

		// problem: fragDistToLight is in NDC and firstHitDistToLight is in world-space, so we cannot compare

		shadow = 1.0;
	}

	return shadow;
}

void
main()
{
	// fetch the coordinate of this fragment in normalized screen-space ( 0 â€“ 1 ) 
	vec2 screenCoord = vec2( gl_FragCoord.x / Camera.rectangle.x, gl_FragCoord.y / Camera.rectangle.y );

	// fetch diffuse color for this fragment
	vec4 diffuse = texture( DiffuseMap, screenCoord );

	// OPTIMIZE: apply SCALE-Matrix only to modelView and not to normalsModelView
	vec4 normal = texture( NormalMap, screenCoord );

	// position of fragment is stored in model-view coordinates = EyeCoordinates (EC) 
	// EC is what we need for lighting-calculations
	vec4 ecPosition = texture( PositionMap, screenCoord );

	vec4 tangent = texture( TangentMap, screenCoord );
	vec4 biTangent = texture( BiTangentMap, screenCoord );

	final_color = vec4( 0.0 );
	final_color.a = 1.0;

	// sky-box material only diffuse
	if ( 133 == diffuse.a )
	{
		final_color = diffuse;
		final_color.a = 1.0;
	}
	else
	{
		float shadow = calculateShadow( ecPosition );

		// not totally in shadow
		if ( shadow > 0.0 )
		{
			float matId = diffuse.a;

			if ( 1.0 == matId )
			{
				final_color = calculateLambertian( diffuse, normal, ecPosition );
			}
			else if ( 2.0 == matId )
			{
				final_color = calculatePhong( diffuse, normal, ecPosition );
			}
			else if ( 3.0 == matId )
			{
				final_color = calculateDoom3Lighting( diffuse, normal, tangent, biTangent, ecPosition );
			}
			else
			{
				final_color = diffuse;
			}

			// apply shadow-filtering: shadow is in the range of 0.0 and 1.0
			// 0.0 applies to 'totally shadowed'
			// 1.0 applies to 'totally in light'
			// all between means partly shadowed, so this will lead to soft-shadows
			final_color *= shadow;
		}
	}
}
