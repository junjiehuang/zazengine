#version 400 core

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
	vec2 shadowResolution;

	vec3 color;

	vec2 power; // x = shininess, y = strength

	vec3 attenuation; // x = constant, y = linear, z = quadratic

	vec2 spot; // x = spot cut-off, y = spot exponent

	mat4 modelMatrix;
	mat4 spaceUniformMatrix;
} Light;

subroutine vec3 light();

subroutine ( light ) vec3 directionalLight( vec3 baseColor, vec3 fragPosViewSpace, vec3 normalViewSpace )
{
	// need light-position and direction in view-space: multiply model-matrix of light with cameras view-matrix
	// OPTIMIZE: premultiply on CPU and pass in through Light.modelViewMatrix
	mat4 lightMVMatrix = Camera.viewMatrix * Light.modelMatrix;

	vec3 lightPosViewSpace = lightMVMatrix[ 3 ].xyz;
	vec3 lightDirToFragViewSpace = normalize( lightPosViewSpace - fragPosViewSpace );
    
	// camera is always at origin 0/0/0 and points into the negative z-achsis so the direction is the negated fragment position in view-space 
	vec3 eyeDirToFragViewSpace = normalize( -fragPosViewSpace );
	vec3 vHalfVector = normalize( lightDirToFragViewSpace + eyeDirToFragViewSpace );
    
	// diffuse (lambert) factor
	float diffuseFactor = max( 0.0, dot( normalViewSpace, lightDirToFragViewSpace ) );
	// OPTIMIZE: calculate only when diffuse > 0
	float specularFactor = pow( max( 0.0, dot( normalViewSpace, vHalfVector ) ), Light.power.x ); // directional light doesn't apply strength at specular factor but at reflected light calculation

	vec3 scatteredLight = diffuseFactor * Light.color;		// no ambient light for now 
	vec3 reflectedLight = Light.color * specularFactor * Light.power.y;

	// TODO: directional shadow-calculation

	return baseColor * scatteredLight + reflectedLight;
}

subroutine ( light ) vec3 spotLight( vec3 baseColor, vec3 fragPosViewSpace, vec3 normalViewSpace )
{
	// need light-position and direction in view-space: multiply model-matrix of light with cameras view-matrix
	// OPTIMIZE: premultiply on CPU and pass in through Light.modelViewMatrix
	mat4 lightMVMatrix = Camera.viewMatrix * Light.modelMatrix;

	vec3 lightPosViewSpace = lightMVMatrix[ 3 ].xyz;
	vec3 lightDirToFragViewSpace = lightPosViewSpace - fragPosViewSpace;
	// need distance in view-space for falloff
    float lightDistanceViewSpace = length( lightDirToFragViewSpace );

	// normalize light direction
	lightDirToFragViewSpace /= lightDistanceViewSpace;

	float attenuation = 1.0 / ( Light.attenuation.x +		// constant attenuation
		Light.attenuation.y * lightDistanceViewSpace +		// linear attenuation
		Light.attenuation.z * lightDistanceViewSpace * lightDistanceViewSpace	// quadratic attenuation

	// calculate the cosine between the direction of the light-direction the the fragment and the direction of the light itself which is stored in the model-view matrix z-achsis
	float spotCos = dot( lightDirToFragViewSpace, lightMVMatrix[ 2 ].xyz );

	// camera is always at origin 0/0/0 and points into the negative z-achsis so the direction is the negated fragment position in view-space 
	vec3 eyeDirToFragViewSpace = normalize( -fragPosViewSpace );
	vec3 halfVector = normalize( lightDirToFragViewSpace + eyeDirToFragViewSpace );
    
	if ( spotCos < Light.spot.x )
	{
		attenuation = 0.0;
	}
	else
	{
		attenuation *= pow( spotCos, Light.spot.y );
	}

	// diffuse (lambert) factor
	// OPTIMIZE: calculate only when attenuation > 0
	float diffuseFactor = max( 0.0, dot( normalViewSpace, lightDirToFragViewSpace ) );
	// OPTIMIZE: calculate only when diffuse > 0 AND attenuation > 0
	float specularFactor = pow( max( 0.0, dot( normalViewSpace, halfVector ) ), Light.power.x ) * Light.power.y;

	// OPTIMIZE: calculate only when diffuse > 0 AND attenuation > 0
	vec3 scatteredLight = diffuseFactor * Light.color * attenuation;	// no ambient light for now 
	vec3 reflectedLight = Light.color * specularFactor * attenuation;	// no light-strength for now 

	// TODO: cube-maped shadow-calculation

	return baseColor * scatteredLight + reflectedLight;
}

subroutine ( light ) vec3 pointLight( vec3 baseColor, vec3 fragPosViewSpace, vec3 normalViewSpace )
{
	// need light-position and direction in view-space: multiply model-matrix of light with cameras view-matrix
	// OPTIMIZE: premultiply on CPU and pass in through Light.modelViewMatrix
	mat4 lightMVMatrix = Camera.viewMatrix * Light.modelMatrix;

	vec3 lightPosViewSpace = lightMVMatrix[ 3 ].xyz;
	vec3 lightDirToFragViewSpace = lightPosViewSpace - fragPosViewSpace;
	// need distance in view-space for falloff
    float lightDistanceViewSpace = length( lightDirToFragViewSpace );

	// normalize light direction
	lightDirToFragViewSpace /= lightDistanceViewSpace;

	float attenuation = 1.0 / ( Light.attenuation.x +		// constant attenuation
		Light.attenuation.y * lightDistanceViewSpace +		// linear attenuation
		Light.attenuation.z * lightDistanceViewSpace * lightDistanceViewSpace	// quadratic attenuation

	// camera is always at origin 0/0/0 and points into the negative z-achsis so the direction is the negated fragment position in view-space 
	vec3 eyeDirToFragViewSpace = normalize( -fragPosViewSpace );
	vec3 halfVector = normalize( lightDirToFragViewSpace + eyeDirToFragViewSpace );
    
	// diffuse (lambert) factor
	// OPTIMIZE: calculate only when attenuation > 0
	float diffuseFactor = max( 0.0, dot( normalViewSpace, lightDirToFragViewSpace ) );
	// OPTIMIZE: calculate only when diffuse > 0 AND attenuation > 0
	float specularFactor = pow( max( 0.0, dot( normalViewSpace, halfVector ) ), Light.power.x ) * Light.power.y;

	// OPTIMIZE: calculate only when diffuse > 0 AND attenuation > 0
	vec3 scatteredLight = diffuseFactor * Light.color * attenuation;	// no ambient light for now 
	vec3 reflectedLight = Light.color * specularFactor * attenuation;	// no light-strength for now 

	// TODO: cube-maped shadow-calculation

	return baseColor * scatteredLight + reflectedLight;
}

vec3
calculateLambertian( in vec3 diffuse, in vec3 normal, in vec3 position )
{
	// IMPORTANT: need to normalize normals due to possible uniform-scaling applied to models
	//normal = normalize( normal );

	// need to transpose light-model matrix to view-space for eye-coordinates 
	// OPTIMIZE: premultiply on CPU
	mat4 lightMV_Matrix = Camera.viewMatrix * Light.modelMatrix;

	// light-position is stored in 4th vector
	vec3 lightPos = lightMV_Matrix[ 3 ].xyz;
	vec3 lightDir = normalize( lightPos - position );
    
	// calculate attenuation-factor
	float attenuationFactor = max( dot( normal, lightDir ), 0.0 );

	return attenuationFactor * diffuse;
}

vec3
calculatePhong( in vec3 diffuse, in vec3 normal, in vec3 position )
{
	// IMPORTANT: need to normalize normals due to possible uniform-scaling applied to models
	//normal = normalize( normal );

	// need to transpose light-model matrix to view-space for eye-coordinates 
	// OPTIMIZE: premultiply on CPU
	mat4 lightMV_Matrix = Camera.viewMatrix * Light.modelMatrix;

	vec3 lightPos = lightMV_Matrix[ 3 ].xyz;
	vec3 lightDir = normalize( lightPos - vec3( position ) );
    
	vec3 eyeDir = normalize( Camera.modelMatrix[ 3 ].xyz - position );
	vec3 halfVector = normalize( lightDir + eyeDir );
    
	// calculate attenuation-factor
	float attenuationFactor = max( dot( normal, lightDir ), 0.0 );

	float specularFactor =  pow( max( dot( normal, halfVector ), 0.0 ), 100 ) * 1.5;

	return attenuationFactor * diffuse + specularFactor;    
}

vec3
calculateDoom3Lighting( in vec4 diffuseIn, in vec4 normalIn, in vec4 tangentIn, in vec4 biTangentIn, in vec4 positionIn )
{
	vec3 fragmentPosition = positionIn.xyz;
	vec3 specularMaterial = vec3( normalIn.a, tangentIn.a, biTangentIn.a );

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

	vec3 finalColor = vec3( 0.0 );

	// calculate attenuation-factor
	float lambertFactor = max( dot( nLocal, lightVec ), 0.0 );

	if ( lambertFactor > 0.0 )
	{
		float shininess =  pow( max( dot( nLocal, halfVecLocal ), 0.0 ), 90.0 );

		finalColor = diffuseIn.rgb * lambertFactor;
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

	return textureProj( ShadowPlanarMap, shadowCoord + vec4( offset.x * 1.0 / Light.shadowResolution.x * shadowCoord.w, offset.y * 1.0 / Light.shadowResolution.y * shadowCoord.w, 0.0, 0.0 ) );
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
	return texture( ShadowPlanarMap, shadowCoord + vec3( offset.x * 1.0 / Light.shadowResolution.x, offset.y * 1.0 / Light.shadowResolution.y, 0.0 )  );
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
		// light-position in world-space
		vec3 lightPos = Light.modelMatrix[ 3 ].xyz;
		// direction from vertex to light-position in world-space
		vec3 lightDir = wcPosition.xyz - lightPos;

		// the distance from the light to the current fragment in world-coordinates
		float fragDistToLight = length( lightDir );
		// the distance from the light to the first hit in NDC (normalized device coordinates)
		float firstHitDistToLight = texture( ShadowCubeMap, lightDir ).r;

		firstHitDistToLight *= 1000.0;

		if ( fragDistToLight < firstHitDistToLight + 0.15 )
		{
			shadow = 1.0;
		}
	}

	return shadow;
}

void main()
{
	// fetch the coordinate of this fragment in normalized screen-space 
	vec2 screenCoord = vec2( gl_FragCoord.x / Camera.window.x, gl_FragCoord.y / Camera.window.y );

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
				final_color.xyz = calculateLambertian( diffuse.xyz, normal.xyz, ecPosition.xyz );
			}
			else if ( 2.0 == matId )
			{
				final_color.xyz = calculatePhong( diffuse.xyz, normal.xyz, ecPosition.xyz );
			}
			else if ( 3.0 == matId )
			{
				final_color.xyz = calculateDoom3Lighting( diffuse, normal, tangent, biTangent, ecPosition );
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
