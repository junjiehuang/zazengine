#version 400 core

uniform sampler2D DiffuseMap;
uniform sampler2D NormalMap;
uniform sampler2D PositionMap;
uniform sampler2D TangentMap;
uniform sampler2D BiTangentMap;

uniform sampler2DShadow ShadowPlanarMap;
uniform samplerCube ShadowCubeMap;

out vec4 final_color;

const float shadow_bias = 0.001;

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

	vec2 specular; // x = shininess, y = strength

	vec3 attenuation; // x = constant, y = linear, z = quadratic

	vec2 spot; // x = spot cut-off, y = spot exponent

	// the near- (x) and far-plane distances (y)
	// this is necessary for correct shadow-mapping 
	vec2 nearFar;
	mat4 modelMatrix;
	mat4 spaceUniformMatrix;
} Light;
	
///////////////////////////////////////////////////////////////////////////
float shadowLookupProj( vec4 shadowCoord, vec2 offset )
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

	return texture( ShadowPlanarMap, shadowCoord + vec3( offset.x * 1.0 / Light.shadowResolution.x, offset.y * 1.0 / Light.shadowResolution.y, 0.0 )  );
}

// shadowing-functions for all three light-types (directional, spot, point)
float calculateShadowProjective( vec4 fragPosViewSpace )
{
	float shadow = 0.0f;

	// for shadow-mapping we need to transform the position of the fragment to light-space
	// before we can apply the light-space transformation we first need to apply
	// the inverse view-matrix of the camera to transform the position back to world-coordinates (WC)
	// note that world-coordinates is the position after the modeling-matrix was applied to the vertex

	// NOTE: the inverse of the view-matrix should be the modelmatrix because:
	// let M be the model-matrix and V be the view-matrix
	// V = inv( M ) thus for a matrix A holds true: inv( inv ( A ) ) = A
	// so when you calculate matrix X = inv( V ) this can be expanded to X = inv( inv( M ) ) which is again M
	vec4 fragPosWorldSpace = Camera.modelMatrix * fragPosViewSpace;
	vec4 shadowCoord = Light.spaceUniformMatrix * fragPosWorldSpace;

	shadowCoord.z -= shadow_bias;

	// doing soft-shadows using 'percentage-closer filtering' (see GPU Gems 1)
	float x,y;
	for ( y = -1.5 ; y <= 1.5; y += 1.0 )
		for ( x = -1.5 ; x <= 1.5 ; x += 1.0 )
			shadow += shadowLookupProj( shadowCoord, vec2( x, y ) );
				
	// our filter-kernel has 16 elements => divide with 16
	shadow /= 16.0;

	return shadow;
}

float calculateShadowDirectional( vec4 fragPosViewSpace )
{
	float shadow = 0.0f;

	// for shadow-mapping we need to transform the position of the fragment to light-space
	// before we can apply the light-space transformation we first need to apply
	// the inverse view-matrix of the camera to transform the position back to world-coordinates (WC)
	// note that world-coordinates is the position after the modeling-matrix was applied to the vertex

	// NOTE: the inverse of the view-matrix should be the modelmatrix because:
	// let M be the model-matrix and V be the view-matrix
	// V = inv( M ) thus for a matrix A holds true: inv( inv ( A ) ) = A
	// so when you calculate matrix X = inv( V ) this can be expanded to X = inv( inv( M ) ) which is again M
	vec4 fragPosWorldSpace = Camera.modelMatrix * fragPosViewSpace;
	vec4 shadowCoord = Light.spaceUniformMatrix * fragPosWorldSpace;

	// doing soft-shadows using 'percentage-closer filtering' (see GPU Gems 1)
	shadowCoord.z -= shadow_bias;

	float x,y;
	for ( y = -1.5 ; y <= 1.5; y += 1.0 )
		for ( x = -1.5 ; x <= 1.5 ; x += 1.0 )
			shadow += shadowLookup( shadowCoord.xyz, vec2( x, y ) );

	// our filter-kernel has 16 elements => divide with 16
	shadow /= 16.0;
	
	return shadow;
}

float calculateShadowCube( vec4 fragPosViewSpace )
{
	float shadow = 0.0f;

	// for shadow-mapping we need to transform the position of the fragment to light-space
	// before we can apply the light-space transformation we first need to apply
	// the inverse view-matrix of the camera to transform the position back to world-coordinates (WC)
	// note that world-coordinates is the position after the modeling-matrix was applied to the vertex

	// NOTE: the inverse of the view-matrix should be the modelmatrix because:
	// let M be the model-matrix and V be the view-matrix
	// V = inv( M ) thus for a matrix A holds true: inv( inv ( A ) ) = A
	// so when you calculate matrix X = inv( V ) this can be expanded to X = inv( inv( M ) ) which is again M
	vec4 fragPosWorldSpace = Camera.modelMatrix * fragPosViewSpace;

	// light-position in world-space
	vec3 lightPosWorldSpace = Light.modelMatrix[ 3 ].xyz;
	// direction from vertex to light-position in world-space
	vec3 lightDirWorldSpace = fragPosWorldSpace.xyz - lightPosWorldSpace;

	// the distance from the light to the current fragment in world-coordinates
	float fragDistToLight = length( lightDirWorldSpace );
	// the distance from the light to the first hit in NDC (normalized device coordinates)
	float firstHitDistToLight = texture( ShadowCubeMap, lightDirWorldSpace ).r;

	// need nearFar from light because shadow-map was rendered from the viewpoint of the light
	firstHitDistToLight *= Light.nearFar.y;

	// TODO: do percentage-closer filtering
	if ( fragDistToLight < firstHitDistToLight + 0.15 )
	{
		shadow = 1.0;
	}

	return shadow;
}
///////////////////////////////////////////////////////////////////////////

vec3 calculateLambertianMaterial( vec3 baseColor, vec3 normalViewSpace, vec3 lightDirToFragViewSpace )
{
	// need to normalize because for some models the normalization seems to be destroyed (e.g. teapot)
	// TODO: research why this is so 
	normalViewSpace = normalize( normalViewSpace );

	// diffuse (lambert) factor
	float diffuseFactor = max( 0.0, dot( normalViewSpace.xyz, lightDirToFragViewSpace ) );
	vec3 scatteredLight = diffuseFactor * Light.color;	// no ambient light for now

	return baseColor * scatteredLight;
}

vec3 calculatePhongMaterial( vec3 baseColor, vec3 normalViewSpace, vec3 lightDirToFragViewSpace, vec3 fragPosViewSpace )
{
	// need to normalize because for some models the normalization seems to be destroyed (e.g. teapot)
	// TODO: research why this is so 
	normalViewSpace = normalize( normalViewSpace );

	// camera is always at origin 0/0/0 and points into the negative z-achsis so the direction is the negated fragment position in view-space 
	vec3 eyeDirToFragViewSpace = normalize( -fragPosViewSpace );
	vec3 halfVector = normalize( lightDirToFragViewSpace + eyeDirToFragViewSpace );

	// diffuse (lambert) factor
	float diffuseFactor = max( 0.0, dot( normalViewSpace.xyz, lightDirToFragViewSpace ) );
	// OPTIMIZE: calculate only when diffuse > 0
	float specularFactor = pow( max( 0.0, dot( normalViewSpace.xyz, halfVector ) ), Light.specular.x ); // directional light doesn't apply strength at specular factor but at reflected light calculation

	vec3 scatteredLight = diffuseFactor * Light.color;		// no ambient light for now 
	vec3 reflectedLight = Light.color * specularFactor * Light.specular.y;

	return baseColor * scatteredLight + reflectedLight; 
}

vec3 calculateDoom3Material( vec3 baseColor, vec4 normalIn, vec3 lightDirToFragViewSpace, vec3 fragPosViewSpace )
{
	// NO NEED TO NORMALIZE FOR D3 MODELS
	// normalIn = normalize( normalIn );

	// fetch the coordinate of this fragment in normalized screen-space 
	vec2 screenCoord = vec2( gl_FragCoord.x / Camera.window.x, gl_FragCoord.y / Camera.window.y );

	vec4 tangentIn = texture( TangentMap, screenCoord );
	vec4 biTangentIn = texture( BiTangentMap, screenCoord );
	vec3 specularMaterial = vec3( normalIn.a, tangentIn.a, biTangentIn.a );

	// camera is always at origin 0/0/0 and points into the negative z-achsis so the direction is the negated fragment position in view-space 
	vec3 eyeDirToFragViewSpace = normalize( -fragPosViewSpace );
	vec3 halfVectorViewSpace = normalize( lightDirToFragViewSpace + eyeDirToFragViewSpace );

	// need to scale light into range -1.0 to +1.0 because was stored in normal-map and does not come from geometry
	// NOTE: this is the normal in local-space, normal-mapping is done in local-space/tangent-space
	vec3 nLocal = 2.0 * normalIn.xyz - 1.0;
	nLocal = normalize( nLocal );

	// calculate our normal in tangent-space
	vec3 t = normalize( tangentIn.xyz );
	vec3 b = normalize( biTangentIn.xyz );
	vec3 nTangent = cross( t, b );
	nTangent = normalize( nTangent );

	// transform into tangent-space
	// TODO answer and research questions
	// 1: why do we need to perform lighting in tangent-space? Could we not transform the local normal-vector in the geometry-stage to view-space?
	// 2. what are we actually doing here with those dot-products?
	// 3. what is the math behind normal-mapping?
	vec3 eyeVec;
	eyeVec.x = dot( eyeDirToFragViewSpace, t );
	eyeVec.y = dot( eyeDirToFragViewSpace, b );
	eyeVec.z = dot( eyeDirToFragViewSpace, nTangent );
	eyeVec = normalize( eyeVec );

	vec3 lightVectorLocal;
	lightVectorLocal.x = dot( lightDirToFragViewSpace, t );
	lightVectorLocal.y = dot( lightDirToFragViewSpace, b );
	lightVectorLocal.z = dot( lightDirToFragViewSpace, nTangent );
	lightVectorLocal = normalize( lightVectorLocal );

	vec3 halfVectorLocal;
	halfVectorLocal.x = dot( halfVectorViewSpace, t );
	halfVectorLocal.y = dot( halfVectorViewSpace, b );
	halfVectorLocal.z = dot( halfVectorViewSpace, nTangent );
	halfVectorLocal = normalize( halfVectorLocal );

	vec3 finalColor = vec3( 0.0 );

	// calculate attenuation-factor
	float lambertFactor = max( 0.0, dot( nLocal, lightVectorLocal ) );

	if ( lambertFactor > 0.0 )
	{
		float shininess =  pow( max( dot( nLocal, halfVectorLocal ), 0.0 ), 90.0 );

		finalColor = baseColor * lambertFactor;
		finalColor += specularMaterial * shininess;
	}

	return finalColor;
}

///////////////////////////////////////////////////////////////////////////
// lighting-functions for all three light-types (directional, spot, point)
subroutine vec3 lightingFunction( vec4 baseColor, vec4 fragPosViewSpace, vec4 normalViewSpace );

subroutine uniform lightingFunction lightingFunctionSelection;

subroutine ( lightingFunction ) vec3 directionalLight( vec4 baseColor, vec4 fragPosViewSpace, vec4 normalViewSpace )
{
	// calculate shadow-contribution first to skip calculations if totally in shadow
	float shadowFactor = calculateShadowDirectional( fragPosViewSpace );
	// total in shadow, return black
	if ( 0.0 == shadowFactor )
	{
		return vec3( 0.0 );
	}

	// need light-position and direction in view-space: multiply model-matrix of light with cameras view-matrix
	// OPTIMIZE: premultiply on CPU and pass in through Light.modelViewMatrix
	mat4 lightMVMatrix = Camera.viewMatrix * Light.modelMatrix;

	vec3 lightPosViewSpace = lightMVMatrix[ 3 ].xyz;
	vec3 lightDirToFragViewSpace = normalize( lightPosViewSpace - fragPosViewSpace.xyz );
     
	vec3 materialAlbedo;

	// lambert-material
	if ( 1.0 == baseColor.a )
	{
		materialAlbedo = calculateLambertianMaterial( baseColor.rgb, normalViewSpace.xyz, lightDirToFragViewSpace );
	}
	// phong-material
	else if ( 2.0 == baseColor.a )
	{
		materialAlbedo = calculatePhongMaterial( baseColor.rgb, normalViewSpace.xyz, lightDirToFragViewSpace, fragPosViewSpace.xyz );
	}
	// doom3-material
	else if ( 3.0 == baseColor.a )
	{
		materialAlbedo = calculateDoom3Material( baseColor.rgb, normalViewSpace, lightDirToFragViewSpace, fragPosViewSpace.xyz );
	}
	// unknown material, just pass through base-color
	else
	{
		materialAlbedo = baseColor.rgb;
	}

	// apply shadow-filtering: shadow is in the range of 0.0 and 1.0
	// 0.0 applies to 'totally shadowed'
	// 1.0 applies to 'totally in light'
	// all between means partly shadowed, so this will lead to soft-shadows
	materialAlbedo *= shadowFactor;

	return materialAlbedo;
}

subroutine ( lightingFunction ) vec3 spotLight( vec4 baseColor, vec4 fragPosViewSpace, vec4 normalViewSpace )
{
	// calculate shadow-contribution first to skip calculations if totally in shadow
	float shadowFactor = calculateShadowProjective( fragPosViewSpace );
	// total in shadow, return black
	if ( 0.0 == shadowFactor )
	{
		return vec3( 0.0 );
	}

	// need light-position and direction in view-space: multiply model-matrix of light with cameras view-matrix
	// OPTIMIZE: premultiply on CPU and pass in through Light.modelViewMatrix
	mat4 lightMVMatrix = Camera.viewMatrix * Light.modelMatrix;

	vec3 lightPosViewSpace = lightMVMatrix[ 3 ].xyz;
	vec3 lightDirToFragViewSpace = lightPosViewSpace - fragPosViewSpace.xyz;
	// need distance in view-space for falloff
    float lightDistanceViewSpace = length( lightDirToFragViewSpace );

	// normalize light direction
	lightDirToFragViewSpace /= lightDistanceViewSpace;

	float attenuation = 1.0 / ( Light.attenuation.x +		// constant attenuation
		Light.attenuation.y * lightDistanceViewSpace +		// linear attenuation
		Light.attenuation.z * lightDistanceViewSpace * lightDistanceViewSpace );	// quadratic attenuation

	// calculate the cosine between the direction of the light-direction the the fragment and the direction of the light itself which is stored in the model-view matrix z-achsis
	float spotCos = dot( lightDirToFragViewSpace, lightMVMatrix[ 2 ].xyz );
	// attenuation would be 0 so no contribution, return black
	// when angle of the half of the spot is larger than the maximum angle of the half of the spot then no contribution of this light
	if ( spotCos < Light.spot.x )
	{
		return vec3( 0.0 );
	}
	else
	{
		// TODO: make configurable
		attenuation *= pow( spotCos, 7.0 );
	}

	vec3 materialAlbedo;

	// lambert-material
	if ( 1.0 == baseColor.a )
	{
		materialAlbedo = calculateLambertianMaterial( baseColor.rgb, normalViewSpace.xyz, lightDirToFragViewSpace );
	}
	// phong-material
	else if ( 2.0 == baseColor.a )
	{
		materialAlbedo = calculatePhongMaterial( baseColor.rgb, normalViewSpace.xyz, lightDirToFragViewSpace, fragPosViewSpace.xyz );
	}
	// doom3-material
	else if ( 3.0 == baseColor.a )
	{
		materialAlbedo = calculateDoom3Material( baseColor.rgb, normalViewSpace, lightDirToFragViewSpace, fragPosViewSpace.xyz );
	}
	// unknown material, just pass through base-color
	else
	{
		materialAlbedo = baseColor.rgb;
	}

	materialAlbedo *= attenuation; // apply spot-light's attenuation

	// apply shadow-filtering: shadow is in the range of 0.0 and 1.0
	// 0.0 applies to 'totally shadowed'
	// 1.0 applies to 'totally in light'
	// all between means partly shadowed, so this will lead to soft-shadows
	materialAlbedo *= shadowFactor;

	return materialAlbedo;
}

subroutine ( lightingFunction ) vec3 pointLight( vec4 baseColor, vec4 fragPosViewSpace, vec4 normalViewSpace )
{
	// calculate shadow-contribution first to skip calculations if totally in shadow
	float shadowFactor = calculateShadowCube( fragPosViewSpace );
	// total in shadow, return black
	if ( 0.0 == shadowFactor )
	{
		return vec3( 0.0 );
	}

	// need light-position and direction in view-space: multiply model-matrix of light with cameras view-matrix
	// OPTIMIZE: premultiply on CPU and pass in through Light.modelViewMatrix
	mat4 lightMVMatrix = Camera.viewMatrix * Light.modelMatrix;

	vec3 lightPosViewSpace = lightMVMatrix[ 3 ].xyz;
	vec3 lightDirToFragViewSpace = lightPosViewSpace - fragPosViewSpace.xyz;
	// need distance in view-space for falloff
    float lightDistanceViewSpace = length( lightDirToFragViewSpace );

	// normalize light direction
	lightDirToFragViewSpace /= lightDistanceViewSpace;

	float attenuation = 1.0 / ( Light.attenuation.x +		// constant attenuation
		Light.attenuation.y * lightDistanceViewSpace +		// linear attenuation
		Light.attenuation.z * lightDistanceViewSpace * lightDistanceViewSpace );	// quadratic attenuation

	vec3 materialAlbedo;

	// lambert-material
	if ( 1.0 == baseColor.a )
	{
		materialAlbedo = calculateLambertianMaterial( baseColor.rgb, normalViewSpace.xyz, lightDirToFragViewSpace );
	}
	// phong-material
	else if ( 2.0 == baseColor.a )
	{
		materialAlbedo = calculatePhongMaterial( baseColor.rgb, normalViewSpace.xyz, lightDirToFragViewSpace, fragPosViewSpace.xyz );
	}
	// doom3-material
	else if ( 3.0 == baseColor.a )
	{
		materialAlbedo = calculateDoom3Material( baseColor.rgb, normalViewSpace, lightDirToFragViewSpace, fragPosViewSpace.xyz );
	}
	// unknown material, just pass through base-color
	else
	{
		materialAlbedo = baseColor.rgb;
	}

	materialAlbedo *= attenuation; // apply point-light's attenuation

	// apply shadow-filtering: shadow is in the range of 0.0 and 1.0
	// 0.0 applies to 'totally shadowed'
	// 1.0 applies to 'totally in light'
	// all between means partly shadowed, so this will lead to soft-shadows
	materialAlbedo *= shadowFactor;

	return materialAlbedo;
}
///////////////////////////////////////////////////////////////////////////

void main()
{
	// fetch the coordinate of this fragment in normalized screen-space 
	vec2 screenCoord = vec2( gl_FragCoord.x / Camera.window.x, gl_FragCoord.y / Camera.window.y );

	vec4 baseColor = texture( DiffuseMap, screenCoord );

	final_color = baseColor;
	final_color.a = 1.0;

	// apply lighting and shadowing only when material is not sky-box (for sky-box just pass through sky-box texture color)
	if ( 0.0 != baseColor.a )
	{
		vec4 normalViewSpace = texture( NormalMap, screenCoord );
		// position of fragment is stored in model-view coordinates = EyeCoordinates (EC) / View space (VS) / Camera Space
		// EC/VS/CameraSpace is what we need for lighting-calculations
		vec4 fragPosViewSpace = texture( PositionMap, screenCoord );

		final_color.xyz = lightingFunctionSelection( baseColor, fragPosViewSpace, normalViewSpace );
	}
}
