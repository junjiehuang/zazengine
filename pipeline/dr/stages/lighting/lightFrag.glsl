#version 400 core

uniform sampler2D DiffuseMap;
uniform sampler2D NormalMap;
uniform sampler2D TangentMap;
uniform sampler2D BiTangentMap;
uniform sampler2D DepthMap;

uniform sampler2DShadow ShadowPlanarMap;
uniform samplerCube ShadowCubeMap;

layout( location = 0 ) out vec4 final_color;

// the screen-coordinate interpolated for each fragment - is used to do the texture-fetches of the MRTs
in vec2 ex_screen_texture_coord;
// the normalized-device-coordinates
in vec3 ex_ndc;

// TODO: make configurable/light
const float shadow_bias = 0.005;

// THE CAMERA CONFIGURATION FOR THE CURRENT VIEW
// NOTE: THIS HAS TO BE THE CAMERA THE GEOMETRY-STAGE WAS RENDERED WITH
layout( shared ) uniform CameraUniforms
{
	// the width (x) and height (y) of the camera-window in pixels ( the resolution )
	vec2 window;	
	// the near- (x) and far-plane distances (y)
	vec2 nearFar;
	// the symetric frustum: right (left=-right) and top (bottom=-top)
	vec2 frustum;

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

// UTILITIES
// calculates the eye-coordinate of the fragment from depth
// THIS WORKS FOR SYMETRIC FRUSTUMS ONLY ( e.g. created by gluPerspective or glm::createPerspective )
vec3 calcEyeFromDepth( float depth )
{
	vec3 eye;

	// TODO: research what is going on here
	eye.z = Camera.nearFar.x * Camera.nearFar.y / ( ( depth * ( Camera.nearFar.y - Camera.nearFar.x ) ) - Camera.nearFar.y );
 	eye.x = ( -ex_ndc.x * eye.z ) * ( Camera.frustum.x / Camera.nearFar.x );
	eye.y = ( -ex_ndc.y * eye.z ) * ( Camera.frustum.y / Camera.nearFar.x );
 
	return eye;
}

// decodes a vec2 direction-vector to its vec3 representation which was encoded in the geometry-stage
vec3 decodeDirection( vec2 dir )
{
	/* SPHEREMAP-TRANSFORM RECONSTRUCTION USING LAMBERT AZIMUTHAL EQUAL-AREA PROJECTION */
	vec2 fenc = dir * 4 - 2;
    float f = dot( fenc, fenc );
    float g = sqrt( 1 - f * 0.25 ); // 0.25 can be represented exactly in binary system
    vec3 n;
    n.xy = fenc * g;
    n.z = 1 - f * 0.5; // 0.5 can be represented exactly in binary system
    return n;
}

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
///////////////////////////////////////////////////////////////////////////

// Shadow-calculations
subroutine float shadowFunction( vec3 fragPosEC );

subroutine ( shadowFunction ) float directionalShadow( vec3 fragPosEC )
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
	vec4 fragPosWS = Camera.modelMatrix * vec4( fragPosEC, 1.0 );
	vec4 shadowCoord = Light.spaceUniformMatrix * fragPosWS;

	// doing soft-shadows using 'percentage-closer filtering' (see GPU Gems 1)
	shadowCoord.z -= shadow_bias;

	// TODO: make samples-count configurable by light
	float x,y;
	for ( y = -1.5 ; y <= 1.5; y += 1.0 )
		for ( x = -1.5 ; x <= 1.5 ; x += 1.0 )
			shadow += shadowLookup( shadowCoord.xyz, vec2( x, y ) );

	// our filter-kernel has 16 elements => divide with 16
	shadow /= 16.0;
	
	return shadow;
}

subroutine ( shadowFunction ) float projectiveShadow( vec3 fragPosEC )
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
	vec4 fragPosWS = Camera.modelMatrix * vec4( fragPosEC, 1.0 );
	vec4 shadowCoord = Light.spaceUniformMatrix * fragPosWS;

	shadowCoord.z -= shadow_bias;

	// doing soft-shadows using 'percentage-closer filtering' (see GPU Gems 1)
	// TODO: make samples-count configurable by light
	float x,y;
	for ( y = -1.5 ; y <= 1.5; y += 1.0 )
		for ( x = -1.5 ; x <= 1.5 ; x += 1.0 )
			shadow += shadowLookupProj( shadowCoord, vec2( x, y ) );
				
	// our filter-kernel has 16 elements => divide with 16
	shadow /= 16.0;

	return shadow;
}

subroutine ( shadowFunction ) float cubeShadow( vec3 fragPosEC )
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
	vec4 fragPosWS = Camera.modelMatrix * vec4( fragPosEC, 1.0 );

	// light-position in world-space
	vec3 lightPosWS = Light.modelMatrix[ 3 ].xyz;
	// direction from vertex to light-position in world-space
	vec3 lightDirWS = fragPosWS.xyz - lightPosWS;

	// the distance from the light to the current fragment in world-coordinates
	float fragDistToLight = length( lightDirWS );

	// TODO: make samples-count configurable by light
	float x,y;
	for ( y = -0.5 ; y <= 0.5; y += 0.1 )
	{
		for ( x = -0.5 ; x <= 0.5 ; x += 0.1 )
		{
			// the distance from the light to the first hit in NDC (normalized device coordinates)
			float firstHitDistToLight = texture( ShadowCubeMap, lightDirWS + vec3( x, y, 0.0 ) ).r;

			// need nearFar from light because shadow-map was rendered from the viewpoint of the light
			firstHitDistToLight *= Light.nearFar.y;

			if ( fragDistToLight < firstHitDistToLight + 0.15 )
			{
				shadow++;
			}
		}
	}

	// our filter-kernel has 110 elements => divide with 110
	shadow /= 110.0;

	return shadow;
}

subroutine ( shadowFunction ) float noShadow( vec3 fragPosEC )
{
	return 1.0;
}

subroutine uniform shadowFunction shadowFunctionSelection;
///////////////////////////////////////////////////////////////////////////

vec3 calculateLambertianMaterial( vec3 baseColor, vec3 normalEC, vec3 lightDirEC )
{
	// diffuse (lambert) factor
	float diffuseFactor = max( 0.0, dot( normalEC, lightDirEC ) );
	vec3 scatteredLight = diffuseFactor * Light.color;	// no ambient light for now

	return baseColor * scatteredLight;
}

vec3 calculatePhongMaterial( vec3 baseColor, vec3 fragPosEC, vec3 normalEC, vec3 lightDirEC )
{
	// camera is always at origin 0/0/0 and points into the negative z-achsis so the direction is the negated fragment position in view-space 
	vec3 eyeDirToFragEC = normalize( -fragPosEC );
	vec3 halfVector = normalize( lightDirEC + eyeDirToFragEC );

	// diffuse (lambert) factor
	float diffuseFactor = max( 0.0, dot( normalEC, lightDirEC ) );
	// OPTIMIZE: calculate only when diffuse > 0
	float specularFactor = pow( max( 0.0, dot( normalEC, halfVector ) ), Light.specular.x ); // directional light doesn't apply strength at specular factor but at reflected light calculation

	vec3 scatteredLight = diffuseFactor * Light.color;		// no ambient light for now 
	vec3 reflectedLight = Light.color * specularFactor * Light.specular.y;

	return baseColor * scatteredLight + reflectedLight; 
}

vec3 calculateDoom3Material( vec3 baseColor, vec3 fragPosEC, vec3 normalTS, vec3 lightDirEC, vec2 genericChannels )
{
	vec4 tangentIn = texture( TangentMap, ex_screen_texture_coord );
	vec4 biTangentIn = texture( BiTangentMap, ex_screen_texture_coord );
	vec3 specularMaterial = vec3( genericChannels.r, tangentIn.b, biTangentIn.b );

	// camera is always at origin 0/0/0 and points into the negative z-achsis so the direction is the negated fragment position in view-space 
	vec3 eyeDirToFragEC = normalize( -fragPosEC );
	vec3 halfVectorEC = normalize( lightDirEC + eyeDirToFragEC );

	// calculate our normal in tangent-space
	// first we need to decode our tangent & bitangent because were stored encoded to a vec2
	vec3 t = decodeDirection( tangentIn.xy );
	vec3 b = decodeDirection( biTangentIn.xy );
	vec3 nTangent = cross( t, b );
	nTangent = nTangent;

	// transform into tangent-space
	// TODO answer and research questions
	// 1: why do we need to perform lighting in tangent-space? Could we not transform the local normal-vector in the geometry-stage to view-space?
	// 2. what are we actually doing here with those dot-products?
	// 3. what is the math behind normal-mapping?
	vec3 eyeVec;
	eyeVec.x = dot( eyeDirToFragEC, t );
	eyeVec.y = dot( eyeDirToFragEC, b );
	eyeVec.z = dot( eyeDirToFragEC, nTangent );
	eyeVec = normalize( eyeVec );

	vec3 lightVectorLocal;
	lightVectorLocal.x = dot( lightDirEC, t );
	lightVectorLocal.y = dot( lightDirEC, b );
	lightVectorLocal.z = dot( lightDirEC, nTangent );
	lightVectorLocal = normalize( lightVectorLocal );

	vec3 halfVectorLocal;
	halfVectorLocal.x = dot( halfVectorEC, t );
	halfVectorLocal.y = dot( halfVectorEC, b );
	halfVectorLocal.z = dot( halfVectorEC, nTangent );
	halfVectorLocal = normalize( halfVectorLocal );

	vec3 finalColor = vec3( 0.0 );

	// calculate attenuation-factor
	float lambertFactor = max( 0.0, dot( normalTS, lightVectorLocal ) );

	if ( lambertFactor > 0.0 )
	{
		float shininess =  pow( max( dot( normalTS, halfVectorLocal ), 0.0 ), 90.0 );

		finalColor = baseColor * lambertFactor;
		finalColor += specularMaterial * shininess;
	}

	return finalColor;
}

vec3 calculateMaterialAlbedo( vec4 baseColor, vec3 fragPosEC, vec3 normalEC, vec3 lightDirEC, vec2 genericChannels )
{
	// lambert-material
	if ( 1.0 == baseColor.a )
	{
		return calculateLambertianMaterial( baseColor.rgb, normalEC, lightDirEC );
	}
	// phong-material
	else if ( 2.0 == baseColor.a )
	{
		return calculatePhongMaterial( baseColor.rgb, fragPosEC, normalEC, lightDirEC );
	}
	// doom3-material
	else if ( 3.0 == baseColor.a )
	{
		return calculateDoom3Material( baseColor.rgb, fragPosEC, normalEC, lightDirEC, genericChannels );
	}
	
	// unknown material, just pass through base-color
	return baseColor.rgb;
}

///////////////////////////////////////////////////////////////////////////
// lighting-functions for all three light-types (directional, spot, point)
subroutine vec3 lightingFunction( vec4 baseColor, vec3 fragPosEC, vec3 normalEC, vec2 genericChannels );

subroutine uniform lightingFunction lightingFunctionSelection;

subroutine ( lightingFunction ) vec3 directionalLight( vec4 baseColor, vec3 fragPosEC, vec3 normalEC, vec2 genericChannels )
{
	// calculate shadow-contribution first to skip calculations if totally in shadow
	float shadowFactor = shadowFunctionSelection( fragPosEC );
	// total in shadow, return black
	if ( 0.0 == shadowFactor )
	{
		return vec3( 0.0 );
	}

	// need light-position and direction in view-space: multiply model-matrix of light with cameras view-matrix
	// OPTIMIZE: premultiply on CPU and pass in through Light.modelViewMatrix
	mat4 lightMVMatrix = Camera.viewMatrix * Light.modelMatrix;
	vec3 lightDirEC = lightMVMatrix[ 2 ].xyz;
	
	vec3 materialAlbedo = calculateMaterialAlbedo( baseColor, fragPosEC, normalEC, lightDirEC, genericChannels );

	// apply shadow-filtering: shadow is in the range of 0.0 and 1.0
	// 0.0 applies to 'totally shadowed'
	// 1.0 applies to 'totally in light'
	// all between means partly shadowed, so this will lead to soft-shadows
	materialAlbedo *= shadowFactor;

	return materialAlbedo;
}

subroutine ( lightingFunction ) vec3 spotLight( vec4 baseColor, vec3 fragPosEC, vec3 normalEC, vec2 genericChannels )
{
	// calculate shadow-contribution first to skip calculations if totally in shadow
	float shadowFactor = shadowFunctionSelection( fragPosEC );
	// total in shadow, return black
	if ( 0.0 == shadowFactor )
	{
		return vec3( 0.0 );
	}

	// need light-position and direction in view-space: multiply model-matrix of light with cameras view-matrix
	// OPTIMIZE: premultiply on CPU and pass in through Light.modelViewMatrix
	mat4 lightMVMatrix = Camera.viewMatrix * Light.modelMatrix;

	vec3 lightPosViewSpace = lightMVMatrix[ 3 ].xyz;
	vec3 lightDirEC = lightPosViewSpace - fragPosEC;
	// need distance in view-space for falloff
    float lightDistanceViewSpace = length( lightDirEC );

	// normalize light direction
	lightDirEC /= lightDistanceViewSpace;

	float attenuation = 1.0 / ( Light.attenuation.x +		// constant attenuation
		Light.attenuation.y * lightDistanceViewSpace +		// linear attenuation
		Light.attenuation.z * lightDistanceViewSpace * lightDistanceViewSpace );	// quadratic attenuation

	// calculate the cosine between the direction of the light-direction the the fragment and the direction of the light itself which is stored in the model-view matrix z-achsis
	float spotCos = dot( lightDirEC, lightMVMatrix[ 2 ].xyz );
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

	vec3 materialAlbedo = calculateMaterialAlbedo( baseColor, fragPosEC, normalEC, lightDirEC, genericChannels );

	materialAlbedo *= attenuation; // apply spot-light's attenuation

	// apply shadow-filtering: shadow is in the range of 0.0 and 1.0
	// 0.0 applies to 'totally shadowed'
	// 1.0 applies to 'totally in light'
	// all between means partly shadowed, so this will lead to soft-shadows
	materialAlbedo *= shadowFactor;

	return materialAlbedo;
}

subroutine ( lightingFunction ) vec3 pointLight( vec4 baseColor, vec3 fragPosEC, vec3 normalEC, vec2 genericChannels )
{
	// calculate shadow-contribution first to skip calculations if totally in shadow
	float shadowFactor = shadowFunctionSelection( fragPosEC );
	// total in shadow, return black
	if ( 0.0 == shadowFactor )
	{
		return vec3( 0.0 );
	}

	// need light-position and direction in view-space: multiply model-matrix of light with cameras view-matrix
	// OPTIMIZE: premultiply on CPU and pass in through Light.modelViewMatrix
	mat4 lightMVMatrix = Camera.viewMatrix * Light.modelMatrix;

	vec3 lightPosViewSpace = lightMVMatrix[ 3 ].xyz;
	vec3 lightDirEC = lightPosViewSpace - fragPosEC;
	// need distance in view-space for falloff
    float lightDistanceViewSpace = length( lightDirEC );

	// normalize light direction
	lightDirEC /= lightDistanceViewSpace;

	float attenuation = 1.0 / ( Light.attenuation.x +		// constant attenuation
		Light.attenuation.y * lightDistanceViewSpace +		// linaer attenuation
		Light.attenuation.z * lightDistanceViewSpace * lightDistanceViewSpace );	// quadratic attenuation

	vec3 materialAlbedo = calculateMaterialAlbedo( baseColor, fragPosEC, normalEC, lightDirEC, genericChannels );

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
	float depth = texture( DepthMap, ex_screen_texture_coord );
	vec4 baseColor = texture( DiffuseMap, ex_screen_texture_coord );
	vec4 normalPacked = texture( NormalMap, ex_screen_texture_coord );

	// reconstruct eye-position (fragments-position in view-space) from depth
	vec3 fragPosEC = calcEyeFromDepth( depth );
	// reconstruct z of normal
	vec3 normalEC = decodeDirection( normalPacked.rg );
	// we need the b&a channels for the materials, don't throw them away
	vec2 genericChannels = normalPacked.ba;

	final_color.rgb = lightingFunctionSelection( baseColor, fragPosEC, normalEC, genericChannels );
	final_color.a = 1.0;
}
