#version 430 core

// defines the output-interface block to the fragment-shader
in IN_OUT_BLOCK
{
	// the screen-coordinate interpolated for each fragment - is used to do the texture-fetches of the MRTs
	vec2 screenTexCoord;
	// the normalized-device-coordinates
	vec3 ndc;
} IN_OUT;

// final color output
layout( location = 0 ) out vec4 final_color;

// THE CAMERA CONFIGURATION FOR THE CURRENT VIEW
// NOTE: THIS HAS TO BE THE CAMERA THE GEOMETRY-STAGE WAS RENDERED WITH
layout( shared ) uniform CameraUniforms
{
	// the resolution of the viewport, z&w are the reciprocal values
	vec4 viewport;	
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
	// color of the light
	vec3 color;
	// attenuation attributes: x = radius, y = 1 / radius
	vec3 attenuation; 
	// specular attributes: x = shininess, y = strength
	vec2 specular; 
	// spot attributes: x = spot cut-off, y = spot exponent
	vec2 spot; 

	// resolution of the planar shadow-map 
	vec2 shadowRes;

	// the near- (x) and far-plane distances (y) - necessary for correct shadow-map sampling
	vec2 nearFar;
	// orientation and position of light in world-space
	mat4 modelMatrix;
	// orientation and position of light in view-space - view-matrix of the Camera
	mat4 modelViewMatrix;
	// planar shadow-map transformation
	mat4 spaceUniformMatrix;
} Light;

// the render-targets of the g-buffer bound as samplers
layout( binding = 0 ) uniform sampler2D DiffuseMap;
layout( binding = 1 ) uniform sampler2D NormalMap;
layout( binding = 2 ) uniform sampler2D TangentMap;
layout( binding = 3 ) uniform sampler2D BiTangentMap;
layout( binding = 4 ) uniform sampler2D DepthMap;

// the shadow-maps - bound depending on light-type
layout( binding = 7 ) uniform sampler2DShadow ShadowPlanarMap;
layout( binding = 20 ) uniform samplerCube ShadowCubeMap;

const float shadow_bias = 0.001;
///////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////
// UTILITIES

// calculates the eye-coordinate of the fragment from depth
// THIS WORKS FOR SYMETRIC FRUSTUMS ONLY ( e.g. created by gluPerspective or glm::createPerspective )
vec3 calcEyeFromDepth( float depth )
{
	vec3 eye;

	// THE NDC-INTERPOLATION WORKS ONLY IN CASE OF A FULL SCREEN QUAD
	
	// calculate EC z-coord but because depth is not stored linearly we need to calculate it normalized
	eye.z = Camera.nearFar.x * Camera.nearFar.y / ( ( depth * ( Camera.nearFar.y - Camera.nearFar.x ) ) - Camera.nearFar.y );
	// solve equations for right frustum plane to get the the EC x-coord
	// NOTE: this works only in case of a symetric frustum, in other case it would be more complex
 	eye.x = ( -IN_OUT.ndc.x * eye.z ) * ( Camera.frustum.x / Camera.nearFar.x );
	// solve equations for top frustum plane to get the the EC y-coord
	// NOTE: this works only in case of a symetric frustum, in other case it would be more complex
	eye.y = ( -IN_OUT.ndc.y * eye.z ) * ( Camera.frustum.y / Camera.nearFar.x );
 
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

	return textureProj( ShadowPlanarMap, shadowCoord + vec4( offset.x * 1.0 / Light.shadowRes.x * shadowCoord.w, offset.y * 1.0 / Light.shadowRes.y * shadowCoord.w, 0.0, 0.0 ) );
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

	return texture( ShadowPlanarMap, shadowCoord + vec3( offset.x * 1.0 / Light.shadowRes.x, offset.y * 1.0 / Light.shadowRes.y, 0.0 )  );
}
///////////////////////////////////////////////////////////////////////////

// Shadow-calculations
// apply shadow-filtering: shadow is in the range of 0.0 and 1.0
// 0.0 applies to 'totally shadowed'
// 1.0 applies to 'totally in light'
// all between means partly shadowed, so this will lead to soft-shadows
subroutine float shadowFunction( vec3 fragPosEC );
subroutine uniform shadowFunction shadowFunctionSelection;

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
	// NOTE: this is VERY VERY high but it just works for now and creates very nice soft-shadows
	shadow /= 110.0;

	return shadow;
}

subroutine ( shadowFunction ) float noShadow( vec3 fragPosEC )
{
	return 1.0;
}
///////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////
// Material-Calculations
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
	// specular factor
	float specularFactor = pow( max( 0.0, dot( normalEC, halfVector ) ), Light.specular.x ); // directional light doesn't apply strength at specular factor but at reflected light calculation

	vec3 scatteredLight = diffuseFactor * Light.color;		// no ambient light for now 
	vec3 reflectedLight = Light.color * specularFactor * Light.specular.y;

	return baseColor * scatteredLight + reflectedLight; 
}

vec3 calculateDoom3Material( vec3 baseColor, vec3 fragPosEC, vec3 normalTS, vec3 lightDirEC, vec2 genericChannels )
{
	vec4 tangentIn = texture( TangentMap, IN_OUT.screenTexCoord );
	vec4 biTangentIn = texture( BiTangentMap, IN_OUT.screenTexCoord );
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
	
	// light-direction in EC is the 3rd (index 2) vector of the model-view matrix of the light
	return calculateMaterialAlbedo( baseColor, fragPosEC, normalEC, Light.modelViewMatrix[ 2 ].xyz, genericChannels ) * shadowFactor;
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

	// light-position in EC is the 4th (index 3) column of the model-view matrix
	vec3 lightDirEC = Light.modelViewMatrix[ 3 ].xyz - fragPosEC;
	// need distance in view-space for falloff
    float lightDistanceEC = length( lightDirEC );

	// normalize light direction
	lightDirEC /= lightDistanceEC;

	// calculate the distance-falloff to the radius will be 1.0 at center and falloff linear with 0.0 beyond the radius
	float attenuation = clamp( 1.0 - ( lightDistanceEC * Light.attenuation.y ), 0.0, 1.0 );

	// calculate the cosine between the direction of the light-direction the the fragment and the direction of the light itself which is stored in the model-view matrix z-achsis
	float spotCos = dot( lightDirEC, Light.modelViewMatrix[ 2 ].xyz );
	// attenuation would be 0 so no contribution, return black
	// when angle of the half of the spot is larger than the maximum angle of the half of the spot then no contribution of this light
	if ( spotCos < Light.spot.x )
	{
		return vec3( 0.0 );
	}
	else
	{
		// simulate smooth falloff to the edges
		attenuation *= pow( spotCos, Light.spot.y );
	}

	return calculateMaterialAlbedo( baseColor, fragPosEC, normalEC, lightDirEC, genericChannels ) * attenuation * shadowFactor;
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

	// light-position in EC is the 4th (index 3) column of the model-view matrix
	vec3 lightDirEC = Light.modelViewMatrix[ 3 ].xyz - fragPosEC;
	// need distance in view-space for falloff
    float lightDistanceEC = length( lightDirEC );

	// normalize light direction
	lightDirEC /= lightDistanceEC;

	// calculate the distance-falloff to the radius will be 1.0 at center and falloff linear with 0.0 beyond the radius
	float attenuation = clamp( 1.0 - ( lightDistanceEC * Light.attenuation.y ), 0.0, 1.0 );

	return calculateMaterialAlbedo( baseColor, fragPosEC, normalEC, lightDirEC, genericChannels ) * attenuation * shadowFactor;
}
///////////////////////////////////////////////////////////////////////////

void main()
{
	float depth = texture( DepthMap, IN_OUT.screenTexCoord );
	vec4 baseColor = texture( DiffuseMap, IN_OUT.screenTexCoord );
	vec4 normalPacked = texture( NormalMap, IN_OUT.screenTexCoord );

	// reconstruct eye-position (fragments-position in view-space) from depth
	vec3 fragPosEC = calcEyeFromDepth( depth );
	// reconstruct z of normal
	vec3 normalEC = decodeDirection( normalPacked.rg );
	// we need the b&a channels for the materials, don't throw them away
	vec2 genericChannels = normalPacked.ba;

	final_color.rgb = lightingFunctionSelection( baseColor, fragPosEC, normalEC, genericChannels );
	final_color.a = 1.0;
}
