#version 330 core

in vec4 ex_shadowCoord;
//in vec4 ex_depth;
in vec4 ex_normal;

out vec4 out_diffuse;
out vec4 out_normal;

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
vec4 packFloatToVec4i( const float value )
{
  const vec4 bitSh = vec4( 256.0 * 256.0 * 256.0, 256.0 * 256.0, 256.0, 1.0 );
  const vec4 bitMsk = vec4( 0.0, 1.0 / 256.0, 1.0 / 256.0, 1.0 / 256.0);
  vec4 res = fract( value * bitSh );
  res -= res.xxyz * bitMsk;
  return res;
}
*/

float shadowLookup( const vec4 shadowCoord, const float offsetX, const float offsetY )
{
	return textureProj( ShadowMap, shadowCoord + vec4( offsetX, offsetY, 0.005, 0.0 ) );
}

void main()
{	
	float shadow = shadowLookup( ex_shadowCoord, 0.0, 0.0 );
	// this fragment is in shadow
	if ( shadow != 1.0 )
	{
		// encode shadow as red
		out_diffuse = vec4( 1.0, 0.0, 0.0, 1.0 );
	}
	else
	{
		// we need to transform the lights direction too when it should not stay with the camera
		// the problem with this approach is, that normalsModelView_Matrix is the matrix transforming
		// normals for the currently rendering object and this will not match the lights transforms
		vec3 lightDir = vec4( normalsModelView_Matrix * vec4( 0.0, 0.0, 1.0, 0.0 ) ).xyz;
		out_diffuse = vec4( 1.0, 1.0, 1.0, 1.0 ) * dot( ex_normal.xyz, lightDir );
	}
	
	//out_depth = packFloatToVec4i( ex_depth.x / ex_depth.y );
	out_normal = ex_normal;
}