#version 330 core

in vec4 ex_color;
in vec4 ex_shadowCoord;

out vec4 out_color;

//uniform sampler2DShadow ShadowMap;
uniform sampler2D ShadowMap;

/*
out vec4 out_diffuse;
out vec4 out_normal;
out vec4 out_depth;
out vec4 out_generic;
*/

/*
float lookup( float offsetX, float offsetY )
{
	float depth = textureProj( ShadowMap, ex_shadowCoord + vec4( offsetX, offsetY, 0.0, 0.0 ) );
	return depth != 1.0 ? 0.75 : 1.0; 
}
*/

void main()
{
vec4 shadowCoordinateWdivide = ex_shadowCoord / ex_shadowCoord.w ;
	out_color = texture2D( ShadowMap, shadowCoordinateWdivide.st );

/*
	float factor = lookup( 0.0, 0.0 );
	out_color = vec4( factor * ex_color.rgb, ex_color.a );
	*/
	/*
	vec4 shadowCoordinateWdivide = ex_shadowCoord / ex_shadowCoord.w ;
		
	// Used to lower moiré pattern and self-shadowing
	shadowCoordinateWdivide.z += 0.0005;

	float distanceFromLight = texture2D( ShadowMap, shadowCoordinateWdivide.st ).z;

	if ( ex_shadowCoord.w > 0.0 )
	{
		//shadow = distanceFromLight < shadowCoordinateWdivide.z ? 0.5 : 1.0 ;
		if ( distanceFromLight < shadowCoordinateWdivide.z )
		{
			out_color = vec4( 1.0, 0.0, 0.0, 1.0 );
		}
		else
		{
			out_color = ex_color;
		}
	}
	else
	{
		out_color = ex_color;
	}
	*/
/*
	out_diffuse = vec4(1, 0, 0, 1);
	out_normal = vec4(1, 0, 0, 1);
	out_depth = vec4(1, 0, 0, 1);
	out_generic = vec4(1, 0, 0, 1);
*/	
}