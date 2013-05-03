#version 330 core

uniform sampler2D DiffuseTexture;
uniform sampler2D NormalTexture;

uniform sampler2D Background;
uniform sampler2D BackgroundDepth;

in vec4 ex_pos;
in vec4 ex_normal;
in vec2 ex_texCoord;

out vec4 out_color;

layout( shared ) uniform CameraUniforms
{
	mat4 modelMatrix;
	mat4 viewMatrix;
	
	vec4 rectangle;
} Camera;

const float blendingFactor = 0.5;
const float refractFactor = 0.05;

void main()
{
	vec4 refractNormal = 2.0 * texture( NormalTexture, ex_texCoord ) - 1.0;

	vec2 screenTexCoord = vec2( gl_FragCoord.x / Camera.rectangle.x, gl_FragCoord.y / Camera.rectangle.y );
	vec2 refractTexCoord = screenTexCoord + refractNormal.xy * refractFactor;

	vec4 bgColorRefract = texture( Background, refractTexCoord );
	vec4 bgColorScreen = texture( Background, screenTexCoord );

    vec4 materialColor = texture( DiffuseTexture, ex_texCoord );
	
	vec3 refractFragDepth = texture( BackgroundDepth, refractTexCoord );

	if ( refractFragDepth.r < gl_FragDepth )
	{
		out_color.rgb = materialColor.rgb * blendingFactor + bgColorScreen.rgb * ( 1.0 - blendingFactor );
	}
	else
	{
		out_color.rgb = materialColor.rgb * blendingFactor + bgColorRefract.rgb * ( 1.0 - blendingFactor );
	}

	out_color.rgb = texture( BackgroundDepth, ex_texCoord );
    out_color.a = 0.0;
}