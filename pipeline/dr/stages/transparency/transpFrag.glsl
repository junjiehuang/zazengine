#version 330 core

uniform sampler2D DiffuseTexture;
uniform sampler2D NormalTexture;

uniform sampler2D Background;

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

const float constantAlpha = 0.001;

void main()
{
	vec4 bumpTex = 2.0 * texture( NormalTexture, ex_texCoord ) - 1.0;

	vec2 screenCoord = vec2( gl_FragCoord.x / Camera.rectangle.x, gl_FragCoord.y / Camera.rectangle.y );

	vec2 newTexCoord = screenCoord + bumpTex.xy;

	vec4 bgColor = texture( Background, newTexCoord );
    vec4 diffuseColor = texture( DiffuseTexture, ex_texCoord );

	diffuseColor.a = constantAlpha;

	out_color.rgb = diffuseColor.rgb * diffuseColor.a + bgColor.rgb * ( 1.0 - diffuseColor.a );
    out_color.a = 0.0;
}