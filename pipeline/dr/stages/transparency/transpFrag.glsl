#version 330 core

uniform sampler2D DiffuseTexture;
uniform sampler2D NormalTexture;

uniform sampler2D Background;

in vec4 ex_normal;
in vec2 ex_texCoord;

out vec4 out_color;

const float constantAlpha = 0.5;

void main()
{
    vec3 normal = vec3( texture( NormalTexture, ex_texCoord ) );
    vec2 texCoord = ex_texCoord.xy + normal.xy;

    vec3 bgColor = vec3( texture( Background, texCoord ) );
    vec4 diffuseColor = texture( DiffuseTexture, texCoord );

	diffuseColor.a = constantAlpha;

    // when using the diffuse-color texture as alpha source:
    out_color.rgb = diffuseColor.rgb * diffuseColor.a + bgColor.rgb * ( 1.0 - diffuseColor.a );
    out_color.a = 0.0;
}