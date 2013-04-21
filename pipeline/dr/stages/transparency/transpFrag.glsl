#version 330 core

uniform sampler2D DiffuseTexture;
uniform sampler2D NormalTexture;

uniform sampler2D Background;

in vec4 ex_normal;
in vec2 ex_texCoord;

out vec4 out_color;

void main()
{
    // fetch normal from normalmap
    vec3 normal = vec3( texture( NormalTexture, ex_texCoord ) );

    // perturb texture coordinate
    vec2 texCoord = ex_texCoord.xy + normal.xy;

    // access the lighting-stage result as background
    vec3 bgColor = vec3( texture( Background, texCoord ) );

    // apply transparency-blending
    vec3 diffuseColor = texture( DiffuseTexture, texCoord );

    // when using the diffuse-color texture as alpha source:
    out_color.rgb = bgColor * 0.5 + diffuseColor.rgb * 0.5;
    out_color.a = 1.0;
}