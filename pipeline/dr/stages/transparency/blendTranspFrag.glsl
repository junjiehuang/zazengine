#version 330 core

uniform sampler2D BackgroundPass;
uniform sampler2D TransparentIntermediate;

in vec2 ex_texCoord;

out vec4 out_color;

void main()
{
    vec4 transpColor = texture( TransparentIntermediate, ex_texCoord );
    vec4 opaqueColor = texture( BackgroundPass, ex_texCoord );

    out_color.rgb = opaqueColor.rgb * transpColor.a + transpColor.rgb;
    out_color.a = 1.0;
}