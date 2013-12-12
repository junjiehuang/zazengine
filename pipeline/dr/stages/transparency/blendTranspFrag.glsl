#version 330 core

uniform sampler2D Background;
uniform sampler2D Transparent;

in vec2 ex_texCoord;

layout( location = 0 ) out vec4 out_color;

void main()
{
    vec4 opaqueColor = texture( Background, ex_texCoord );
	vec4 transpColor = texture( Transparent, ex_texCoord );

    out_color.rgb = opaqueColor.rgb * transpColor.a + transpColor.rgb;
    out_color.a = 1.0;
}