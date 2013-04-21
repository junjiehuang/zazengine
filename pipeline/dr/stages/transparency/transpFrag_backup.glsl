#version 330 core

uniform sampler2D NormalTexture;
uniform sampler2D DiffuseTexture;
uniform sampler2D Background;

in vec4 ex_normal;
in vec2 ex_texCoord;

out vec4 out_color;

void main()
{
    // fetch normal from normalmap
    vec4 normal = texture( NormalMap, ex_texCoord );

    // perturb texture coordinate
    vec2 texCoord = ex_texCoord.xy + normal.xy;
    // access the lighting-stage result as background
    vec4 lStageBgColor = texture( Background, texCoord );

    // a is 1 when another transparency is already accumulated at this fragment, 
    // in this case lStageBgColor will not be added because of multiplication by 0 but the transparency accumulated will be used
    // a is 0 when no transparency has been accumulated at this fragment, 
    // in this case lStageBgColor will be used because the content of transpAccColor is all 0
    vec4 bgColor = lStageBgColor * ( 1.0 - transpAccColor.a ) + transpAccColor;

    // apply transparency-blending
    vec3 diffuseColor = texture( DiffuseColor, texCoord );

    // when using the diffuse-color texture as alpha source:
    out_color.rgb = bgColor * ( 1.0 - diffuseColor.a ) + diffuseColor.rgb * diffuseColor.a;

    // when using a constant for the whole object as alpha-source:
    // 10% transparent means alpha = 0.9 =>
    // bgColor contributes 10% to the color
    // diffuseColor contributes 90% of the color
    out_color.rgb = bgColor * ( 1.0 - alpha ) + diffuseColor.rgb * alpha;
    out_color.a = 1.0;
}