#version 400 core

uniform samplerCube EnvironmentMap;

// defines the output-interface block to the fragment-shader
in VS_TO_FS_OUT
{
	vec3 normal;
	vec3 incident;
} VS_TO_FS;

layout( location = 0 ) out vec4 out_color;

// TODO: make configurable
const float etaR = 1.14;
const float etaG = 1.12;
const float etaB = 1.10;
const float fresnelPower = 2.0;
const float F = ( ( 1.0 - etaG ) * ( 1.0 - etaG ) ) / ( ( 1.0 + etaG ) * ( 1.0 + etaG ) );

void main()
{
    float ratio = F + ( 1.0 - F ) * pow( 1.0 - dot( -VS_TO_FS.incident, VS_TO_FS.normal ), fresnelPower );

    vec3 refractR = vec3( refract( VS_TO_FS.incident, VS_TO_FS.normal, etaR ) );
    vec3 refractG = vec3( refract( VS_TO_FS.incident, VS_TO_FS.normal, etaG) );
    vec3 refractB = vec3( refract( VS_TO_FS.incident, VS_TO_FS.normal, etaB ) );

    vec3 reflectDir = vec3( reflect( VS_TO_FS.incident, VS_TO_FS.normal ) );

    vec3 refractColor;
    refractColor.r = texture( EnvironmentMap, refractR ).r;
    refractColor.g  = texture( EnvironmentMap, refractG ).g;
    refractColor.b  = texture( EnvironmentMap, refractB ).b;

    vec3 reflectColor = texture(EnvironmentMap, reflectDir).rgb;

	out_color.rgb = mix(refractColor, reflectColor, ratio);
    out_color.a = 0.0;
}