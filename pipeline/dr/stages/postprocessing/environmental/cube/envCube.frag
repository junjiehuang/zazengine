#version 430 core

// defines the input-interface block from the vertex-shader
in IN_OUT_BLOCK
{
	vec3 normal;
	vec3 incident;
} IN_OUT;

layout( location = 0 ) out vec4 out_color;

layout( binding = 20 ) uniform samplerCube EnvironmentMap;

// THE CONFIGURATION OF THE CURRENTLY ENVIRONMENTAL CUBE MATERIAL
layout( shared ) uniform EnvironmentalCubeMaterialUniforms
{
	// the configuration of the material. x = blending-factor, y = refraction-factor
	vec2 params;
} EnvironmentalCubeMaterial;

// TODO: make configurable
const float etaR = 1.14;
const float etaG = 1.12;
const float etaB = 1.10;
const float fresnelPower = 2.0;
const float F = ( ( 1.0 - etaG ) * ( 1.0 - etaG ) ) / ( ( 1.0 + etaG ) * ( 1.0 + etaG ) );

void main()
{
    float ratio = F + ( 1.0 - F ) * pow( 1.0 - dot( -IN_OUT.incident, IN_OUT.normal ), fresnelPower );

    vec3 refractR = vec3( refract( IN_OUT.incident, IN_OUT.normal, etaR ) );
    vec3 refractG = vec3( refract( IN_OUT.incident, IN_OUT.normal, etaG) );
    vec3 refractB = vec3( refract( IN_OUT.incident, IN_OUT.normal, etaB ) );

    vec3 reflectDir = vec3( reflect( IN_OUT.incident, IN_OUT.normal ) );

    vec3 refractColor;
    refractColor.r = texture( EnvironmentMap, refractR ).r;
    refractColor.g  = texture( EnvironmentMap, refractG ).g;
    refractColor.b  = texture( EnvironmentMap, refractB ).b;

    vec3 reflectColor = texture(EnvironmentMap, reflectDir).rgb;

	// TODO: something is still wrong when objects are scaled

	out_color.rgb = mix(refractColor, reflectColor, ratio);
    out_color.a = 0.0;
}