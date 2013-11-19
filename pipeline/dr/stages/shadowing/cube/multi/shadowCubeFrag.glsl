#version 400 core

layout( shared ) uniform LightUniforms
{
	vec4 config;

	mat4 modelMatrix;
	mat4 spaceUniformMatrix;
} Light;

in vec4 out_position_world;

void
main()
{
	// light-position in world-space
	vec3 lightPos_world = Light.modelMatrix[ 3 ].xyz;
	vec3 lightDir_world = out_position_world.xyz - lightPos_world;

	// calculate distance 
	float ws_dist = length( lightDir_world ); 
 
	// map value to [0;1] by dividing by far plane distance 
	float ws_dist_normalized = ws_dist / 1000.0; 
	gl_FragDepth = ws_dist_normalized;
}
