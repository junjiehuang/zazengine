#version 400 core

layout( location = 0 ) out float fragDepth;

in vec3 out_lightDir_world;

subroutine float calculateDepth();

subroutine ( calculateDepth ) float calculateDepthDistance()
{
	// calculate distance 
	float ws_dist = length( out_lightDir_world ); 
 
	// map value to [0;1] by dividing by far plane distance 
	float ws_dist_normalized = ws_dist / 1000.0; 
	gl_FragDepth = ws_dist_normalized;
	return ws_dist_normalized;
}

subroutine ( calculateDepth ) float calculateDepthNDC()
{
	gl_FragDepth = gl_FragCoord.z;
	return gl_FragCoord.z;
}

subroutine uniform calculateDepth calculateDepthSelection;

void main()
{
	fragDepth = calculateDepthSelection();
}