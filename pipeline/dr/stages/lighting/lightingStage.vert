#version 430 core

// final light-pass mesh has only vertex-positions which are already in NDC
layout( location = 0 ) in vec3 in_vertPos;

// defines the output-interface block to the fragment-shader
out IN_OUT_BLOCK
{
	// the screen-coordinate interpolated for each fragment - is used to do the texture-fetches of the MRTs
	vec2 screenTexCoord;
	// the normalized-device-coordinates
	vec3 ndc;
} OUT;

// THE CAMERA CONFIGURATION FOR THE CURRENT VIEW
// NOTE: THIS HAS TO BE THE CAMERA THE GEOMETRY-STAGE WAS RENDERED WITH
layout( shared ) uniform CameraUniforms
{
	// the resolution of the viewport, z&w are the reciprocal values
	vec4 viewport;	
	// the near- (x) and far-plane distances (y)
	vec2 nearFar;
	// the symetric frustum: right (left=-right) and top (bottom=-top)
	vec2 frustum;

	// the model-matrix of the camera (orienation within world-space)
	mat4 modelMatrix;
	// the view-matrix of the camera to apply to the objects to transform to view/eye/camera-space (is its inverse model-matrix)
	mat4 viewMatrix;
	// the projection-matrix of the camera
	mat4 projectionMatrix;
} Camera;

// need to get the texturesize for calculating relative screen-texture coords
layout( binding = 0 ) uniform sampler2D DiffuseMap;

void main()
{
	// the final pass for each light is always done using a full-screen-quad in NDC
	gl_Position = vec4( in_vertPos, 1.0 );

	// just write the values through as the vertex positions are already in NDC
	OUT.ndc = gl_Position.xyz;
	
	// need to transform from NDC which is in range [-1, 1] to texture-space [0, 1]
	vec2 viewportTextureSpace = ( OUT.ndc.xy + 1.0 ) * 0.5;
	// need to know texture-size of the MRT textures which should match, just take DepthMap as reference
	ivec2 textureDimensions = textureSize( DiffuseMap, 0 );
	// calculate the relative screen-texture coordinates
	// this is necessary because in dynamic environmental mapping the viewport-size can be smaller than the resolution of the MRT
	// thus only a part of the MRTs is rendered to and thus we need to adjust the texture-coordinates accordingly
	OUT.screenTexCoord = vec2( ( viewportTextureSpace.x * Camera.viewport.x ) / textureDimensions.x, ( viewportTextureSpace.y * Camera.viewport.y ) / textureDimensions.y );
}