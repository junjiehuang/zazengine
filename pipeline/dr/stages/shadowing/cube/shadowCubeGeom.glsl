#version 400 core
 
layout ( triangles ) in;
layout ( triangle_strip, max_vertices = 18 ) out;

layout( shared ) uniform CameraUniforms
{
	vec4 rectangle;

	mat4 modelMatrix;
	mat4 viewMatrix;

	mat4 projectionMatrix;
} Camera;

layout( shared ) uniform TransformUniforms
{
	mat4 modelMatrix;
	mat4 modelViewMatrix;

	mat4 normalsModelViewMatrix;
} Transforms;

uniform mat4 u_cubeModelViewMatrices[ 10 ];

in vec4 out_position_world;

void
main()
{
	for ( int cubeFace = 0; cubeFace < 6; ++cubeFace )
	{
		gl_Layer = cubeFace;

		for( int i = 0; i < gl_in.length(); ++i )
		{
			gl_Position = Transforms.projectionMatrix * u_cubeModelViewMatrices[ cubeFace ] * gl_in[ i ].gl_Position;
	
			EmitVertex();
		}

		EndPrimitive();
	}
}