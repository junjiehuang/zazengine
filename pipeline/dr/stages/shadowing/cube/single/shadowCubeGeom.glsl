#version 400 core
 
layout ( triangles ) in;
layout ( triangle_strip, max_vertices = 18 ) out;

uniform mat4 u_cubeMVPTransforms[ 6 ];

out vec4 out_position_world;

void
main()
{
	for ( gl_Layer = 0; gl_Layer < 6; ++gl_Layer )
	{
		for( int i = 0; i < 3; ++i )
		{
			out_position_world = gl_in[ i ].gl_Position;
			gl_Position = u_cubeMVPTransforms[ gl_Layer ] * out_position_world;
			EmitVertex();
		}
		EndPrimitive();
	}
}