#version 400 core
 
layout ( triangles ) in;
layout ( triangle_strip, max_vertices = 18 ) out;

// transforms the vertex from world-space to the according cube-face model-view-projection
uniform mat4 u_cubeMVPTransforms[ 6 ];

// pass through vertex world-position
out vec4 out_position_world;

void main()
{
	for ( int i = 0; i < 6; ++i )
	{
		for( int j = 0; j < gl_in.length(); ++j )
		{
			// assign layer for each vertex because its a per-vertex attribute
			gl_Layer = i;

			// pass through
			out_position_world = gl_in[ j ].gl_Position;
			// transform vertex from world-position to clip-space of i. cube-face
			gl_Position = u_cubeMVPTransforms[ i ] * out_position_world;

			EmitVertex();
		}

		EndPrimitive();
	}
}