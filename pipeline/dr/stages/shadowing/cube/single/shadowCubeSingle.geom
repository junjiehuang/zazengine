#version 430 core
 
layout ( triangles ) in;
layout ( triangle_strip, max_vertices = 18 ) out;

// defines the output-interface block to the fragment-shader
out IN_OUT_BLOCK
{
	vec4 posWorld;
} OUT;

// transforms the vertex from world-space to the according cube-face model-view-projection
uniform mat4 u_cubeMVPTransforms[ 6 ];

void main()
{
	for ( int i = 0; i < 6; ++i )
	{
		for( int j = 0; j < gl_in.length(); ++j )
		{
			// assign layer for each vertex because its a per-vertex attribute
			gl_Layer = i;

			// pass through
			OUT.posWorld = gl_in[ j ].gl_Position;
			// transform vertex from world-position to clip-space of i. cube-face
			gl_Position = u_cubeMVPTransforms[ i ] * OUT.posWorld;

			EmitVertex();
		}

		EndPrimitive();
	}
}