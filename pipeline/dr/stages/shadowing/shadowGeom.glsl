#version 400 core
 
layout ( triangles ) in;
layout ( triangle_strip, max_vertices = 3 ) out;

layout( shared ) uniform TransformUniforms
{
	mat4 modelViewMatrix;
	mat4 projectionMatrix;

	mat4 normalsModelViewMatrix;
} Transforms;

uniform mat4 u_cubeModelViewMatrices[ 6 ];

subroutine void layeredRendering( int vertexIndex );

subroutine ( layeredRendering ) void layeredRenderingCube( int vertexIndex )
{
	for ( int i = 0; i < 6; i++ )
	{
		gl_Layer = i;

		gl_Position = Transforms.projectionMatrix * u_cubeModelViewMatrices[ i ] * gl_in[ vertexIndex ].gl_Position;
	
		EmitVertex();
	}
}

subroutine ( layeredRendering ) void layeredRenderingPlanar( int vertexIndex )
{
	gl_Position = Transforms.projectionMatrix * Transforms.modelViewMatrix * gl_in[ vertexIndex ].gl_Position;
}

subroutine uniform layeredRendering layeredRenderingSelection;

 void main()
{
	for( int i = 0; i < gl_in.length(); i++ )
	{
		layeredRenderingSelection( i );

		EmitVertex();
	}
}