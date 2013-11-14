#version 400 core
 
layout ( triangles ) in;
layout ( triangle_strip, max_vertices = 3 ) out;

layout( shared ) uniform TransformUniforms
{
	mat4 modelViewMatrix;
	mat4 projectionMatrix;

	mat4 normalsModelViewMatrix;
} Transforms;

subroutine void layerRendering( int vertexIndex );

subroutine ( layerRendering ) void layerRenderingCube( int vertexIndex )
{
	gl_Position = Transforms.projectionMatrix * gl_in[ vertexIndex ].gl_Position;
}

subroutine ( layerRendering ) void layerRenderingPlanar( int vertexIndex )
{
	gl_Position = Transforms.projectionMatrix * gl_in[ vertexIndex ].gl_Position;
}

subroutine uniform layerRendering layerRenderingSelection;

 void main()
{
	for( int i = 0; i < gl_in.length(); i++ )
	{
		layerRenderingSelection( i );

		EmitVertex();
	}
}