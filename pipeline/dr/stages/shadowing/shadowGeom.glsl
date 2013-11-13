#version 400 core
 
layout ( triangles ) in;
layout ( triangle_strip, max_vertices = 3 ) out;

layout( shared ) uniform TransformUniforms
{
	mat4 modelViewMatrix;
	mat4 projectionMatrix;

	mat4 normalsModelViewMatrix;
} Transforms;

subroutine vec4 layerRendering( int vertexIndex );

subroutine ( layerRendering ) vec4 layerRenderingCube( int vertexIndex )
{
	return Transforms.projectionMatrix * Transforms.modelViewMatrix * gl_in[ vertexIndex ].gl_Position;
}

subroutine ( layerRendering ) vec4 layerRenderingPlanar( int vertexIndex )
{
	// OPTIMIZE: premultiply projection & modelView on CPU 
	return Transforms.projectionMatrix * Transforms.modelViewMatrix * gl_in[ vertexIndex ].gl_Position;
}

subroutine uniform layerRendering layerRenderingSelection;

 void main()
{
	for( int i = 0; i < gl_in.length(); i++ )
	{
		// copy attributes
		gl_Position = layerRenderingSelection( i );
 
		// done with the vertex
		EmitVertex();
	}
}