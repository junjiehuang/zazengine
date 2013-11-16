#version 400 core
 
layout ( triangles ) in;
layout ( triangle_strip, max_vertices = 3 ) out;

layout( shared ) uniform TransformUniforms
{
	mat4 modelViewMatrix;
	mat4 projectionMatrix;

	mat4 normalsModelViewMatrix;
} Transforms;

 void main()
{
	for( int i = 0; i < gl_in.length(); i++ )
	{
		// apply model-view and projection matrix in geometry-shader
		// OPTIMIZE: do this in vertex-shader in case of planar-shadow mapping
		gl_Position = Transforms.projectionMatrix * Transforms.modelViewMatrix * gl_in[ i ].gl_Position;

		EmitVertex();
	}
}