#version 330 core

in vec3 in_vertPos;

layout( shared ) uniform transforms
{
	mat4 modelView_Matrix;				// 0
	mat4 projection_Matrix;				// 64

	mat4 normalsModelView_Matrix;		// 128
};

void main()
{
	// HINT: only apply projection-matrix because we do orthogonal quad rendering and apply no modeling
	gl_Position = projection_Matrix * vec4( in_vertPos, 1.0 );
}
