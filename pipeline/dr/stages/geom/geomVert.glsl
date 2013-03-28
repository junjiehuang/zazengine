#version 330 core

in vec3 in_vertPos;
in vec3 in_vertNorm;
in vec2 in_texCoord;

out vec4 ex_position;
out vec4 ex_normal;
out vec2 ex_texCoord;

layout( shared ) uniform transforms
{
	mat4 modelView_Matrix;				// 0
	mat4 projection_Matrix;				// 64

	mat4 normalsModelView_Matrix;		// 128
};

void main()
{
	// NOTE: that due to multiplication with modelView only and without projection
	// only interpoation between vertices will happen but no perspective division
	ex_position = modelView_Matrix * vec4( in_vertPos, 1.0 );
	ex_normal = normalsModelView_Matrix * vec4( in_vertNorm, 0.0 );
	ex_texCoord = in_texCoord;

	// OPTIMIZE: premultiply projection & modelView on CPU 
	gl_Position = projection_Matrix * modelView_Matrix * vec4( in_vertPos, 1.0 );
}
