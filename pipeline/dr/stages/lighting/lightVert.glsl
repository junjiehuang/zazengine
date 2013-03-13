#version 330 core

in vec3 in_vertPos;

layout(shared) uniform transforms
{
	mat4 model_Matrix;					// 0
	mat4 modelView_Matrix;				// 64
	mat4 modelViewProjection_Matrix;	// 128
	
	mat4 normalsModelView_Matrix;		// 192
	mat4 normalsModel_Matrix;			// 256
	
	mat4 projection_Matrix;				// 320
	mat4 viewing_Matrix;				// 384
	
	mat4 projectionInv_Matrix;			// 448
	mat4 viewingInv_Matrix;				// 512
};

void main()
{
	// only apply projection-matrix because we do orthogonal quad rendering and apply no modeling
	gl_Position = projection_Matrix * vec4( in_vertPos, 1.0 );
}
