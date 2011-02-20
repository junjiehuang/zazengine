#version 330 core

in vec3 in_vertPos;

layout(shared) uniform mvp_transform
{
	mat4 model_Matrix;					// 0
	mat4 modelView_Matrix;				// 64
	mat4 modelViewProjection_Matrix;	// 128
	
	mat4 normalsModelView_Matrix;		// 196
	
	mat4 projection_Matrix;				// 254
	mat4 viewing_Matrix;				// 320
	
	mat4 projectionInv_Matrix;			// 384
	mat4 viewingInv_Matrix;				// 448
};

void main()
{
	gl_Position = modelViewProjection_Matrix * vec4( in_vertPos, 1.0 );
}
