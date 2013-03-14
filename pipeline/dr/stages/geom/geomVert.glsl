#version 330 core

in vec3 in_vertPos;
in vec3 in_vertNorm;

out vec4 ex_normal;

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
	gl_Position = modelViewProjection_Matrix * vec4( in_vertPos, 1.0 );
	// lighting is applied in world-space (model-space)
	ex_normal = normalsModel_Matrix * vec4( in_vertNorm, 0.0 );
}
