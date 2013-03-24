#version 330 core

in vec3 in_vertPos;
in vec3 in_vertNorm;
in vec2 in_texCoord;

out vec4 ex_position;
out vec4 ex_normal;
out vec2 ex_texCoord;

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
	// lighting is applied in Eye-Coordinates (after model-view is applied)
	ex_position = modelView_Matrix * vec4( in_vertPos, 1.0 );
	ex_normal = normalsModelView_Matrix * vec4( in_vertNorm, 0.0 );
	ex_texCoord = in_texCoord;

	gl_Position = modelViewProjection_Matrix * vec4( in_vertPos, 1.0 );
}
