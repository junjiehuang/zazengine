#version 330 core

in vec3 in_vertPos;
in vec3 in_vertNorm;

out vec4 ex_shadowCoord;
//out vec4 ex_depth;
out vec4 ex_normal;

layout(shared) uniform transforms
{
	mat4 model_Matrix;					// 0
	mat4 modelView_Matrix;				// 64
	mat4 modelViewProjection_Matrix;	// 128
	
	mat4 normalsModelView_Matrix;		// 192
	
	mat4 projection_Matrix;				// 256
	mat4 viewing_Matrix;				// 320
	
	mat4 projectionInv_Matrix;			// 384
	mat4 viewingInv_Matrix;				// 448
};

// contains light-direction in 8,9,10
layout(shared) uniform light
{
	mat4 light_ModelMatrix;
	mat4 light_SpaceMatrix;
	mat4 light_SpaceUnitMatrix;
};


void main()
{
	//gl_Position = projection_Matrix * viewing_Matrix * model_Matrix * vec4( in_vertPos, 1.0 );
	gl_Position = modelViewProjection_Matrix * vec4( in_vertPos, 1.0 );
	
	//ex_depth.xy = gl_Position.zw;
	
	ex_normal = normalsModelView_Matrix * vec4( in_vertNorm, 0.0 );
	
	ex_shadowCoord = light_SpaceUnitMatrix * vec4( in_vertPos, 1.0 );
	//ex_shadowCoord = light_SpaceUnitMatrix * model_Matrix * vec4( in_vertPos, 1.0 );
}
