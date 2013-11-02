#include "AssImpUtils.h"

void
AssImpUtils::assimpMatToGlm( const aiMatrix4x4& assImpMat, glm::mat4& glmMat )
{
	glmMat[ 0 ][ 0 ] = assImpMat.a1;
	glmMat[ 0 ][ 1 ] = assImpMat.a2;
	glmMat[ 0 ][ 2 ] = assImpMat.a3;
	glmMat[ 0 ][ 3 ] = assImpMat.a4;

	glmMat[ 1 ][ 0 ] = assImpMat.b1;
	glmMat[ 1 ][ 1 ] = assImpMat.b2;
	glmMat[ 1 ][ 2 ] = assImpMat.b3;
	glmMat[ 1 ][ 3 ] = assImpMat.b4;

	glmMat[ 2 ][ 0 ] = assImpMat.c1;
	glmMat[ 2 ][ 1 ] = assImpMat.c2;
	glmMat[ 2 ][ 2 ] = assImpMat.c3;
	glmMat[ 2 ][ 3 ] = assImpMat.c4;

	glmMat[ 3 ][ 0 ] = assImpMat.d1;
	glmMat[ 3 ][ 1 ] = assImpMat.d2;
	glmMat[ 3 ][ 2 ] = assImpMat.d3;
	glmMat[ 3 ][ 3 ] = assImpMat.d4;
}

void
AssImpUtils::assimpMatToGlm( const aiMatrix3x3& assImpMat, glm::mat3& glmMat )
{
	glmMat[ 0 ][ 0 ] = assImpMat.a1;
	glmMat[ 0 ][ 1 ] = assImpMat.a2;
	glmMat[ 0 ][ 2 ] = assImpMat.a3;

	glmMat[ 1 ][ 0 ] = assImpMat.b1;
	glmMat[ 1 ][ 1 ] = assImpMat.b2;
	glmMat[ 1 ][ 2 ] = assImpMat.b3;

	glmMat[ 2 ][ 0 ] = assImpMat.c1;
	glmMat[ 2 ][ 1 ] = assImpMat.c2;
	glmMat[ 2 ][ 2 ] = assImpMat.c3;
}

void
AssImpUtils::assimpQuatToGlm( const aiQuaternion& assImpQuat, glm::quat& glmQuat )
{
	glmQuat.x = assImpQuat.x;
	glmQuat.y = assImpQuat.y;
	glmQuat.z = assImpQuat.z;
	glmQuat.w = assImpQuat.w;
}

void
AssImpUtils::assimpVecToGlm( const aiVector3D& assImpVec, glm::vec3& glmVec )
{
	glmVec.x = assImpVec.x;
	glmVec.y = assImpVec.y;
	glmVec.z = assImpVec.z;
}
