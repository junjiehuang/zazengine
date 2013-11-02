/*
 * AssImpUtils.h
 *
 *  Created on: 02. Nov. 2013
 *      Author: jonathan
 */

#ifndef ASSIMP_UTILS
#define ASSIMP_UTILS

#include <assimp/scene.h>
#include <glm/ext.hpp>

class AssImpUtils
{
	public:
		static void assimpMatToGlm( const aiMatrix4x4&, glm::mat4& );
		static void assimpMatToGlm( const aiMatrix3x3&, glm::mat3& );

		static void assimpQuatToGlm( const aiQuaternion&, glm::quat& );

		static void assimpVecToGlm( const aiVector3D&, glm::vec3& );
};

#endif
