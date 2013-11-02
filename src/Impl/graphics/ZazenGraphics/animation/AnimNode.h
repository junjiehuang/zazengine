/*
 *  AnimNode.cpp
 *  zaZengine
 *
 *  Created by Jonathan Thaler on 02. Nov. 13.
 *
 */

#ifndef _ANIMNODE_H_
#define _ANIMNODE_H_

#include <glm/glm.hpp>

#include <string>
#include <vector>

class AnimNode
{
	public:
		friend class AnimationFactory;

		// TODO: refine when implementation finished: when only one field: collapse
		struct Bone {
			glm::mat4 m_meshToBoneTransf;
		};

		template <typename T> struct AnimKey {
			double m_time;
			T m_value;
		};

		struct AnimChannel {
			std::vector<AnimKey<glm::vec3>> m_positionKeys;
			std::vector<AnimKey<glm::mat3>> m_rotationKeys;
			std::vector<AnimKey<glm::vec3>> m_scalingKeys;
		};

		AnimNode();
		~AnimNode();

		const std::string& getName() const { return this->m_name; };
		const glm::mat4& getTransform() const { return this->m_transform; };
		const std::vector<AnimNode*>& getChildren() const { return this->m_children; };

	private:
		std::string m_name;
		glm::mat4 m_transform;

		Bone* m_bone;
		AnimChannel* m_animChannel;

		std::vector<AnimNode*> m_children;
};

#endif
