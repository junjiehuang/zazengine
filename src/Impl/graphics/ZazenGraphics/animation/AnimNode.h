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
#include <glm/ext.hpp>

#include <string>
#include <vector>

class AnimNode
{
	public:
		friend class AnimationFactory;

		// TODO: refine when implementation finished: when only one field: collapse
		struct Bone {
			std::string m_name;
			glm::mat4 m_meshToBoneTransf;
		};

		template <typename T> struct AnimKey {
			double m_time;
			T m_value;
		};

		struct AnimChannel {
			std::vector<AnimKey<glm::vec3>> m_positionKeys;
			std::vector<AnimKey<glm::quat>> m_rotationKeys;
			std::vector<AnimKey<glm::vec3>> m_scalingKeys;
		};

		AnimNode();
		~AnimNode();

		const std::string& getName() const { return this->m_name; };

		const glm::mat4& getTransform() const { return this->m_transform; };
		const std::vector<AnimNode*>& getChildren() const { return this->m_children; };

		const Bone* const getBone() { return this->m_bone; };
		const AnimChannel* const getChannel() { return this->m_animChannel; };

	private:
		std::string m_name;
		glm::mat4 m_transform;

		Bone* m_bone;
		AnimChannel* m_animChannel;

		std::vector<AnimNode*> m_children;
};

#endif
