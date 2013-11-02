/*
 *  Animation.cpp
 *  zaZengine
 *
 *  Created by Jonathan Thaler on 30.10.13.
 *
 */

#ifndef _ANIMATION_H_
#define _ANIMATION_H_

#include "AnimNode.h"

#include "../Program/Program.h"

class Animation
{
	public:
		friend class AnimationFactory;

		Animation( double, double );
		~Animation();

		void perform( double );
		void updateToProgram( Program* );

	private:
		AnimNode* m_rootNode;

		std::vector<glm::mat4> m_transforms;

		double m_currentTime;
		double m_currentFrame;

		double m_ticksPerSecond;
		double m_durationTicks;
		double m_durationInSec;

		void performRecursive( AnimNode*, const glm::mat4& );

		void interpolatePosition( const std::vector<AnimNode::AnimKey<glm::vec3>>&, glm::mat4& );
		void interpolateRotation( const std::vector<AnimNode::AnimKey<glm::quat>>&, glm::mat4& );
		void interpolateScaling( const std::vector<AnimNode::AnimKey<glm::vec3>>&, glm::mat4& );

		template <typename T> void findFirstAndNext( const std::vector<AnimNode::AnimKey<T>>&, AnimNode::AnimKey<T>&, AnimNode::AnimKey<T>& );
};

#endif
