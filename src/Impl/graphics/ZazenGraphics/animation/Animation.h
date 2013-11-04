/*
 *  Animation.cpp
 *  zaZengine
 *
 *  Created by Jonathan Thaler on 30.10.13.
 *
 */

#ifndef _ANIMATION_H_
#define _ANIMATION_H_

#include "../geometry/MeshNode.h"

#include <glm/glm.hpp>
#include <glm/ext.hpp>

#include <string>
#include <vector>
#include <map>

class Animation
{
	public:
		friend class AnimationFactory;

		struct AnimationBone {
			std::string m_name;
			glm::mat4 m_meshToBoneTransf;
		};

		template <typename T> struct AnimationKey {
			double m_time;
			T m_value;
		};

		struct AnimationChannel {
			std::vector<AnimationKey<glm::vec3>> m_positionKeys;
			std::vector<AnimationKey<glm::quat>> m_rotationKeys;
			std::vector<AnimationKey<glm::vec3>> m_scalingKeys;
		};

		struct AnimationNode {
			std::string m_name;
			glm::mat4 m_transform;
			AnimationChannel m_animationChannels;
		};

		Animation( double, double );
		~Animation();

		void animate( MeshNode* );

		const std::vector<glm::mat4>& getTransforms() const { return this->m_transforms; };

	private:
		struct AnimationSkeleton {
			glm::mat4 m_transform;
			AnimationNode* m_animationNode;
			AnimationBone* m_animationBone;
			std::vector<AnimationSkeleton*> m_children;
		};

		AnimationSkeleton* m_skeleton;
		std::map<std::string, AnimationNode*> m_animationNodes;
		std::map<std::string, AnimationBone*> m_animationBones;

		std::vector<glm::mat4> m_transforms;

		double m_currentTime;
		double m_currentFrame;
		unsigned int m_lastPosChannelIndex;
		unsigned int m_lastRotChannelIndex;
		unsigned int m_lastScaleChannelIndex;

		double m_ticksPerSecond;
		double m_durationTicks;
		double m_durationInSec;

		void buildAnimationSkeleton( MeshNode* );

		void animateSkeleton( AnimationSkeleton*, const glm::mat4& );

		void interpolatePosition( const std::vector<AnimationKey<glm::vec3>>&, glm::mat4& );
		void interpolateRotation( const std::vector<AnimationKey<glm::quat>>&, glm::mat4& );
		void interpolateScaling( const std::vector<AnimationKey<glm::vec3>>&, glm::mat4& );

		template <typename T> void findFirstAndNext( const std::vector<AnimationKey<T>>&, AnimationKey<T>&, AnimationKey<T>&, unsigned int& );
};

#endif
