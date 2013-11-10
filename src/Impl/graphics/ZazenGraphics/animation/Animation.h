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

		Animation( double, double );
		~Animation();

		// calculate bone-transformations for current frame-time
		void update();

		// initializes this animation with a sekeleton (separation of meshes and animations!)
		void initSkeleton( MeshNode* );

		// gets the master-list of the bones-transformations in the correct order (bone-indices are very imporant!)
		const std::vector<glm::mat4>& getBoneTransforms() const { return this->m_boneTransforms; };

	private:
		// an animation-key holds its time-frame and its data
		template <typename T> struct AnimationKey {
			double m_time;
			T m_value;
		};

		// an animation channel consists of a list of position-keys, rotation-keys and scaling-keys
		struct AnimationChannel {
			std::vector<AnimationKey<glm::vec3>> m_positionKeys;
			std::vector<AnimationKey<glm::quat>> m_rotationKeys;
			std::vector<AnimationKey<glm::vec3>> m_scalingKeys;
		};

		// the animation-skeleton part: builds the animation-skeleton hierarchy
		struct AnimationSkeletonPart {
			glm::mat4 m_localTransform; // local transform of node: is the model-matrix of the according mesh-node
			const glm::mat4* m_boneOffset; // bone-offset of this part: is not null if this part has a bone
			AnimationChannel* m_animationChannel; // the animation-channel of this part: is not null if this part has an animation-channel
			std::vector<AnimationSkeletonPart*> m_children; // children of this part
		};

		// the root of the animation-skeleton
		AnimationSkeletonPart* m_skeletonRoot; 
		// all animation-channels this animation has, maped by their node-names
		std::map<std::string, AnimationChannel> m_animationChannels;

		// the master-list of the bone-transformations which will be re-calculated during each call to update
		std::vector<glm::mat4> m_boneTransforms;

		// current frame-time of this animation (between 0.0 and m_durationInSec)
		double m_currentTime;
		// current frame of this animation (between 0.0 and m_durationTicks)
		double m_currentFrame;
		// store indices of last channels of position, rotation and scaling interpolations to reduce search-overhead from O(n) to O(1)
		unsigned int m_lastPosChannelIndex;
		unsigned int m_lastRotChannelIndex;
		unsigned int m_lastScaleChannelIndex;

		// the frequency of this animation: how many ticks do we advance per second
		double m_ticksPerSecond;
		// duration of the whole animation in ticks (warning: is floating-point!)
		double m_durationTicks;
		// duration of the whole animation in seconds (warning: is floating-point!)
		double m_durationInSec;

		AnimationSkeletonPart* buildAnimationSkeleton( MeshNode* );

		void animateSkeleton( AnimationSkeletonPart*, const glm::mat4& );

		void interpolatePosition( const std::vector<AnimationKey<glm::vec3>>&, glm::mat4& );
		void interpolateRotation( const std::vector<AnimationKey<glm::quat>>&, glm::mat4& );
		void interpolateScaling( const std::vector<AnimationKey<glm::vec3>>&, glm::mat4& );

		template <typename T> void findFirstAndNext( const std::vector<AnimationKey<T>>&, AnimationKey<T>&, AnimationKey<T>&, unsigned int& );

		void cleanUpSkeletonPart( AnimationSkeletonPart* );
};

#endif
