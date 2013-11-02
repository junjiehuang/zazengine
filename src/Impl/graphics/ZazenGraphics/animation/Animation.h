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

class Animation
{
	public:
		friend class AnimationFactory;

		Animation();
		~Animation();

	private:
		AnimNode* m_rootNode;

		double m_ticksPerSecond;
		double m_durationTicks;
};

#endif
