/*
 * ShadowMappingState.h
 *
 *  Created on: Feb 7, 2011
 *      Author: jonathan
 */

#ifndef SHADOWMAPPINGSTATE_H_
#define SHADOWMAPPINGSTATE_H_

#include "State.h"

class ShadowMappingState : public State
{
 public:
	ShadowMappingState() : State("ShadowMappingState") {};
	 virtual ~ShadowMappingState() {};

	 void enable()
	 {
		 glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);
		 glDepthMask(GL_FALSE);
	 };

	 void disable()
	 {
		 glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
		 glDepthMask(GL_TRUE);
	 };
};

#endif /* SHADOWMAPPINGSTATE_H_ */
