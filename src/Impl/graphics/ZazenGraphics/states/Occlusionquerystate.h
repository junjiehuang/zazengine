#ifndef OCCLUSIONQUERYSTATE_H_
#define OCCLUSIONQUERYSTATE_H_

#include "State.h"

class OcclusionQueryState : public State
{
 public:
	 OcclusionQueryState() : State("OcclusionQueryState") {};
	 virtual ~OcclusionQueryState() {};
	 
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

#endif /*OCCLUSIONQUERYSTATE_H_*/
