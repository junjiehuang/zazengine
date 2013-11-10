#ifndef _IEVENTLISTENER_H_
#define _IEVENTLISTENER_H_

#include "Event.h"

class IEventListener
{
	public:
		virtual ~IEventListener() {};

		virtual bool sendEvent( Event& e ) = 0;
};

#endif /* _IEVENTLISTENER_H_ */
