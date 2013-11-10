#ifndef _IEVENTMANAGER_H_
#define _IEVENTMANAGER_H_

#include "Event.h"

class IEventManager
{
	public:
		virtual ~IEventManager() {};

		virtual bool registerForEvent( EventID, IEventListener* ) = 0;
		virtual bool unregisterForEvent( EventID, IEventListener* ) = 0;

		virtual bool postEvent( const Event& ) = 0;
};


#endif /* _IEVENTMANAGER_H_ */
