/*
 * IEventManager.h
 *
 *  Created on: 7 Aug 2011
 *      Author: jonathan
 */

#ifndef IEVENTMANAGER_H_
#define IEVENTMANAGER_H_

#include "Event.h"

class IEventManager
{
	public:
		virtual ~IEventManager() {};

		virtual bool registerForEvent( EventID, IEventListener* ) = 0;
		virtual bool unregisterForEvent( EventID, IEventListener* ) = 0;

		virtual bool postEvent( const Event& ) = 0;
};


#endif /* IEVENTMANAGER_H_ */
