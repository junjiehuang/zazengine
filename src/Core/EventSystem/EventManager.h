/*
 * EventManager.h
 *
 *  Created on: 02.07.2010
 *      Author: joni
 */

#ifndef EVENTMANAGER_H_
#define EVENTMANAGER_H_

#include "Event.h"
#include "IEventListener.h"

#include <map>
#include <list>

/**
 * processQueue is guaranteed to be only called from the main thread.
 */

class EventManager
{
	public:
		EventManager();
		~EventManager();

		bool registerForEvent(EventID, IEventListener*);
		bool unregisterForEvent(EventID, IEventListener*);

		bool postEvent( const Event& );

		void processQueue();

	private:
		struct Registration {
			EventID eventID;
			IEventListener* listener;
			bool unregister;
		};

		static long instances;

		std::list<Event> eventQueue;
		std::list<Registration> regQueue;

		std::map<EventID, std::list<IEventListener*>* > eventListeners;

		void broadCast( Event& );

};

#endif /* EVENTMANAGER_H_ */
