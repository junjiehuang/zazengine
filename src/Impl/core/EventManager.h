/*
 * EventManager.h
 *
 *  Created on: 02.07.2010
 *      Author: Jonathan Thaler
 */

#ifndef EVENTMANAGER_H_
#define EVENTMANAGER_H_

#include <core/IEventListener.h>
#include <core/IEventManager.h>

#include <map>
#include <list>

#include <boost/thread.hpp>

/**
 * processQueue is guaranteed to be only called from the main thread.
 */

class EventManager : public IEventManager
{
	public:
		EventManager();
		virtual ~EventManager();

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

		boost::mutex regMutex;
		boost::mutex queueMutex;

		void broadCast( Event& );

};

#endif /* EVENTMANAGER_H_ */
