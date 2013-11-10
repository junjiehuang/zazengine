#ifndef _EVENTMANAGER_H_
#define _EVENTMANAGER_H_

#include <core/IEventListener.h>
#include <core/IEventManager.h>

#include <map>
#include <list>

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

		void broadCast( Event& );
};

#endif /* _EVENTMANAGER_H_ */
