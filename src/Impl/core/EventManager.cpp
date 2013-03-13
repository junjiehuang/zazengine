/*
 * EventManager.cpp
 *
 *  Created on: 02.07.2010
 *      Author: Jonathan Thaler
 */

#include "EventManager.h"

#include <errno.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <string.h>

#include <iostream>
#include <stdexcept>
#include <sstream>

using namespace std;

long EventManager::instances = 0;

EventManager::EventManager()
{
	EventManager::instances++;

	stringstream listenerSemName;
	stringstream queueSemName;

	listenerSemName << "EventManager" << EventManager::instances << "ListenerSem";
	queueSemName << "EventManager" << EventManager::instances << "QueueSem";
}

EventManager::~EventManager()
{
}

bool
EventManager::registerForEvent( EventID eventID, IEventListener* listener )
{
	this->regMutex.lock();

	Registration reg;
	reg.listener = listener;
	reg.eventID = eventID;
	reg.unregister = false;

	this->regQueue.push_back( reg );

	this->regMutex.unlock();

	return true;
}

bool
EventManager::unregisterForEvent( EventID eventID, IEventListener* listener )
{
	this->regMutex.lock();

	Registration reg;
	reg.listener = listener;
	reg.eventID = eventID;
	reg.unregister = true;

	this->regQueue.push_back( reg );

	this->regMutex.unlock();

	return true;
}

bool
EventManager::postEvent( const Event& e )
{
	this->queueMutex.lock();

	this->eventQueue.push_back( e );

	this->queueMutex.unlock();

	return true;
}

void
EventManager::processQueue()
{
	//sem_wait(this->queueSem);
	list<Event> eventQueueCpy( this->eventQueue );
	this->eventQueue.clear();
	//sem_post(this->queueSem);

	//sem_wait(this->registrationSem);
	list<Registration> regQueueCpy( this->regQueue );
	this->regQueue.clear();
	//sem_post(this->registrationSem);

	list<Registration>::iterator regIter = regQueueCpy.begin();
	while ( regIter != regQueueCpy.end() )
	{
		Registration& reg = *regIter++;

		map<EventID, list<IEventListener*>* >::iterator findIter = this->eventListeners.find( reg.eventID );
		if ( findIter != this->eventListeners.end() )
		{
			if ( false == reg.unregister )
			{
				findIter->second->push_back( reg.listener );
			}
			else
			{
				// TODO: implement unregister
			}
		}
		else
		{
			if ( false == reg.unregister )
			{
				list<IEventListener*>* listeners = new list<IEventListener*>();
				listeners->push_back( reg.listener );

				this->eventListeners.insert( make_pair( reg.eventID, listeners ) );
			}
		}
	}

	list<Event>::iterator eventIter = eventQueueCpy.begin();
	while ( eventIter != eventQueueCpy.end() )
	{
		/*
		timeval t;
		gettimeofday( &t, NULL );
		long int tNow = (t.tv_usec + 1000000 * t.tv_sec);

		if ( tNow - Core::getInstance().startTime >= 10000000)
		{
			Core::getInstance().stop();
			break;
		}
		*/

		Event& e = *eventIter++;

		// broadcast
		if ( e.getTarget() == 0 )
		{
			this->broadCast( e );
		}
		else
		{
			e.getTarget()->sendEvent( e );
		}
	}
}

void
EventManager::broadCast( Event& e )
{
	map<EventID, list<IEventListener*>* >::iterator findIter = this->eventListeners.find( e.getID() );
	if ( findIter != this->eventListeners.end() )
	{
		list<IEventListener*>* listeners = findIter->second;

		list<IEventListener*>::iterator listenersIter = listeners->begin();
		while ( listenersIter != listeners->end() )
		{
			(*listenersIter)->sendEvent( e );
			listenersIter++;
		}
	}
}
