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
	Registration reg;
	reg.listener = listener;
	reg.eventID = eventID;
	reg.unregister = false;

	this->regQueue.push_back( reg );

	return true;
}

bool
EventManager::unregisterForEvent( EventID eventID, IEventListener* listener )
{
	Registration reg;
	reg.listener = listener;
	reg.eventID = eventID;
	reg.unregister = true;

	this->regQueue.push_back( reg );

	return true;
}

bool
EventManager::postEvent( const Event& e )
{
	this->eventQueue.push_back( e );

	return true;
}

void
EventManager::processQueue()
{
	list<Event> eventQueueCpy( this->eventQueue );
	this->eventQueue.clear();

	list<Registration> regQueueCpy( this->regQueue );
	this->regQueue.clear();

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
