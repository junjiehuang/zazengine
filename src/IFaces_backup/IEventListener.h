/*
 * IEventListener.h
 *
 *  Created on: 02.07.2010
 *      Author: joni
 */

#ifndef IEVENTLISTENER_H_
#define IEVENTLISTENER_H_

#include "Event.h"

class IEventListener
{
	public:
		virtual ~IEventListener() {};

		//virtual const std::string& getID() const = 0;
		virtual bool sendEvent( Event& e ) = 0;
};

#endif /* IEVENTLISTENER_H_ */
