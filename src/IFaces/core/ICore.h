/*
 * ICore.h
 *
 *  Created on: 7 Aug 2011
 *      Author: jonathan
 */

#ifndef ICORE_H_
#define ICORE_H_

#include "IGameObject.h"
#include "ISubSystem.h"
#include "IEventManager.h"

class ICore
{
	public:
		virtual ~ICore() {};

		virtual double getProcessingFactor() const = 0;

		virtual ISubSystem* getSubSystemByID( const std::string& ) = 0;
		virtual ISubSystem* getSubSystemByType( const std::string& ) = 0;

		virtual IGameObject* getObjectByName( const std::string& ) = 0;

		virtual IEventManager& getEventManager() const = 0;

};

#endif /* ICORE_H_ */
