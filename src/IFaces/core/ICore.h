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

#include "../audio/IAudio.h"
#include "../graphics/IGraphics.h"
#include "../input/IInput.h"
#include "../physics/IPhysics.h"

class ICore
{
	public:
		virtual ~ICore() {};

		virtual void stop() = 0;

		virtual float getProcessingFactor() const = 0;

		virtual long long getCurrentMillis() const = 0;

		virtual void logError( const std::string& ) const = 0;
		virtual void logError( const std::ostream& ) const = 0;

		virtual void logWarning( const std::string& ) const = 0;
		virtual void logWarning( const std::ostream& ) const = 0;

		virtual void logInfo( const std::string& ) const = 0;
		virtual void logInfo( const std::ostream& ) const = 0;

		virtual void logDebug( const std::string& ) const = 0;
		virtual void logDebug( const std::ostream& ) const = 0;

		virtual IAudio* getAudio() = 0;
		virtual IGraphics* getGraphics() = 0;
		virtual IInput* getInput() = 0;
		virtual IPhysics* getPhysics() = 0;

		// TODO do we need those two?
		virtual ISubSystem* getSubSystemByID( const std::string& ) = 0;
		virtual ISubSystem* getSubSystemByType( const std::string& ) = 0;

		virtual IGameObject* getObjectByName( const std::string& ) = 0;

		virtual IEventManager& getEventManager() const = 0;

};

#endif /* ICORE_H_ */
