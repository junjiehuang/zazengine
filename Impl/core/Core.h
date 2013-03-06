/*
 * PlayGround.h
 *
 *  Created on: 27.06.2010
 *      Author: joni
 */

#ifndef CORE_H_
#define CORE_H_

#ifdef DLL_EXPORTS
#define DLL_API __declspec(dllexport)
#else
#define DLL_API __declspec(dllimport)
#endif

#include "ZazenGameObjectFactory.h"
#include "ZazenSubSystemFactory.h"
#include "EventManager.h"

#include <core/ICore.h>

#include <list>

/* BIG TARGET: Discrete Realtime Autonomus Entity Simulation
 *
 * TODO: clear, powerful and flexible event-system
 * TODO: solve how input moves around
 * TODO: integrate event-system with LUA-scripting
 *
 * TODO: client-server model for world
 *
 */
class DLL_API Core : public ICore
{
	public:
		static bool initalize( const std::string& );
		static bool shutdown();
		static Core& getInstance() { return *Core::instance; };

		void start();
		void stop();

		double getProcessingFactor() const { return this->m_processingFactor; };

		ISubSystem* getSubSystemByID( const std::string& );
		ISubSystem* getSubSystemByType( const std::string& );

		IGameObject* getObjectByName( const std::string& );

		IEventManager& getEventManager() const { return *this->m_eventManager; };

	private:
		static Core* instance;

		double m_processingFactor;

		bool m_runCore;

		EventManager* m_eventManager;

		ZazenGameObjectFactory* m_gameObjectFactory;
		ZazenSubSystemFactory* m_subSystemFactory;

		std::list<ISubSystem*> m_subSystems;
		std::list<IGameObject*> m_gameObjects;

		Core();
		virtual ~Core();

		bool loadConfig( const std::string& );
		ISubSystem* loadSubSystem( const std::string&, const std::string& );
};

#endif /* CORE_H_ */