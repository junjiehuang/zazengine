/*
 * PlayGround.h
 *
 *  Created on: 27.06.2010
 *      Author: Jonathan Thaler
 */

#ifndef CORE_H_
#define CORE_H_

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
		static ICore* getInstance() { return Core::instance; };
		static Core& getRef() { return *Core::instance; };

		void start();
		void stop();

		float getProcessingFactor() const { return this->m_processingFactor; };

		long long getCurrentMillis() const;

		IAi* getAi() { return this->m_ai; };
		IAudio* getAudio() { return this->m_audio; };
		IGraphics* getGraphics() { return this->m_graphics; };
		IInput* getInput() { return this->m_input; };
		IPhysics* getPhysics() { return this->m_physics; };

		ISubSystem* getSubSystemByID( const std::string& );
		ISubSystem* getSubSystemByType( const std::string& );

		IGameObject* getObjectByName( const std::string& );

		IEventManager& getEventManager() const { return *this->m_eventManager; };

	private:
		static Core* instance;

		float m_processingFactor;

		bool m_runCore;

		EventManager* m_eventManager;

		IAi* m_ai;
		IAudio* m_audio;
		IGraphics* m_graphics;
		IInput* m_input;
		IPhysics* m_physics;

		ZazenGameObjectFactory* m_gameObjectFactory;
		ZazenSubSystemFactory* m_subSystemFactory;

		std::list<ISubSystem*> m_subSystems;
		std::list<IGameObject*> m_gameObjects;

		Core();
		virtual ~Core();

		bool loadConfig( const std::string& );
		ISubSystem* loadSubSystem( const std::string&, const std::string& );
		bool checkSubSystemType( ISubSystem* ); 
};

#endif /* CORE_H_ */
