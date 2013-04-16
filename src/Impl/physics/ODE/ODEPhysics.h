/*
 * ODEPhysics.h
 *
 *  Created on: 06.07.2010
 *      Author: Jonathan Thaler
 */

#ifndef ODEPHYSICS_H_
#define ODEPHYSICS_H_

#include "ODEPhysicsEntity.h"

#include <core/ICore.h>
#include <core/ILogger.h>

#include <physics/IPhysics.h>
#include <ode/ode.h>
#include <list>

class ODEPhysics : public IPhysics
{
	public:
		ODEPhysics( const std::string& id, ICore* core );
		virtual ~ODEPhysics();

		const std::string& getID() const { return this->id; };
		const std::string& getType() const { return this->type; };

		bool isAsync() const { return true; };

		bool start();
		bool stop();
		bool pause();

		bool initialize( TiXmlElement* );
		bool shutdown();

		bool process( double );
		bool finalizeProcess();

		bool sendEvent( Event& e );

		ODEPhysicsEntity* createEntity( TiXmlElement*, IGameObject* parent );

	private:
		std::string id;
		std::string type;

		dWorldID worldID;
		dSpaceID spaceID;
		dJointGroupID contactGroupID;

		std::list<Event> receivedEvents;
		std::list<ODEPhysicsEntity*> entities;

		ICore* m_core;
		ILogger* m_logger;

		bool initODE( TiXmlElement* );

		void updateEntities();
		void processEvents();
		void doSimulation();
		void generateEvents();

		static void collisionCallback( void *data, dGeomID o1, dGeomID o2 );
};

#endif /* PLAYGROUNDPHYSICS_H_ */
