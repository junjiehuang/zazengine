/*
 * ODEPhysics.cpp
 *
 *  Created on: 06.07.2010
 *      Author: Jonathan Thaler
 */
#include "ODEPhysics.h"

#include <core/ICore.h>

#include "types/PhysicPlane.h"
#include "types/PhysicSphere.h"
#include "types/PhysicBox.h"

#include <errno.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include <iostream>

#define MAX_CONTACTS 3
#define GRAVITY_FACTOR 1

using namespace std;

ODEPhysics::ODEPhysics( const std::string& id, ICore* core )
	: id( id ),
	  type( "physics" ),
	  m_core( core )
{
}

ODEPhysics::~ODEPhysics()
{
}

bool
ODEPhysics::initialize( TiXmlElement* configElem )
{
	this->m_logger = this->m_core->getLogger( "zaZenODE" );
	this->m_logger->logInfo( "=============== ODEPhysics initializing... ===============" );

	if ( false == this->initODE( configElem ) )
	{
		return false;
	}

	this->m_logger->logInfo( "================ ODEPhysics initialized =================" );

	return true;
}

bool
ODEPhysics::shutdown()
{
	this->m_logger->logInfo( "=============== ODEPhysics shutting down... ===============" );

	std::list<ODEPhysicsEntity*>::iterator iter = this->entities.begin();
	while ( iter != this->entities.end() )
	{
		ODEPhysicsEntity* entity = *iter++;

		delete entity;
	}

	dWorldDestroy( this->worldID );
	dSpaceDestroy( this->spaceID );
	dCloseODE();

	this->m_logger->logInfo( "================ ODEPhysics shutdown =================" );

	return true;
}

bool
ODEPhysics::start()
{
	return true;
}

bool
ODEPhysics::stop()
{
	return true;
}

bool
ODEPhysics::pause()
{
	return true;
}

bool
ODEPhysics::process( double factor )
{
	this->doSimulation();

	return true;
}

bool
ODEPhysics::finalizeProcess()
{
	this->generateEvents();

	return true;
}

void
ODEPhysics::doSimulation()
{
	this->processEvents();

	dSpaceCollide( this->spaceID, this, ODEPhysics::collisionCallback );
	dWorldQuickStep( this->worldID, 0.001f );
	dJointGroupEmpty( this->contactGroupID );

	this->updateEntities();
}

void
ODEPhysics::generateEvents()
{
	// update of new positions
	std::list<ODEPhysicsEntity*>::iterator iter = this->entities.begin();
	while ( iter != this->entities.end() )
	{
		ODEPhysicsEntity* entity = *iter++;

		if ( false == entity->isStatic() )
		{
			const float* pos = entity->getPos();
			const float* rot = entity->getRot();
			const float* vel = entity->getVel();

			Event e( "UPDATE_PHYSICS" );
			e.setTarget( entity->getParent() );

			e.addValue( "pos", boost::any( pos ) );
			e.addValue( "rot", boost::any( rot ) );
			e.addValue( "vel", boost::any( vel ) );

			this->m_core->getEventManager().postEvent( e );
		}
	}
}

// this call should be guaranteed to come just from the main-thread during a sync-point
// when the event-manager dispatches the queue. never should one do a direct call
// to sendEvent, instead all events should be queued in the manager and be dispatched
// during the sync-point.
// => it is save to process receivedEvents in the thread and it is save to not protect
// it by semaphore because the thread does not process the queue during this call
bool
ODEPhysics::sendEvent( Event& e )
{
	this->receivedEvents.push_back( e );

	return true;
}

ODEPhysicsEntity*
ODEPhysics::createEntity( TiXmlElement* objectNode, IGameObject* parent )
{
	TiXmlElement* typeNode = objectNode->FirstChildElement( "type" );
	if ( 0 == typeNode )
	{
		this->m_logger->logError( "ODEPhysics::createEntity: no type-node found for physics-instance - ignoring object" );
		return NULL;
	}

	string typeID;
	const char* str = typeNode->Attribute( "id" );
	if ( 0 == str)
	{
		this->m_logger->logError( "ODEPhysics::createEntity: no id attribute found in physic type - ignoring object" );
		return NULL;
	}
	else
	{
		typeID = str;
	}

	float posX = 0.0;
	float posY = 0.0;
	float posZ = 0.0;
	float mass = 1.0f;
	bool staticFlag = false;

	str = typeNode->Attribute( "px" );
	if ( 0 != str )
	{
		posX = ( float ) atof( str );
	}

	str = typeNode->Attribute( "py" );
	if ( 0 != str )
	{
		posY = ( float ) atof( str );
	}

	str = typeNode->Attribute( "pz" );
	if ( 0 != str )
	{
		posZ = ( float ) atof( str );
	}

	str = typeNode->Attribute( "mass" );
	if ( 0 != str )
	{
		mass = ( float ) atof( str );
	}

	str = typeNode->Attribute( "static" );
	if ( 0 != str )
	{
		string caseStr = str;
		std::transform( caseStr.begin(), caseStr.end(), caseStr.begin(), tolower );
		if ( caseStr == "true" )
		{
			staticFlag = true;
		}
	}

	ODEPhysicsEntity* entity = new ODEPhysicsEntity( parent );

	if ( "SPHERE" == typeID )
	{
		float r = 0;

		str = typeNode->Attribute( "r" );
		if ( 0 != str )
		{
			r = ( float ) atof( str );
		}

		entity->m_physicType = new PhysicSphere( staticFlag, mass, r );
	}
	else if ( "BOX" == typeID )
	{
		float sx = 0;
		float sy = 0;
		float sz = 0;

		str = typeNode->Attribute( "sx" );
		if ( 0 != str )
		{
			sx = ( float ) atof( str );
		}

		str = typeNode->Attribute( "sy" );
		if ( 0 != str )
		{
			sy = ( float ) atof( str );
		}

		str = typeNode->Attribute( "sz" );
		if ( 0 != str )
		{
			sz = ( float ) atof( str );
		}

		entity->m_physicType = new PhysicBox( staticFlag, mass, sx, sy, sz );
	}
	else if ( "PLANE" == typeID )
	{
		float dx = 0;
		float dy = 0;
		float dz = 0;
		float d = 1;

		str = typeNode->Attribute( "dx" );
		if ( 0 != str )
		{
			dx = ( float ) atof( str );
		}

		str = typeNode->Attribute( "dy" );
		if ( 0 != str )
		{
			dy = ( float ) atof( str );
		}

		str = typeNode->Attribute( "dz" );
		if ( 0 != str )
		{
			dz = ( float ) atof( str );
		}

		str = typeNode->Attribute( "d" );
		if ( 0 != str )
		{
			d = ( float ) atof( str );
		}

		entity->m_physicType = new PhysicPlane( true, mass, dx, dy, dz, d );
	}

	entity->m_physicType->create( this->worldID, this->spaceID );
	entity->m_physicType->setPosition( posX, posY, posZ );

	this->entities.push_back( entity );

	return entity;
}

void
ODEPhysics::updateEntities()
{
	std::list<ODEPhysicsEntity*>::iterator iter = this->entities.begin();
	while ( iter != this->entities.end() )
	{
		ODEPhysicsEntity* entity = *iter++;

		if ( false == entity->isStatic() )
			entity->update();
	}
}

void
ODEPhysics::processEvents()
{
	std::list<Event>::iterator eventsIter = this->receivedEvents.begin();
	while ( eventsIter != this->receivedEvents.end() )
	{
		Event& e = *eventsIter++;
	}

	this->receivedEvents.clear();

	// process events of entities
	std::list<ODEPhysicsEntity*>::iterator iter = this->entities.begin();
	while ( iter != this->entities.end() )
	{
		ODEPhysicsEntity* entity = *iter++;

		eventsIter = entity->queuedEvents.begin();
		while ( eventsIter != entity->queuedEvents.end() )
		{
			Event& e = *eventsIter++;

			if ( e == "moveForward" )
			{
				// TODO: alter position
			}
		}

		entity->queuedEvents.clear();
	}
}

bool
ODEPhysics::initODE( TiXmlElement* configElem )
{
	float gravX = 0.0f;
	float gravY = -9.8f;
	float gravZ = 0.0f;

	TiXmlElement* gravityNode = configElem->FirstChildElement( "gravity" );
	if ( 0 != gravityNode )
	{
		const char* str = gravityNode->Attribute( "x" );
		if ( 0 != str)
		{
			gravX = ( float ) atof( str );
		}

		str = gravityNode->Attribute( "y" );
		if ( 0 != str)
		{
			gravY = ( float ) atof( str );
		}

		str = gravityNode->Attribute( "z" );
		if ( 0 != str)
		{
			gravZ = ( float ) atof( str );
		}
	}

	dInitODE();

	this->worldID = dWorldCreate();
	this->spaceID = dHashSpaceCreate( 0 );
	dWorldSetGravity( this->worldID, gravX, gravY, gravZ );
	dWorldSetCFM( this->worldID, 1e-5f );
	this->contactGroupID = dJointGroupCreate( 0 );

	return true;
}

void
ODEPhysics::collisionCallback( void* data, dGeomID o1, dGeomID o2 )
{
	ODEPhysics* instance = ( ODEPhysics* ) data;

	dBodyID b1 = dGeomGetBody( o1 );
    dBodyID b2 = dGeomGetBody( o2 );

	// ignore bodyless geometry (e.g. static)
	if ( !( b1 || b2 ) )
	{
		return;
	}

	dContact contacts [ MAX_CONTACTS ];
	for ( int i = 0; i < MAX_CONTACTS; i++ )
	{
		contacts[ i ].surface.mode = dContactBounce | dContactSoftCFM;
		contacts[ i ].surface.mu = dInfinity;
		contacts[ i ].surface.mu2 = 0;
		contacts[ i ].surface.bounce = 0.8f;
		contacts[ i ].surface.bounce_vel = 0.1f;
		contacts[ i ].surface.soft_cfm = 0.01f;
    }

	int totalContacts = dCollide( o1, o2, MAX_CONTACTS, &contacts[ 0 ].geom, sizeof( dContact ) );
	
	// no contacts
	if ( 0 == totalContacts )
	{
		return;
	}

	for ( int i = 0; i < totalContacts; i++ )
	{
		// dJointCreateContact needs to know which world and joint group to work with as well as the dContact
		// object itself. It returns a new dJointID which we then use with dJointAttach to finally create the
		// temporary contact joint between the two geom bodies.
		dJointID c = dJointCreateContact( instance->worldID, instance->contactGroupID, contacts + i );
		dJointAttach( c, b1, b2 );
	}

	long long currentMillis = instance->m_core->getCurrentMillis();

	std::list<ODEPhysicsEntity*>::iterator iter = instance->entities.begin();
	while ( iter != instance->entities.end() )
	{
		ODEPhysicsEntity* entity = *iter++;

		if ( b1 == entity->getBodyId() )
		{
			const float* vel = entity->getVel();
			long long lastCollDelta = currentMillis - entity->getLastCollTs();
			float velLength = sqrt( pow( vel[ 0 ], 2 ) + pow( vel[ 1 ], 2 ) + pow( vel[ 2 ], 2 ) );

			// only create collision-event when there was enough time since the last and the object is already moving
			if ( lastCollDelta > 250 && velLength > 1.0 )
			{
				entity->setLastCollTs( currentMillis );

				Event e( "COLLIDES_WITH" );
				e.setTarget( entity->getParent() );

				instance->m_core->getEventManager().postEvent( e );
			}

			break;
		}
	}
}

extern "C"
{	
	__declspec( dllexport ) ISubSystem*
	createInstance ( const char* id, ICore* core )
	{
		return new ODEPhysics( id, core );
	}

	__declspec( dllexport ) void
	deleteInstance ( ISubSystem* subSys )
	{
		if ( 0 == subSys )
			return;

		if ( 0 == dynamic_cast<ODEPhysics*>( subSys ) )
			return;

		delete subSys;
	}
}