/*
 * ODEPhysics.cpp
 *
 *  Created on: 06.07.2010
 *      Author: joni
 */

#include "ODEPhysics.h"

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

ODEPhysics::ODEPhysics()
	: id("ODEPhysics"),
	  type("physics")
{
	this->sem = 0;
	this->thread = 0;

	this->doProcessing = false;
	this->runThread = false;
}

ODEPhysics::~ODEPhysics()
{

}

bool
ODEPhysics::initialize( TiXmlElement* )
{
	cout << endl << "=============== ODEPhysics initializing... ===============" << endl;

	dInitODE();

	this->worldID = dWorldCreate();
	dWorldSetGravity(this->worldID, 0.0f, -9.8f * GRAVITY_FACTOR, 0.0f);
	this->spaceID = dHashSpaceCreate(0);
	this->contactGroupID = dJointGroupCreate(0);

	this->sem = new Semaphore();
	this->sem->grab();

	this->thread = new Thread();
	this->thread->start( ODEPhysics::threadFunc, this );

	cout << "================ ODEPhysics initialized =================" << endl;

	return true;
}

bool
ODEPhysics::shutdown()
{
	cout << endl << "=============== ODEPhysics shutting down... ===============" << endl;

	this->doProcessing = false;
	this->runThread = false;
	this->sem->release();

	this->thread->join();
	delete this->thread;
	this->thread = 0;

	delete this->sem;
	this->sem = 0;

	std::list<ODEPhysicsEntity*>::iterator iter = this->entities.begin();
	while ( iter != this->entities.end() )
	{
		ODEPhysicsEntity* entity = *iter++;

		delete entity;
	}

	dWorldDestroy(this->worldID);
	dSpaceDestroy(this->spaceID);
	dCloseODE();

	cout << "================ ODEPhysics shutdown =================" << endl;

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
ODEPhysics::process(double factor)
{
	//cout << "ODEPhysics::process" << endl;

	this->doProcessing = true;
	this->sem->release();

	return true;
}

bool
ODEPhysics::finalizeProcess()
{
	//cout << "ODEPhysics::finalizeProcess" << endl;

	this->doProcessing = false;
	this->sem->grab();

	std::list<ODEPhysicsEntity*>::iterator iter = this->entities.begin();
	while ( iter != this->entities.end() )
	{
		ODEPhysicsEntity* entity = *iter++;

		const dReal* pos = dBodyGetPosition( entity->physicType->getBodyID() );
		const dReal* rot = dBodyGetRotation( entity->physicType->getBodyID() );
		const dReal* vel = dBodyGetLinearVel( entity->physicType->getBodyID() );

		entity->pos[0] = pos[0];
		entity->pos[1] = pos[1];
		entity->pos[2] = pos[2];

		for (int i = 0; i < 12; i++)
			entity->rot[i] = rot[i];

		entity->vel[0] = vel[0];
		entity->vel[1] = vel[1];
		entity->vel[2] = vel[2];

		entity->processConsumers();
	}

	return true;
}

bool
ODEPhysics::sendEvent(const Event& e)
{
	return true;
}

ODEPhysicsEntity*
ODEPhysics::createEntity( TiXmlElement* objectNode )
{
	TiXmlElement* typeNode = objectNode->FirstChildElement( "type" );
	if ( 0 == typeNode )
	{
		cout << "ERROR ... no type-node found for physics-instance - ignoring object" << endl;
		return 0;
	}

	string typeID;
	const char* str = typeNode->Attribute( "id" );
	if ( 0 == str)
	{
		cout << "ERROR ... no id attribute found in physic type - ignoring object" << endl;
		return 0;
	}
	else
	{
		typeID = str;
	}

	PhysicType* type = 0;

	if ( "SPHERE" == typeID )
	{
		type = new PhysicSphere( false, 1, 1 );
	}
	else if ( "BOX" == typeID )
	{
		type = new PhysicBox( false, 1, 1, 1, 1 );
	}
	else if ( "PLANE" == typeID )
	{
		type = new PhysicPlane( false, 1, 0, 0, 0, 1 );
	}

	type->create( this->worldID, this->spaceID );

	ODEPhysicsEntity* entity = new ODEPhysicsEntity();
	entity->physicType = type;

	memset( entity->pos, 0, sizeof( entity->pos ) );
	memset( entity->rot, 0, sizeof( entity->rot ) );
	memset( entity->vel, 0, sizeof( entity->vel ) );

	return entity;
}

void
ODEPhysics::processInternal()
{
	this->sem->grab();

	while ( this->doProcessing )
	{
		for (int i = 0; i < 10; i++) {
			dSpaceCollide(this->spaceID, this, ODEPhysics::collisionCallback);
			dWorldQuickStep(this->worldID, 0.001);
			dJointGroupEmpty(this->contactGroupID);
		}
	}

/*
	memcpy(node->matrix.data, pRotMatrix, 11 * sizeof(float));
	memcpy(&node->matrix.data[12], pPos, 3 * sizeof(float));
*/

	this->sem->release();
}

void
ODEPhysics::collisionCallback(void* data, dGeomID o1, dGeomID o2)
{
	ODEPhysics* instance = (ODEPhysics*) data;

	dContact contacts[MAX_CONTACTS];
	for (int i = 0; i < MAX_CONTACTS; i++) {
		contacts[i].surface.mode = dContactBounce | dContactSoftCFM;
		contacts[i].surface.mu = dInfinity;
		contacts[i].surface.mu2 = 0;
		contacts[i].surface.bounce = 0.8;
		contacts[i].surface.bounce_vel = 0.1;
		contacts[i].surface.soft_cfm = 0.01;
    }

	if (int totalContacts = dCollide(o1, o2, MAX_CONTACTS, &contacts[0].geom, sizeof(dContactGeom))) {
		dBodyID b1 = dGeomGetBody(o1);
		dBodyID b2 = dGeomGetBody(o2);

		for (int i = 0; i < totalContacts; i++) {
			// dJointCreateContact needs to know which world and joint group to work with as well as the dContact
			// object itself. It returns a new dJointID which we then use with dJointAttach to finally create the
			// temporary contact joint between the two geom bodies.
			dJointID c = dJointCreateContact(instance->worldID, instance->contactGroupID, contacts + i);

			dJointAttach(c, b1, b2);
		}
	}
}

void*
ODEPhysics::threadFunc(void* args)
{
	ODEPhysics* instance = (ODEPhysics*) args;

	cout << "ODEPhysics " << instance->id << " is up and processing" << endl;

	instance->runThread = true;
	while ( instance->runThread )
	{
		instance->processInternal();
	}

	cout << "ODEPhysics " << instance->id << " finished thread" << endl;

	return 0;
}
