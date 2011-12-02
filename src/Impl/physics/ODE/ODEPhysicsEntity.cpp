/*
 * ODEPhysicsEntity.cpp
 *
 *  Created on: Sep 30, 2010
 *      Author: jonathan
 */

#include "ODEPhysicsEntity.h"

#include <iostream>

using namespace std;

ODEPhysicsEntity::ODEPhysicsEntity( IGameObject* p )
	: IPhysicsEntity( p ),
	entityType( "physics" )
{
	this->physicType = 0;
}

ODEPhysicsEntity::~ODEPhysicsEntity()
{

}

void
ODEPhysicsEntity::update()
{
	dBodyCopyPosition( this->physicType->getBodyID(), this->pos );
	dBodyCopyRotation( this->physicType->getBodyID(), this->rot );
	memcpy( this->vel, dBodyGetLinearVel( this->physicType->getBodyID() ), 3 * sizeof( float ) );

	//cout << "Physics:" << this->getParent()->getName() << " has position of (" << this->pos[0] << "/" << this->pos[1] << "/" << this->pos[2] << ")" << endl;
}

bool
ODEPhysicsEntity::sendEvent( Event& e )
{
	if ( e == "moveForward" )
	{
		this->queuedEvents.push_back( e );
		return true;
	}

	return false;
}

