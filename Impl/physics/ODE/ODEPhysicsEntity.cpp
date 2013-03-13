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
	m_entityType( "physics" )
{
	this->m_physicType = 0;
}

ODEPhysicsEntity::~ODEPhysicsEntity()
{

}

void
ODEPhysicsEntity::update()
{
	dBodyCopyPosition( this->m_physicType->getBodyID(), this->m_pos );
	dBodyCopyRotation( this->m_physicType->getBodyID(), this->m_rot );
	memcpy( this->m_vel, dBodyGetLinearVel( this->m_physicType->getBodyID() ), 3 * sizeof( float ) );

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

