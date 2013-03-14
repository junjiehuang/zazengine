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
}

bool
ODEPhysicsEntity::sendEvent( Event& e )
{
	return false;
}

