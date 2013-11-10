#include "ODEPhysicsEntity.h"

#include <iostream>

using namespace std;
using namespace boost;

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
	if ( e == "POSITION_CHANGED" )
	{
		const float* matrix = any_cast<const float*>( e.getValue( "matrix" ) );
		this->m_physicType->setPosition( matrix[ 12 ], matrix[ 13 ], matrix[ 14 ] );
	}

	return false;
}

