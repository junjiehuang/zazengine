/*
 * PlayGroundRendererEntity.cpp
 *
 *  Created on: 09.07.2010
 *      Author: joni
 */

#include "ZazenGraphicsEntity.h"

#include <glm/gtc/type_ptr.hpp>

#include <iostream>

using namespace std;

ZazenGraphicsEntity::ZazenGraphicsEntity( IGameObject* p )
	: IGraphicsEntity( p ),
	m_type( "graphics" )
{
	this->m_orientation = 0;
}

ZazenGraphicsEntity::~ZazenGraphicsEntity()
{
}

bool
ZazenGraphicsEntity::sendEvent( Event& e )
{
	// process immediately because no call to graphics api, just coping of data (yet)
	if ( e == "updatePhysics" )
	{
		Value& pos = e.getValue( "pos" );
		Value& rot = e.getValue( "rot" );

		this->setOrientation( pos.data, rot.data );

		return true;
	}
	else if  ( e == "SDLK_RIGHT" )
	{
		this->m_orientation->changeHeading( -0.1 * this->lastItFact );
	}
	else if ( e == "SDLK_LEFT" )
	{
		this->m_orientation->changeHeading( 0.1 * this->lastItFact );
	}
	else if ( e == "SDLK_UP" )
	{
		this->m_orientation->changePitch( -0.1 * this->lastItFact );
	}
	else if ( e == "SDLK_DOWN" )
	{
		this->m_orientation->changePitch( 0.1 * this->lastItFact );
	}
	else if ( e == "SDLK_w" )
	{
		this->m_orientation->strafeForward( -0.1 * this->lastItFact );
	}
	else if ( e == "SDLK_s" )
	{
		this->m_orientation->strafeForward( 0.1 * this->lastItFact );
	}
	else if ( e == "SDLK_d" )
	{
		this->m_orientation->changeRoll( -0.1 * this->lastItFact );
	}
	else if ( e == "SDLK_a" )
	{
		this->m_orientation->changeRoll( 0.1 * this->lastItFact );
	}

	return false;
}

void
ZazenGraphicsEntity::setOrientation( const float* pos, const float* rot)
{
	// do through m_orientation because need notification of matrix changed
	memcpy( glm::value_ptr( this->instance->modelMatrix ), rot, 11 * sizeof( float ) );
	memcpy( &glm::value_ptr( this->instance->modelMatrix )[12], pos, 3 * sizeof( float ) );
}
