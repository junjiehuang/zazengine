/*
 * PlayGroundRendererEntity.cpp
 *
 *  Created on: 09.07.2010
 *      Author: Jonathan Thaler
 */

#include "ZazenGraphicsEntity.h"
#include "ZazenGraphics.h"

#include <glm/gtc/type_ptr.hpp>

#include <iostream>

using namespace std;
using namespace boost;

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
		boost::any& pos = e.getValue( "pos" );
		boost::any& rot = e.getValue( "rot" );

		this->setOrientation( any_cast<const float*>( pos ), any_cast<const float*>( rot ) );

		return true;
	}
	else if ( e == "KEY_PRE_Q" )
	{
		this->m_orientation->changeRoll( 50.0f * ZazenGraphics::getInstance().getCore().getProcessingFactor() );
	}
	else if ( e == "KEY_PRE_E" )
	{
		this->m_orientation->changeRoll( -50.0f * ZazenGraphics::getInstance().getCore().getProcessingFactor() );
	}
	else if ( e == "KEY_PRE_W" )
	{
		this->m_orientation->strafeForward( -100.0f * ZazenGraphics::getInstance().getCore().getProcessingFactor() );
	}
	else if ( e == "KEY_PRE_S" )
	{
		this->m_orientation->strafeForward( 100.0f * ZazenGraphics::getInstance().getCore().getProcessingFactor() );
	}
	else if ( e == "KEY_PRE_A" )
	{
		this->m_orientation->strafeRight( -100.0f * ZazenGraphics::getInstance().getCore().getProcessingFactor() );
	}
	else if ( e == "KEY_PRE_D" )
	{
		this->m_orientation->strafeRight( 100.0f * ZazenGraphics::getInstance().getCore().getProcessingFactor() );
	}
	else if ( e == "MOUSE_MOVE" )
	{
		int x = any_cast<int>( e.getValue( "x" ) );
		int y = any_cast<int>( e.getValue( "y" ) );

		this->m_orientation->changeHeading( -50.0f * x * ZazenGraphics::getInstance().getCore().getProcessingFactor() );
		this->m_orientation->changePitch( 50.0f * y * ZazenGraphics::getInstance().getCore().getProcessingFactor() );
	}

	return false;
}

void
ZazenGraphicsEntity::setOrientation( const float* pos, const float* rot)
{
	this->m_orientation->setRaw( rot, pos );
}
